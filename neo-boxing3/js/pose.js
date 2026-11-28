import { AppState, hexToRgba } from './state.js';
import { targetManager } from './targets.js';

class PoseTracker {
  constructor() {
    this.detector = null;
    this.videoEl = document.getElementById('webcam-video');
    this.rawPoses = [];
    
    // Smoothed Wrist Positions for display & collision
    this.leftWristSmoothed = { x: 960, y: 540, rawX: 0.5, rawY: 0.5, vx: 0, vy: 0 };
    this.rightWristSmoothed = { x: 960, y: 540, rawX: 0.5, rawY: 0.5, vx: 0, vy: 0 };

    this.leftWristHistory = [];
    this.rightWristHistory = [];
    this.isReady = false;
    this.lastFrameTime = Date.now();
  }

  async init() {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({
        video: { width: 1280, height: 720, facingMode: 'user' },
        audio: false
      });
      this.videoEl.srcObject = stream;
      await this.videoEl.play();

      const detectorConfig = {
        modelType: poseDetection.movenet.modelType.SINGLEPOSE_LIGHTNING
      };
      this.detector = await poseDetection.createDetector(
        poseDetection.SupportedModels.MoveNet,
        detectorConfig
      );

      this.isReady = true;
      this.detectLoop();
    } catch (err) {
      console.warn("Camera or TensorFlow MoveNet setup issue. Touch/click fallback ready.", err);
    }
  }

  async detectLoop() {
    if (this.detector && this.videoEl.readyState >= 2) {
      try {
        const poses = await this.detector.estimatePoses(this.videoEl);
        this.rawPoses = poses;
        if (poses.length > 0) {
          this.processPose(poses[0]);
        }
      } catch (e) {
        console.error("Pose detection loop error:", e);
      }
    }
    requestAnimationFrame(() => this.detectLoop());
  }

  processPose(pose) {
    if (!pose || !pose.keypoints) return;
    const kps = pose.keypoints;
    const now = Date.now();
    const dt = Math.max(0.001, (now - this.lastFrameTime) / 1000);
    this.lastFrameTime = now;

    const vidW = this.videoEl.videoWidth || 1280;
    const vidH = this.videoEl.videoHeight || 720;

    const lw = kps[9];  // Left Wrist
    const rw = kps[10]; // Right Wrist

    // Process Left Wrist
    if (lw && lw.score > 0.25) {
      const normX = 1 - (lw.x / vidW);
      const normY = lw.y / vidH;
      this.updateWristState(this.leftWristSmoothed, normX, normY, dt);
      this.updatePunchVelocity(this.leftWristHistory, this.leftWristSmoothed, 'LEFT', now);
    }

    // Process Right Wrist
    if (rw && rw.score > 0.25) {
      const normX = 1 - (rw.x / vidW);
      const normY = rw.y / vidH;
      this.updateWristState(this.rightWristSmoothed, normX, normY, dt);
      this.updatePunchVelocity(this.rightWristHistory, this.rightWristSmoothed, 'RIGHT', now);
    }
  }

  updateWristState(wristObj, normX, normY, dt) {
    wristObj.rawX = normX;
    wristObj.rawY = normY;

    // Map directly onto the canvas so the glove sits on the detected wrist.
    const targetX = normX * 1920;
    const targetY = normY * 1080;

    // Higher alpha keeps the rendered glove tight to the wrist while still
    // smoothing out camera jitter.
    const alpha = 0.65;
    const prevX = wristObj.x;
    const prevY = wristObj.y;

    wristObj.x = prevX + alpha * (targetX - prevX);
    wristObj.y = prevY + alpha * (targetY - prevY);

    wristObj.vx = (wristObj.x - prevX) / dt;
    wristObj.vy = (wristObj.y - prevY) / dt;
  }

  updatePunchVelocity(history, wristObj, handLabel, now) {
    history.push({ x: wristObj.x, y: wristObj.y, timestamp: now });
    if (history.length > 5) history.shift();

    if (history.length < 3) return;

    const first = history[0];
    const last = history[history.length - 1];
    const dt = (last.timestamp - first.timestamp) / 1000;
    if (dt <= 0) return;

    const dx = last.x - first.x;
    const dy = last.y - first.y;
    const speed = Math.sqrt(dx * dx + dy * dy) / dt;

    if (speed > 800) {
      let punchType = 'JAB';
      if (Math.abs(dy) > Math.abs(dx) * 1.4 && dy < 0) {
        punchType = 'UPPERCUT';
      } else if (Math.abs(dx) > Math.abs(dy) * 1.5) {
        punchType = 'HOOK';
      }

      const power = Math.min(100, Math.max(30, (speed / 2800) * 100));

      if (AppState.currentScreen === 'PLAYING') {
        const hit = targetManager.checkPunchCollision(
          wristObj.x,
          wristObj.y,
          AppState.gloveRadius,
          punchType,
          power
        );

        if (hit) {
          AppState.totalPunches++;
          history.length = 0;
        }
      }
    }
  }

  drawSkeleton(ctx, opacity = 1.0) {
    if (this.rawPoses.length === 0) return;
    const pose = this.rawPoses[0];
    if (!pose || !pose.keypoints) return;

    const vidW = this.videoEl.videoWidth || 1280;
    const vidH = this.videoEl.videoHeight || 720;

    ctx.save();
    ctx.globalAlpha = opacity;

    const adjacentPairs = poseDetection.util.getAdjacentPairs(poseDetection.SupportedModels.MoveNet);
    
    ctx.lineWidth = 4;
    ctx.strokeStyle = '#06b6d4';
    ctx.shadowColor = '#06b6d4';
    ctx.shadowBlur = 10;

    for (const pair of adjacentPairs) {
      const kp1 = pose.keypoints[pair[0]];
      const kp2 = pose.keypoints[pair[1]];

      if (kp1.score > 0.25 && kp2.score > 0.25) {
        const x1 = (1 - (kp1.x / vidW)) * 1920;
        const y1 = (kp1.y / vidH) * 1080;
        const x2 = (1 - (kp2.x / vidW)) * 1920;
        const y2 = (kp2.y / vidH) * 1080;

        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
      }
    }

    for (const kp of pose.keypoints) {
      if (kp.score > 0.25) {
        const mappedX = (1 - (kp.x / vidW)) * 1920;
        const mappedY = (kp.y / vidH) * 1080;

        ctx.beginPath();
        ctx.arc(mappedX, mappedY, 7, 0, Math.PI * 2);
        ctx.fillStyle = '#ffffff';
        ctx.shadowColor = '#00f0ff';
        ctx.shadowBlur = 12;
        ctx.fill();
        ctx.strokeStyle = '#06b6d4';
        ctx.lineWidth = 2;
        ctx.stroke();
      }
    }

    ctx.restore();
  }

  drawGloveIndicators(ctx, opacity = 1.0) {
    ctx.save();
    ctx.globalAlpha = opacity;

    const r = AppState.gloveRadius;

    if (this.leftWristSmoothed) {
      const lColor = AppState.leftGloveColor || '#ef4444';
      this.renderGloveRing(ctx, this.leftWristSmoothed.x, this.leftWristSmoothed.y, r, lColor, hexToRgba(lColor, 0.35), 'L');
    }

    if (this.rightWristSmoothed) {
      const rColor = AppState.rightGloveColor || '#06b6d4';
      this.renderGloveRing(ctx, this.rightWristSmoothed.x, this.rightWristSmoothed.y, r, rColor, hexToRgba(rColor, 0.35), 'R');
    }

    ctx.restore();
  }

  renderGloveRing(ctx, x, y, radius, color, fillColor, label) {
    ctx.save();
    ctx.translate(x, y);

    const isBlack = color === '#000000';
    const strokeColor = isBlack ? '#ffffff' : color;
    const shadowColor = isBlack ? '#ffffff' : color;

    ctx.shadowColor = shadowColor;
    ctx.shadowBlur = 20;

    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = 5;
    ctx.stroke();

    ctx.fillStyle = isBlack ? 'rgba(0, 0, 0, 0.85)' : fillColor;
    ctx.beginPath();
    ctx.arc(0, 0, radius * 0.85, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = isBlack ? '#00f0ff' : '#ffffff';
    ctx.beginPath();
    ctx.arc(0, 0, 8, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = isBlack ? '#00f0ff' : '#ffffff';
    ctx.font = '900 18px Orbitron';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, 0, 0);

    ctx.restore();
  }
}

export const poseTracker = new PoseTracker();