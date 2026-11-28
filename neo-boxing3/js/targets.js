import { AppState, hexToRgba } from './state.js';
import { PUNCH_TYPES } from './config.js';
import { particleEngine } from './particles.js';
import { soundEngine } from './sound.js';

class TargetManager {
  constructor() {
    this.targets = [];
  }

  reset(count = 5) {
    this.targets = [];
    this.populateTargets(count);
  }

  populateTargets(count) {
    const columns = 3;
    const rows = 3;
    const marginX = 420;
    const topY = 250;
    const bottomY = 740;
    const widthStep = (1920 - marginX * 2) / (columns - 1);
    const heightStep = (bottomY - topY) / (rows - 1);

    const possiblePositions = [];
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < columns; c++) {
        possiblePositions.push({
          x: marginX + c * widthStep,
          y: topY + r * heightStep
        });
      }
    }

    possiblePositions.sort(() => Math.random() - 0.5);

    for (let i = 0; i < Math.min(count, possiblePositions.length); i++) {
      this.targets.push(this.createTarget(possiblePositions[i].x, possiblePositions[i].y));
    }
  }

  createTarget(x, y) {
    const requiredPunch = AppState.gameMode === 'DRILL' ? AppState.currentDrillPunch : (Math.random() < 0.6 ? 'ANY' : PUNCH_TYPES[Math.floor(Math.random() * PUNCH_TYPES.length)]);
    return {
      id: Math.random(),
      x,
      y,
      radius: 80,
      active: true,
      requiredPunch,
      pulseAngle: Math.random() * Math.PI * 2,
      cooldown: 0
    };
  }

  drawTargets(ctx, theme) {
    for (const target of this.targets) {
      if (!target.active) continue;

      if (target.cooldown > 0) {
        target.cooldown--;
      }

      target.pulseAngle += 0.05;
      const currentRadius = target.radius + Math.sin(target.pulseAngle) * 6;

      const colorToUse = AppState.targetColor || theme.targetColor;

      ctx.save();
      ctx.translate(target.x, target.y);

      // Outer Pulsing Glow
      ctx.shadowColor = colorToUse;
      ctx.shadowBlur = 25;

      // Outer Ring
      ctx.beginPath();
      ctx.arc(0, 0, currentRadius, 0, Math.PI * 2);
      ctx.strokeStyle = colorToUse;
      ctx.lineWidth = 6;
      ctx.stroke();

      // Inner Translucent Fill
      ctx.fillStyle = hexToRgba(colorToUse, 0.35);
      ctx.beginPath();
      ctx.arc(0, 0, currentRadius * 0.75, 0, Math.PI * 2);
      ctx.fill();

      // Uploaded image overlay, clipped to the target face
      this.drawTargetImage(ctx, currentRadius * 0.75);

      ctx.restore();
    }
  }

  drawTargetImage(ctx, radius) {
    const img = AppState.targetImage;
    if (!img) return;

    const imgW = img.naturalWidth || img.width;
    const imgH = img.naturalHeight || img.height;
    if (!imgW || !imgH) return;

    ctx.save();
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.clip();

    // Cover-fit the uploaded image inside the circular target face.
    const scale = Math.max((radius * 2) / imgW, (radius * 2) / imgH);
    const drawW = imgW * scale;
    const drawH = imgH * scale;
    ctx.globalAlpha = 0.92;
    ctx.drawImage(img, -drawW / 2, -drawH / 2, drawW, drawH);
    ctx.restore();
  }

  // Power multiplier: 1x at 0 power up to 2x at 100 power.
  calculatePoints(power) {
    const clampedPower = Math.max(0, Math.min(100, power));
    const powerMultiplier = 1 + (clampedPower / 100);
    const basePoints = 100 + (AppState.combo * 25);
    return Math.round(basePoints * powerMultiplier);
  }

  checkPunchCollision(gloveX, gloveY, gloveRadius, punchType, power) {
    for (const target of this.targets) {
      if (!target.active || target.cooldown > 0) continue;

      const dx = gloveX - target.x;
      const dy = gloveY - target.y;
      const distance = Math.sqrt(dx * dx + dy * dy);

      const maxImpactDist = target.radius + gloveRadius;

      if (distance <= maxImpactDist) {
        if (target.requiredPunch === 'ANY' || target.requiredPunch === punchType || AppState.gameMode !== 'DRILL') {
          
          particleEngine.spawnExplosion(target.x, target.y, 40, AppState.activeThemeKey, power);
          soundEngine.playTargetHit(power);
          
          AppState.successfulHits++;
          AppState.combo++;
          if (AppState.combo > AppState.maxCombo) AppState.maxCombo = AppState.combo;
          
          AppState.score += this.calculatePoints(power);
          AppState.recentPower = Math.floor(power);
          if (power > AppState.maxPower) AppState.maxPower = Math.floor(power);

          soundEngine.playComboBeep(AppState.combo);

          this.respawnTarget(target);

          if (AppState.audioProfile === 'REVV' && power > 50) {
            document.getElementById('game-container').classList.add('shake');
            setTimeout(() => document.getElementById('game-container').classList.remove('shake'), 250);
          }

          return true;
        }
      }
    }
    return false;
  }

  respawnTarget(target) {
    target.x = 350 + Math.random() * (1920 - 700);
    target.y = 200 + Math.random() * (1080 - 400);
    target.requiredPunch = AppState.gameMode === 'DRILL' ? AppState.currentDrillPunch : (Math.random() < 0.6 ? 'ANY' : PUNCH_TYPES[Math.floor(Math.random() * PUNCH_TYPES.length)]);
    target.cooldown = 10;
    soundEngine.playTargetSpawn();
  }
}

export const targetManager = new TargetManager();