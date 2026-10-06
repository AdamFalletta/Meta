import { AppState, hexToRgba } from './state.js';
import { PUNCH_TYPES } from './config.js';
import { particleEngine } from './particles.js';
import { soundEngine } from './sound.js';

class TargetManager {
  constructor() {
    this.targets = [];
    this.reachCornerTargets = [
      { id: 'topLeft', name: 'TOP LEFT', x: 250, y: 220, hit: false },
      { id: 'topRight', name: 'TOP RIGHT', x: 1670, y: 220, hit: false },
      { id: 'bottomLeft', name: 'BOTTOM LEFT', x: 250, y: 860, hit: false },
      { id: 'bottomRight', name: 'BOTTOM RIGHT', x: 1670, y: 860, hit: false }
    ];
  }

  reset(count = 5) {
    this.targets = [];
    this.populateTargets(count);
  }

  populateTargets(count) {
    const columns = 3;
    const rows = 3;
    const marginX = 420;
    const marginY = 260;
    const widthStep = (1920 - marginX * 2) / (columns - 1);
    const heightStep = (1080 - marginY * 2) / (rows - 1);

    const possiblePositions = [];
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < columns; c++) {
        possiblePositions.push({
          x: marginX + c * widthStep,
          y: marginY + r * heightStep
        });
      }
    }

    possiblePositions.sort(() => Math.random() - 0.5);

    for (let i = 0; i < Math.min(count, possiblePositions.length); i++) {
      this.targets.push(this.createTarget(possiblePositions[i].x, possiblePositions[i].y));
    }
  }

  // Returns punch-specific color scheme with filled inner backgrounds
  getPunchColorScheme(requiredPunch, defaultThemeColor) {
    switch (requiredPunch) {
      case 'JAB':
      case 'UPPER':
      case 'UPPERCUT':
        return { main: '#00f0ff', secondary: '#0077ff', innerBg: '#00a8ff' };
      case 'HOOK':
      case 'CROSS':
        return { main: '#ff2200', secondary: '#ff8800', innerBg: '#e60000' };
      case 'ANY':
        return { main: '#d000ff', secondary: '#8800ff', innerBg: '#a800e6' };
      default:
        return { 
          main: defaultThemeColor || '#00f0ff', 
          secondary: '#0077ff', 
          innerBg: '#00a8ff' 
        };
    }
  }

  createTarget(x, y) {
    const requiredPunch = AppState.gameMode === 'DRILL' 
      ? AppState.currentDrillPunch 
      : (Math.random() < 0.6 ? 'ANY' : PUNCH_TYPES[Math.floor(Math.random() * PUNCH_TYPES.length)]);
    
    // Static ray burst offsets anchored to the target
    const burstRayCount = 12 + Math.floor(Math.random() * 6);
    const burstRays = [];
    for (let i = 0; i < burstRayCount; i++) {
      burstRays.push({
        angle: (Math.PI * 2 * i) / burstRayCount + (Math.random() * 0.2 - 0.1),
        length: 20 + Math.random() * 35,
        width: 2 + Math.random() * 4
      });
    }

    return {
      id: Math.random(),
      x,
      y,
      radius: 75,
      active: true,
      requiredPunch,
      pulseAngle: Math.random() * Math.PI * 2,
      burstRays,
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
      const pulseScale = 1 + Math.sin(target.pulseAngle) * 0.04;
      const baseRadius = target.radius * pulseScale;

      const baseThemeColor = AppState.targetColor || theme.targetColor;
      const colors = this.getPunchColorScheme(target.requiredPunch, baseThemeColor);

      ctx.save();
      ctx.translate(target.x, target.y);

      // --- 1. SPARK / BURST RAYS (OUTER) ---
      ctx.save();
      ctx.strokeStyle = colors.main;
      ctx.fillStyle = colors.main;
      ctx.shadowColor = colors.main;
      ctx.shadowBlur = 15;

      for (const ray of target.burstRays) {
        const currentLength = ray.length * (0.8 + Math.sin(target.pulseAngle * 2 + ray.angle) * 0.2);
        const innerX = Math.cos(ray.angle) * (baseRadius + 8);
        const innerY = Math.sin(ray.angle) * (baseRadius + 8);
        const outerX = Math.cos(ray.angle) * (baseRadius + 8 + currentLength);
        const outerY = Math.sin(ray.angle) * (baseRadius + 8 + currentLength);

        ctx.lineWidth = ray.width;
        ctx.beginPath();
        ctx.moveTo(innerX, innerY);
        ctx.lineTo(outerX, outerY);
        ctx.stroke();

        // Tip pixel square
        ctx.fillRect(outerX - 2, outerY - 2, 4, 4);
      }
      ctx.restore();

      // --- 2. OUTER DUAL NEON RINGS ---
      ctx.save();
      ctx.shadowColor = colors.main;
      ctx.shadowBlur = 18;

      // Primary Outer Thick Ring
      ctx.beginPath();
      ctx.arc(0, 0, baseRadius, 0, Math.PI * 2);
      ctx.strokeStyle = colors.main;
      ctx.lineWidth = 6;
      ctx.stroke();

      // Secondary Outer Accent Ring
      ctx.beginPath();
      ctx.arc(0, 0, baseRadius - 6, 0, Math.PI * 2);
      ctx.strokeStyle = colors.secondary;
      ctx.lineWidth = 3;
      ctx.stroke();
      ctx.restore();

      // --- 3. SEGMENTED HUD NOTCHES ---
      ctx.save();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 3;
      const segments = 4;
      for (let i = 0; i < segments; i++) {
        const startAngle = (Math.PI / 2) * i + Math.PI / 8;
        const endAngle = startAngle + Math.PI / 4;
        ctx.beginPath();
        ctx.arc(0, 0, baseRadius - 12, startAngle, endAngle);
        ctx.stroke();
      }
      ctx.restore();

      // --- 4. FILLED COLOR INNER CORE ---
      ctx.save();
      ctx.fillStyle = colors.innerBg;
      ctx.shadowColor = colors.main;
      ctx.shadowBlur = 12;
      ctx.beginPath();
      ctx.arc(0, 0, baseRadius - 18, 0, Math.PI * 2);
      ctx.fill();

      // White outline ring separating core and notches
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 3;
      ctx.stroke();
      ctx.restore();

      // --- 5. TARGET PUNCH TEXT ---
      ctx.save();
      ctx.fillStyle = '#ffffff';
      ctx.shadowColor = '#000000';
      ctx.shadowBlur = 6;
      ctx.font = '900 24px "Press Start 2P", Orbitron, monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(target.requiredPunch, 0, 2);
      ctx.restore();

      ctx.restore();
    }
  }

  drawReachTargets(ctx) {
    for (const corner of this.reachCornerTargets) {
      ctx.save();
      ctx.translate(corner.x, corner.y);

      const isHit = AppState.reachCorners[corner.id];
      ctx.shadowColor = isHit ? '#10b981' : '#f59e0b';
      ctx.shadowBlur = 20;

      ctx.beginPath();
      ctx.arc(0, 0, 85, 0, Math.PI * 2);
      ctx.fillStyle = isHit ? 'rgba(16, 185, 129, 0.4)' : 'rgba(245, 158, 11, 0.3)';
      ctx.fill();

      ctx.strokeStyle = isHit ? '#10b981' : '#f59e0b';
      ctx.lineWidth = 6;
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.font = '900 20px Orbitron';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(isHit ? '✓ COMPLETED' : corner.name, 0, 0);

      ctx.restore();
    }
  }

  checkReachTouch(gloveX, gloveY, gloveRadius) {
    let hitsCount = 0;
    for (const corner of this.reachCornerTargets) {
      if (AppState.reachCorners[corner.id]) {
        hitsCount++;
        continue;
      }

      const dx = gloveX - corner.x;
      const dy = gloveY - corner.y;
      const dist = Math.sqrt(dx * dx + dy * dy);

      if (dist <= 85 + gloveRadius) {
        AppState.reachCorners[corner.id] = true;
        particleEngine.spawnExplosion(corner.x, corner.y, 40, 'GOLDEN_ARENA', 80);
        soundEngine.playTargetHit(80);
        hitsCount++;
      }
    }
    return hitsCount;
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
          
          const comboBonus = AppState.combo * 25;
          const powerBonus = Math.floor(power * 2.5);
          AppState.score += 100 + comboBonus + powerBonus;
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
    
    // Regenerate spike rays on respawn
    const burstRayCount = 12 + Math.floor(Math.random() * 6);
    target.burstRays = [];
    for (let i = 0; i < burstRayCount; i++) {
      target.burstRays.push({
        angle: (Math.PI * 2 * i) / burstRayCount + (Math.random() * 0.2 - 0.1),
        length: 20 + Math.random() * 35,
        width: 2 + Math.random() * 4
      });
    }

    soundEngine.playTargetSpawn();
  }
}

export const targetManager = new TargetManager();
