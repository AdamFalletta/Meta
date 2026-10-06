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

      // Text Label
      ctx.fillStyle = '#ffffff';
      ctx.font = '900 22px Orbitron';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(target.requiredPunch, 0, 0);

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
    soundEngine.playTargetSpawn();
  }
}

export const targetManager = new TargetManager();