import { THEMES } from './config.js';

class ParticleSystem {
  constructor() {
    this.particles = [];
  }

  spawnExplosion(x, y, count = 35, themeKey = 'NEO_BLUE', power = 50) {
    const colors = THEMES[themeKey].particleColors;
    const speedMult = 1 + (power / 40);

    for (let i = 0; i < count; i++) {
      const angle = Math.random() * Math.PI * 2;
      const speed = (Math.random() * 9 + 4) * speedMult;
      const color = colors[Math.floor(Math.random() * colors.length)];
      const size = Math.random() * 10 + 4;
      const life = Math.random() * 25 + 20;

      this.particles.push({
        x, y,
        vx: Math.cos(angle) * speed,
        vy: Math.sin(angle) * speed,
        size,
        maxLife: life,
        life,
        color,
        alpha: 1.0
      });
    }
  }

  updateAndDraw(ctx) {
    for (let i = this.particles.length - 1; i >= 0; i--) {
      const p = this.particles[i];
      p.x += p.vx;
      p.y += p.vy;
      p.vx *= 0.93;
      p.vy *= 0.93;
      p.life--;
      p.alpha = Math.max(0, p.life / p.maxLife);

      if (p.life <= 0) {
        this.particles.splice(i, 1);
        continue;
      }

      ctx.save();
      ctx.globalAlpha = p.alpha;
      ctx.fillStyle = p.color;
      ctx.shadowColor = p.color;
      ctx.shadowBlur = 12;
      ctx.beginPath();
      ctx.arc(p.x, p.y, p.size * (p.life / p.maxLife), 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }
  }
}

export const particleEngine = new ParticleSystem();