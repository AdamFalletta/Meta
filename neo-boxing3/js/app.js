import { THEMES, DEFAULT_LEADERBOARD, DEFAULT_LEADERBOARDS } from './config.js';
import { AppState, hexToRgba } from './state.js';
import { soundEngine } from './sound.js';
import { announcer } from './announcer.js';
import { particleEngine } from './particles.js';
import { targetManager } from './targets.js';
import { poseTracker } from './pose.js';

let initialsState = ['A', 'A', 'A'];
let currentInitialSlot = 0;
let gameTimerInterval = null;

const canvas = document.getElementById('game-canvas');
const ctx = canvas.getContext('2d');

// Rich arena backdrop. The generated image is tinted per theme at draw time.
const arenaBackground = new Image();
arenaBackground.src = 'assets/arena-bg.jpg';

const crowdFlashes = [];
let lastCrowdFlashAt = 0;

function drawSpotlights(ctx, theme, now) {
  const H = 1080;
  const lightColor = theme.targetColor || theme.accent;
  const positions = [180, 560, 960, 1360, 1740];

  ctx.save();
  ctx.globalCompositeOperation = 'screen';

  positions.forEach((x, index) => {
    const sway = Math.sin(now * 0.35 + index * 1.4) * 42;
    const baseX = x + (x - 960) * 0.08;
    const floorX = baseX + sway;
    const pulse = 0.55 + 0.45 * Math.sin(now * 1.15 + index * 0.9);
    const alpha = 0.12 + pulse * 0.1;

    // Layered cones soften the beam edges without a costly canvas blur.
    const layers = [
      { width: 2.15, alpha: 0.2 },
      { width: 1.6, alpha: 0.32 },
      { width: 1.1, alpha: 0.48 },
      { width: 0.7, alpha: 0.65 },
      { width: 0.34, alpha: 0.85 }
    ];

    layers.forEach(layer => {
      const topHalf = 12 * layer.width;
      const bottomHalf = 198 * layer.width;
      const beam = ctx.createLinearGradient(0, 0, 0, 940);
      beam.addColorStop(0, hexToRgba(lightColor, alpha * layer.alpha));
      beam.addColorStop(0.55, hexToRgba(lightColor, alpha * layer.alpha * 0.55));
      beam.addColorStop(1, hexToRgba(lightColor, 0));
      ctx.beginPath();
      ctx.moveTo(baseX - topHalf, 0);
      ctx.lineTo(baseX + topHalf, 0);
      ctx.lineTo(floorX + bottomHalf, 940);
      ctx.lineTo(floorX - bottomHalf, 940);
      ctx.closePath();
      ctx.fillStyle = beam;
      ctx.fill();
    });

    // Lamp head bloom.
    const bloom = ctx.createRadialGradient(baseX, 28, 2, baseX, 28, 76);
    bloom.addColorStop(0, 'rgba(255, 255, 255, 0.92)');
    bloom.addColorStop(0.25, hexToRgba(lightColor, 0.6));
    bloom.addColorStop(1, 'rgba(0, 0, 0, 0)');
    ctx.fillStyle = bloom;
    ctx.beginPath();
    ctx.arc(baseX, 28, 76, 0, Math.PI * 2);
    ctx.fill();

    // Light pool on the canvas.
    const pool = ctx.createRadialGradient(floorX, 930, 10, floorX, 930, 250);
    pool.addColorStop(0, hexToRgba(lightColor, 0.2 + pulse * 0.14));
    pool.addColorStop(1, 'rgba(0, 0, 0, 0)');
    ctx.fillStyle = pool;
    ctx.beginPath();
    ctx.ellipse(floorX, 930, 250, 95, 0, 0, Math.PI * 2);
    ctx.fill();
  });

  ctx.restore();
}

function drawCrowdFlashes(ctx, now) {
  // Spawn short-lived camera flashes across the crowd rows.
  if (now - lastCrowdFlashAt > 0.08) {
    lastCrowdFlashAt = now;
    const spawnCount = Math.random() < 0.65 ? 1 : 2;
    for (let i = 0; i < spawnCount; i++) {
      const big = Math.random() < 0.14;
      crowdFlashes.push({
        x: 40 + Math.random() * 1840,
        y: 290 + Math.random() * 320,
        born: now,
        life: 0.22 + Math.random() * 0.4,
        size: big ? 5 + Math.random() * 3 : 2 + Math.random() * 3,
        big
      });
    }
  }

  ctx.save();
  ctx.globalCompositeOperation = 'screen';

  for (let i = crowdFlashes.length - 1; i >= 0; i--) {
    const flash = crowdFlashes[i];
    const age = now - flash.born;
    if (age >= flash.life) {
      crowdFlashes.splice(i, 1);
      continue;
    }

    const t = 1 - age / flash.life;
    const coreRadius = flash.size * (0.6 + t * 0.8);
    const halo = ctx.createRadialGradient(flash.x, flash.y, 0, flash.x, flash.y, coreRadius * 12);
    halo.addColorStop(0, `rgba(255, 255, 255, ${0.95 * t})`);
    halo.addColorStop(0.16, `rgba(220, 245, 255, ${0.55 * t})`);
    halo.addColorStop(1, 'rgba(0, 0, 0, 0)');

    ctx.fillStyle = halo;
    ctx.beginPath();
    ctx.arc(flash.x, flash.y, coreRadius * 12, 0, Math.PI * 2);
    ctx.fill();

    if (flash.big) {
      ctx.fillStyle = `rgba(255, 255, 255, ${0.4 * t})`;
      ctx.fillRect(flash.x - coreRadius * 5, flash.y - 1, coreRadius * 10, 2);
      ctx.fillRect(flash.x - 1, flash.y - coreRadius * 5, 2, coreRadius * 10);
    }

    ctx.fillStyle = `rgba(255, 255, 255, ${t})`;
    ctx.beginPath();
    ctx.arc(flash.x, flash.y, coreRadius, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.restore();
}

function drawArenaBackdrop(ctx, theme) {
  const W = 1920;
  const H = 1080;
  const now = Date.now() * 0.001;
  const glowColor = theme.targetColor || theme.accent;

  if (arenaBackground.complete && arenaBackground.naturalWidth > 0) {
    const scale = Math.max(W / arenaBackground.naturalWidth, H / arenaBackground.naturalHeight);
    const drawW = arenaBackground.naturalWidth * scale;
    const drawH = arenaBackground.naturalHeight * scale;
    ctx.drawImage(arenaBackground, (W - drawW) / 2, (H - drawH) / 2, drawW, drawH);
  } else {
    const fallback = ctx.createLinearGradient(0, 0, 0, H);
    fallback.addColorStop(0, theme.bgGrad[0]);
    fallback.addColorStop(1, theme.bgGrad[1]);
    ctx.fillStyle = fallback;
    ctx.fillRect(0, 0, W, H);
  }

  // Push the theme color hard into the photo midtones.
  ctx.save();
  ctx.globalCompositeOperation = 'overlay';
  ctx.globalAlpha = 0.36;
  ctx.fillStyle = glowColor;
  ctx.fillRect(0, 0, W, H);
  ctx.restore();

  // Extra color wash so the arena palette reads instantly.
  const tint = ctx.createLinearGradient(0, 0, 0, H);
  tint.addColorStop(0, hexToRgba(glowColor, 0.16));
  tint.addColorStop(0.45, hexToRgba(glowColor, 0.05));
  tint.addColorStop(1, hexToRgba(glowColor, 0.26));
  ctx.fillStyle = tint;
  ctx.fillRect(0, 0, W, H);

  // Vignette keeps the HUD and targets readable over the busy crowd.
  const vignette = ctx.createRadialGradient(W / 2, H * 0.45, H * 0.16, W / 2, H * 0.5, H * 0.85);
  vignette.addColorStop(0, 'rgba(2, 6, 23, 0)');
  vignette.addColorStop(0.68, 'rgba(2, 6, 23, 0.18)');
  vignette.addColorStop(1, 'rgba(2, 6, 23, 0.74)');
  ctx.fillStyle = vignette;
  ctx.fillRect(0, 0, W, H);

  // Ring-floor color bounce.
  const floorGlow = ctx.createRadialGradient(960, 990, 60, 960, 990, 780);
  floorGlow.addColorStop(0, hexToRgba(glowColor, 0.2));
  floorGlow.addColorStop(1, 'rgba(0, 0, 0, 0)');
  ctx.fillStyle = floorGlow;
  ctx.fillRect(0, 0, W, H);

  drawSpotlights(ctx, theme, now);
  drawCrowdFlashes(ctx, now);
}

/* =========================================================================
   CANVAS RENDER & HUD
   ========================================================================= */
function renderGameLoop() {
  ctx.clearRect(0, 0, 1920, 1080);

  const activeTheme = THEMES[AppState.activeThemeKey];
  drawArenaBackdrop(ctx, activeTheme);

  if (AppState.currentScreen === 'ATTRACT') {
    renderAttractVisuals(ctx, activeTheme);
  } else if (AppState.currentScreen === 'PLAYING') {
    targetManager.drawTargets(ctx, activeTheme);
    particleEngine.updateAndDraw(ctx);

    poseTracker.drawSkeleton(ctx, 0.35);
    poseTracker.drawGloveIndicators(ctx, 0.85);

    renderArcadeHUD(ctx, activeTheme);
  }

  requestAnimationFrame(renderGameLoop);
}

function renderAttractVisuals(ctx, theme) {
  const now = Date.now() * 0.002;
  ctx.save();
  ctx.strokeStyle = theme.accentGlow;
  ctx.lineWidth = 2;

  for (let i = 0; i < 3; i++) {
    ctx.beginPath();
    for (let x = 0; x < 1920; x += 20) {
      const y = 540 + Math.sin(now + x * 0.003 + i) * 80 * (i + 1);
      if (x === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
  ctx.restore();
}

function drawHudPanel(ctx, x, y, w, h, color, fill = 'rgba(2, 6, 23, 0.74)', cut = 20, glow = 18) {
  ctx.save();
  ctx.beginPath();
  ctx.moveTo(x + cut, y);
  ctx.lineTo(x + w - cut, y);
  ctx.lineTo(x + w, y + cut);
  ctx.lineTo(x + w, y + h - cut);
  ctx.lineTo(x + w - cut, y + h);
  ctx.lineTo(x + cut, y + h);
  ctx.lineTo(x, y + h - cut);
  ctx.lineTo(x, y + cut);
  ctx.closePath();

  ctx.fillStyle = fill;
  ctx.fill();

  ctx.shadowColor = color;
  ctx.shadowBlur = glow;
  ctx.strokeStyle = color;
  ctx.lineWidth = 3;
  ctx.stroke();

  ctx.shadowBlur = 0;
  ctx.strokeStyle = hexToRgba(color, 0.28);
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.restore();
}

function drawHudLogo(ctx, centerX, y, accent) {
  const neo = 'NEO ';
  const boxing = 'BOXING';

  ctx.save();
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  ctx.font = '900 46px Orbitron';

  const neoWidth = ctx.measureText(neo).width;
  const boxingWidth = ctx.measureText(boxing).width;
  const startX = centerX - (neoWidth + boxingWidth) / 2;

  ctx.fillStyle = accent;
  ctx.shadowColor = accent;
  ctx.shadowBlur = 18;
  ctx.fillText(neo, startX, y);

  ctx.fillStyle = '#fb7185';
  ctx.shadowColor = '#fb7185';
  ctx.shadowBlur = 18;
  ctx.fillText(boxing, startX + neoWidth, y);

  ctx.shadowBlur = 0;
  const bar = ctx.createLinearGradient(startX, 0, startX + neoWidth + boxingWidth, 0);
  bar.addColorStop(0, hexToRgba(accent, 0));
  bar.addColorStop(0.5, hexToRgba(accent, 0.85));
  bar.addColorStop(1, 'rgba(251, 113, 133, 0)');
  ctx.fillStyle = bar;
  ctx.fillRect(startX, y + 30, neoWidth + boxingWidth, 3);
  ctx.restore();
}

function drawArcGauge(ctx, cx, cy, radius, value, label, color, px, py, pw, ph) {
  drawHudPanel(ctx, px, py, pw, ph, color, 'rgba(2, 6, 23, 0.68)', 18, 20);

  const startAngle = Math.PI * 0.82;
  const endAngle = Math.PI * 2.18;
  const pct = Math.max(0, Math.min(1, value / 100));

  ctx.save();
  ctx.lineCap = 'round';

  // Track.
  ctx.strokeStyle = hexToRgba(color, 0.17);
  ctx.lineWidth = 18;
  ctx.beginPath();
  ctx.arc(cx, cy, radius, startAngle, endAngle);
  ctx.stroke();

  // Tick marks.
  const ticks = 14;
  for (let i = 0; i <= ticks; i++) {
    const t = i / ticks;
    const angle = startAngle + (endAngle - startAngle) * t;
    const inner = radius - 17;
    const outer = radius + 16;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(angle) * inner, cy + Math.sin(angle) * inner);
    ctx.lineTo(cx + Math.cos(angle) * outer, cy + Math.sin(angle) * outer);
    ctx.strokeStyle = t <= pct ? hexToRgba(color, 0.9) : 'rgba(148, 163, 184, 0.22)';
    ctx.lineWidth = 3;
    ctx.stroke();
  }

  // Filled arc.
  if (pct > 0) {
    ctx.strokeStyle = color;
    ctx.shadowColor = color;
    ctx.shadowBlur = 20;
    ctx.lineWidth = 16;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, startAngle, startAngle + (endAngle - startAngle) * pct);
    ctx.stroke();
  }

  ctx.shadowBlur = 0;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = 'rgba(226, 232, 240, 0.92)';
  ctx.font = '700 18px Orbitron';
  ctx.fillText(label, cx, cy - 8);

  ctx.fillStyle = '#ffffff';
  ctx.shadowColor = color;
  ctx.shadowBlur = 16;
  ctx.font = '900 40px Orbitron';
  ctx.fillText(`${Math.round(value)}%`, cx, cy + 38);
  ctx.restore();
}

function formatTimer(seconds) {
  const clamped = Math.max(0, seconds);
  const minutes = Math.floor(clamped / 60);
  const secs = clamped % 60;
  return `${String(minutes).padStart(2, '0')}:${String(secs).padStart(2, '0')}`;
}

function renderArcadeHUD(ctx, theme) {
  const accent = theme.targetColor || theme.accent;
  const accuracy = AppState.totalPunches > 0
    ? Math.floor((AppState.successfulHits / AppState.totalPunches) * 100)
    : 100;

  ctx.save();

  // SCORE
  drawHudPanel(ctx, 24, 22, 440, 128, accent, 'rgba(2, 6, 23, 0.74)');
  ctx.textAlign = 'left';
  ctx.textBaseline = 'alphabetic';
  ctx.fillStyle = 'rgba(226, 232, 240, 0.9)';
  ctx.shadowColor = accent;
  ctx.shadowBlur = 8;
  ctx.font = '700 20px Orbitron';
  ctx.fillText('SCORE', 54, 64);

  ctx.fillStyle = '#ffffff';
  ctx.shadowColor = accent;
  ctx.shadowBlur = 18;
  ctx.font = '900 54px Orbitron';
  ctx.fillText(AppState.score.toLocaleString(), 54, 126);

  // COMBO
  drawHudPanel(ctx, 1188, 22, 312, 128, '#fbbf24', 'rgba(2, 6, 23, 0.74)');
  ctx.shadowBlur = 8;
  ctx.fillStyle = '#fbbf24';
  ctx.font = '700 20px Orbitron';
  ctx.fillText('COMBO', 1214, 64);

  ctx.textAlign = 'right';
  ctx.fillStyle = AppState.combo > 0 ? '#fde047' : '#64748b';
  ctx.shadowColor = '#fbbf24';
  ctx.shadowBlur = AppState.combo > 0 ? 18 : 0;
  ctx.font = '900 54px Orbitron';
  ctx.fillText(`x${AppState.combo}`, 1476, 126);

  ctx.shadowBlur = 0;
  ctx.fillStyle = AppState.combo > 0 ? hexToRgba('#fbbf24', 0.8) : 'rgba(148, 163, 184, 0.25)';
  ctx.fillRect(1214, 143, 262, 3);

  // TIMER
  const lowTime = AppState.timeRemaining <= 10;
  drawHudPanel(ctx, 1524, 22, 372, 128, lowTime ? '#ef4444' : accent, 'rgba(2, 6, 23, 0.74)');
  ctx.textAlign = 'center';
  ctx.fillStyle = lowTime ? '#fca5a5' : 'rgba(226, 232, 240, 0.9)';
  ctx.shadowColor = lowTime ? '#ef4444' : accent;
  ctx.shadowBlur = 8;
  ctx.font = '700 20px Orbitron';
  ctx.fillText('TIMER', 1710, 64);

  ctx.fillStyle = lowTime ? '#f87171' : '#ffffff';
  ctx.shadowColor = lowTime ? '#ef4444' : accent;
  ctx.shadowBlur = lowTime ? 22 : 16;
  ctx.font = '900 54px Orbitron';
  ctx.fillText(formatTimer(AppState.timeRemaining), 1710, 126);

  // Center logo
  drawHudLogo(ctx, 826, 96, accent);

  // Lower meters
  drawArcGauge(ctx, 178, 930, 80, AppState.recentPower, 'POWER', '#ef4444', 28, 820, 300, 234);
  drawArcGauge(ctx, 515, 930, 80, accuracy, 'ACCURACY', '#10b981', 350, 820, 330, 234);

  ctx.restore();
}

/* =========================================================================
   STORAGE & LEADERBOARD SYSTEM
   ========================================================================= */
function updateInitialsDisplay() {
  const slots = document.querySelectorAll('.initial-slot');
  slots.forEach((slot, index) => {
    slot.textContent = initialsState[index];
    if (index === currentInitialSlot) {
      slot.className = "initial-slot w-16 h-20 bg-slate-950 border-2 border-amber-400 rounded-xl flex items-center justify-center font-arcade text-4xl font-black text-amber-300 scale-105 shadow-[0_0_15px_rgba(245,158,11,0.5)] transition cursor-pointer";
    } else {
      slot.className = "initial-slot w-16 h-20 bg-slate-950 border-2 border-slate-800 rounded-xl flex items-center justify-center font-arcade text-4xl font-black text-slate-500 transition cursor-pointer hover:border-slate-700";
    }
  });
}

function cycleInitialChar(direction) {
  const alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
  let charIdx = alphabet.indexOf(initialsState[currentInitialSlot]);
  if (charIdx === -1) charIdx = 0;

  if (direction === 'UP') {
    charIdx = (charIdx + 1) % alphabet.length;
  } else {
    charIdx = (charIdx - 1 + alphabet.length) % alphabet.length;
  }
  initialsState[currentInitialSlot] = alphabet[charIdx];
  updateInitialsDisplay();
}

const LEADERBOARD_STORAGE_KEY = 'neoBoxingLeaderboardsV2';

function getAllLeaderboards() {
  try {
    const stored = localStorage.getItem(LEADERBOARD_STORAGE_KEY);
    if (stored) {
      const parsed = JSON.parse(stored);
      if (parsed && typeof parsed === 'object' && !Array.isArray(parsed)) return parsed;
    }

    // Migrate the old single-board save into the 5-target board.
    const legacy = localStorage.getItem('neoBoxingLeaderboard');
    if (legacy) {
      const migrated = JSON.parse(legacy);
      if (Array.isArray(migrated)) return { 5: migrated };
    }
  } catch (err) {
    console.warn('Could not read leaderboards', err);
  }
  return {};
}

function getLeaderboard(targetCount = AppState.targetCountSetting) {
  const boards = getAllLeaderboards();
  const saved = boards[targetCount];
  if (Array.isArray(saved)) return saved.map(entry => ({ ...entry }));

  const defaults = DEFAULT_LEADERBOARDS[targetCount] || DEFAULT_LEADERBOARD;
  return defaults.map(entry => ({ ...entry }));
}

function saveLeaderboard(data, targetCount = AppState.targetCountSetting) {
  const boards = getAllLeaderboards();
  boards[targetCount] = data;
  localStorage.setItem(LEADERBOARD_STORAGE_KEY, JSON.stringify(boards));
}

function renderLeaderboardUI() {
  const targetCount = AppState.leaderboardTargetCount || AppState.targetCountSetting;
  const leaderboard = getLeaderboard(targetCount);
  const listEl = document.getElementById('leaderboard-list');
  listEl.innerHTML = '';

  const label = document.getElementById('leaderboard-target-label');
  if (label) label.textContent = `${targetCount} TARGETS · TOP 5 CHAMPIONS`;

  document.querySelectorAll('.lb-target-btn').forEach(btn => {
    const isActive = parseInt(btn.getAttribute('data-value'), 10) === targetCount;
    btn.classList.toggle('option-active', isActive);
    btn.classList.toggle('text-slate-400', !isActive);
  });

  if (leaderboard.length === 0) {
    listEl.innerHTML = '<p class="text-center text-slate-500 font-arcade text-sm py-10">NO SCORES YET — BE THE FIRST CHAMPION</p>';
    return;
  }

  leaderboard.forEach((item, index) => {
    const row = document.createElement('div');
    row.className = 'flex items-center justify-between p-4 bg-slate-950/60 rounded-xl border border-slate-800 font-arcade';
    row.innerHTML = `
      <div class="flex items-center gap-4">
        <span class="text-xl font-black ${index === 0 ? 'text-amber-400' : 'text-slate-500'}">#${index + 1}</span>
        <span class="text-2xl font-black text-slate-200 tracking-widest">${item.initials}</span>
      </div>
      <span class="text-2xl font-black text-cyan-400">${item.score.toLocaleString()} PTS</span>
      <span class="text-xs text-slate-500 font-mono">${item.date}</span>
    `;
    listEl.appendChild(row);
  });
}

function updateAttractChampion() {
  const el = document.getElementById('attract-top-score');
  if (!el) return;
  const top = getLeaderboard(AppState.targetCountSetting)[0];
  el.textContent = top ? `${top.initials} — ${top.score.toLocaleString()} PTS` : 'NO CHAMPION YET';
}

/* =========================================================================
   GAME CONTROLLER
   ========================================================================= */
function switchScreen(targetScreen) {
  AppState.currentScreen = targetScreen;

  const overlays = ['attract-overlay', 'menu-overlay', 'countdown-overlay', 'gameover-overlay', 'highscore-overlay', 'leaderboard-overlay'];
  overlays.forEach(id => document.getElementById(id).classList.add('hidden'));

  if (targetScreen === 'ATTRACT') {
    updateAttractChampion();
    document.getElementById('attract-overlay').classList.remove('hidden');
  } else if (targetScreen === 'MENU') {
    document.getElementById('menu-overlay').classList.remove('hidden');
  } else if (targetScreen === 'COUNTDOWN') {
    document.getElementById('countdown-overlay').classList.remove('hidden');
  } else if (targetScreen === 'GAMEOVER') {
    document.getElementById('gameover-overlay').classList.remove('hidden');
    renderGameOverStats();
  } else if (targetScreen === 'LEADERBOARD') {
    AppState.leaderboardTargetCount = AppState.targetCountSetting;
    renderLeaderboardUI();
    document.getElementById('leaderboard-overlay').classList.remove('hidden');
  }
}

function startCountdown() {
  soundEngine.ensureContext();
  switchScreen('COUNTDOWN');
  let count = 3;
  const numEl = document.getElementById('countdown-number');
  numEl.textContent = count;
  soundEngine.playCountdownBeep(false);

  const countInterval = setInterval(() => {
    count--;
    if (count > 0) {
      numEl.textContent = count;
      soundEngine.playCountdownBeep(false);
      announcer.speak(count.toString());
    } else {
      clearInterval(countInterval);
      numEl.textContent = "GO!";
      soundEngine.playDingDingDing();
      soundEngine.playCrowdCheer(95);
      announcer.speak("Fight!");
      setTimeout(launchGameplay, 800);
    }
  }, 1000);
}

function launchGameplay() {
  switchScreen('PLAYING');
  AppState.score = 0;
  AppState.combo = 0;
  AppState.maxCombo = 0;
  AppState.totalPunches = 0;
  AppState.successfulHits = 0;
  AppState.recentPower = 0;
  AppState.maxPower = 0;
  AppState.timeRemaining = 60;
  targetManager.reset(AppState.targetCountSetting);

  soundEngine.ensureContext();
  soundEngine.startCrowdHum();

  if (gameTimerInterval) clearInterval(gameTimerInterval);
  gameTimerInterval = setInterval(() => {
    AppState.timeRemaining--;
    if (AppState.timeRemaining === 10) {
      announcer.speak("Ten seconds remaining");
    }
    if (AppState.timeRemaining <= 0) {
      clearInterval(gameTimerInterval);
      finishGame();
    }
  }, 1000);
}

function openHighScoreEntry() {
  initialsState = ['A', 'A', 'A'];
  currentInitialSlot = 0;
  updateInitialsDisplay();
  document.getElementById('final-score-display').textContent =
    `FINAL SCORE (${AppState.targetCountSetting} TARGETS): ${AppState.score.toLocaleString()}`;
  document.getElementById('highscore-overlay').classList.remove('hidden');
}

function finishGame() {
  soundEngine.stopCrowdHum();
  soundEngine.playDingDingDing();
  soundEngine.playCrowdCheer(85);
  announcer.speak("Game Over");

  const targetCount = AppState.targetCountSetting;
  const leaderboard = getLeaderboard(targetCount);
  const rank = leaderboard.filter(entry => entry.score > AppState.score).length + 1;
  const qualifies = rank <= 5;

  switchScreen('GAMEOVER');

  const praise = qualifies ? 'WAY TO GO CHAMP!' : 'GOOD JOB!';
  document.getElementById('gameover-praise').textContent = praise;
  document.getElementById('gameover-subtitle').textContent = qualifies
    ? `You punched your way into the ${targetCount}-target top 5.`
    : `${targetCount}-target session complete — keep training for the top 5.`;

  const status = document.getElementById('gameover-leaderboard-status');
  if (qualifies) {
    status.className = 'px-5 py-3 rounded-xl border-2 border-amber-400/70 bg-amber-500/15 font-arcade text-sm font-bold tracking-wide text-center text-amber-300 shadow-[0_0_22px_rgba(245,158,11,0.25)]';
    status.innerHTML = `NEW HIGH SCORE &nbsp;•&nbsp; RANK #${rank} &nbsp;•&nbsp; ${targetCount} TARGETS`;
  } else {
    status.className = 'px-5 py-3 rounded-xl border-2 border-slate-600/60 bg-slate-900/60 font-arcade text-sm font-bold tracking-wide text-center text-slate-400';
    status.innerHTML = `RANK #${rank} &nbsp;•&nbsp; TOP 5 REQUIRED &nbsp;•&nbsp; ${targetCount} TARGETS`;
  }

  document.getElementById('btn-claim-score').classList.toggle('hidden', !qualifies);

  setTimeout(() => announcer.speak(qualifies ? "Way to go champ!" : "Good job!"), 1400);
}

function renderGameOverStats() {
  document.getElementById('stat-final-score').textContent = AppState.score.toLocaleString();
  const accuracy = AppState.totalPunches > 0 ? Math.floor((AppState.successfulHits / AppState.totalPunches) * 100) : 100;
  document.getElementById('stat-final-accuracy').textContent = `${accuracy}%`;
  document.getElementById('stat-final-combo').textContent = `x${AppState.maxCombo}`;
  document.getElementById('stat-final-power').textContent = AppState.maxPower;
  document.getElementById('stat-final-targets').textContent = AppState.targetCountSetting;
}

function setupSelectionGroup(containerId, btnClass, callback) {
  const container = document.getElementById(containerId);
  if (!container) return;
  const buttons = container.getElementsByClassName(btnClass);
  Array.from(buttons).forEach(btn => {
    btn.addEventListener('click', () => {
      Array.from(buttons).forEach(b => b.classList.remove('option-active', 'option-active-red'));
      const isActiveRed = btn.getAttribute('data-value') === 'REVV';
      btn.classList.add(isActiveRed ? 'option-active-red' : 'option-active');
      callback(btn.getAttribute('data-value'));
    });
  });
}

/* =========================================================================
   INITIALIZATION & EVENT LISTENERS
   ========================================================================= */
window.addEventListener('load', () => {
  poseTracker.init();
  renderGameLoop();
  updateAttractChampion();

  // Attract Start Button
  document.getElementById('btn-start-game').addEventListener('click', () => {
    soundEngine.ensureContext();
    switchScreen('MENU');
  });

  // Enter the arena directly — no calibration screens.
  document.getElementById('btn-start-match').addEventListener('click', () => {
    startCountdown();
  });

  // Fullscreen Toggle Button
  document.getElementById('btn-fullscreen').addEventListener('click', () => {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen().catch(err => console.log(err));
    } else {
      document.exitFullscreen();
    }
  });

  // Target Color Preset Buttons
  const targetColorBtns = document.querySelectorAll('.target-color-btn');
  targetColorBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      targetColorBtns.forEach(b => {
        b.classList.remove('ring-2', 'ring-cyan-400', 'scale-105', 'border-2', 'border-white', 'z-10');
        b.classList.add('border', 'border-slate-700');
      });
      btn.classList.remove('border', 'border-slate-700');
      btn.classList.add('ring-2', 'ring-cyan-400', 'scale-105', 'border-2', 'border-white', 'z-10');
      AppState.targetColor = btn.getAttribute('data-color');
      const modeLabel = document.getElementById('target-color-mode');
      if (modeLabel) modeLabel.textContent = `PICKED — ${AppState.targetColor}`;
    });
  });

  // Target Image Overlay Upload
  const targetImageInput = document.getElementById('target-image-input');
  const targetImageStatus = document.getElementById('target-image-status');
  const targetImagePreview = document.getElementById('target-image-preview');
  const clearTargetImageBtn = document.getElementById('btn-clear-target-image');

  document.getElementById('btn-upload-target-image').addEventListener('click', () => targetImageInput.click());

  targetImageInput.addEventListener('change', (e) => {
    const file = e.target.files && e.target.files[0];
    if (!file) return;

    if (!file.type.startsWith('image/')) {
      targetImageStatus.textContent = 'Please choose an image file';
      return;
    }

    const reader = new FileReader();
    reader.onload = () => {
      const upload = new Image();
      upload.onload = () => {
        // Downscale once into a square canvas so per-frame drawing stays cheap.
        const size = 512;
        const normalized = document.createElement('canvas');
        normalized.width = size;
        normalized.height = size;
        const nctx = normalized.getContext('2d');
        const scale = Math.max(size / upload.naturalWidth, size / upload.naturalHeight);
        const drawW = upload.naturalWidth * scale;
        const drawH = upload.naturalHeight * scale;
        nctx.drawImage(upload, (size - drawW) / 2, (size - drawH) / 2, drawW, drawH);

        AppState.targetImage = normalized;
        AppState.targetImageName = file.name;

        targetImagePreview.src = reader.result;
        targetImagePreview.classList.remove('hidden');
        targetImageStatus.textContent = file.name;
        targetImageStatus.classList.add('text-cyan-400');
        clearTargetImageBtn.disabled = false;
      };
      upload.onerror = () => {
        targetImageStatus.textContent = 'Could not load that image';
      };
      upload.src = reader.result;
    };
    reader.onerror = () => {
      targetImageStatus.textContent = 'Could not read that file';
    };
    reader.readAsDataURL(file);
  });

  clearTargetImageBtn.addEventListener('click', () => {
    AppState.targetImage = null;
    AppState.targetImageName = '';
    targetImageInput.value = '';
    targetImagePreview.src = '';
    targetImagePreview.classList.add('hidden');
    targetImageStatus.textContent = 'No image overlay';
    targetImageStatus.classList.remove('text-cyan-400');
    clearTargetImageBtn.disabled = true;
  });

  // Left Glove Preset Buttons
  const leftGloveBtns = document.querySelectorAll('.left-glove-btn');
  leftGloveBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      leftGloveBtns.forEach(b => {
        b.classList.remove('ring-2', 'ring-red-500', 'scale-105', 'border-2', 'border-white', 'z-10');
        b.classList.add('border', 'border-slate-700');
      });
      btn.classList.remove('border', 'border-slate-700');
      btn.classList.add('ring-2', 'ring-red-500', 'scale-105', 'border-2', 'border-white', 'z-10');
      AppState.leftGloveColor = btn.getAttribute('data-color');
      const label = document.getElementById('label-left-glove');
      if (label) label.textContent = AppState.leftGloveColor;
    });
  });

  // Right Glove Preset Buttons
  const rightGloveBtns = document.querySelectorAll('.right-glove-btn');
  rightGloveBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      rightGloveBtns.forEach(b => {
        b.classList.remove('ring-2', 'ring-cyan-400', 'scale-105', 'border-2', 'border-white', 'z-10');
        b.classList.add('border', 'border-slate-700');
      });
      btn.classList.remove('border', 'border-slate-700');
      btn.classList.add('ring-2', 'ring-cyan-400', 'scale-105', 'border-2', 'border-white', 'z-10');
      AppState.rightGloveColor = btn.getAttribute('data-color');
      const label = document.getElementById('label-right-glove');
      if (label) label.textContent = AppState.rightGloveColor;
    });
  });

  // Selection Groups for Menu
  setupSelectionGroup('mode-selector', 'mode-btn', (val) => AppState.gameMode = val);
  setupSelectionGroup('target-count-selector', 'target-btn', (val) => AppState.targetCountSetting = parseInt(val));
  setupSelectionGroup('theme-selector', 'theme-btn', (val) => AppState.activeThemeKey = val);

  // Leaderboard Modal Buttons
  document.getElementById('btn-show-leaderboard').addEventListener('click', () => switchScreen('LEADERBOARD'));
  document.getElementById('btn-close-leaderboard').addEventListener('click', () => switchScreen('MENU'));

  // Leaderboard board selector (one board per target count)
  document.querySelectorAll('.lb-target-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      AppState.leaderboardTargetCount = parseInt(btn.getAttribute('data-value'), 10);
      renderLeaderboardUI();
    });
  });

  // Game Over Screen Buttons
  document.getElementById('btn-claim-score').addEventListener('click', openHighScoreEntry);
  document.getElementById('btn-restart-game').addEventListener('click', () => startCountdown());
  document.getElementById('btn-return-menu').addEventListener('click', () => switchScreen('MENU'));

  const slotBtns = document.querySelectorAll('.initial-slot');
  slotBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      currentInitialSlot = parseInt(btn.getAttribute('data-slot'));
      updateInitialsDisplay();
    });
  });

  document.getElementById('btn-initial-prev').addEventListener('click', () => cycleInitialChar('UP'));
  document.getElementById('btn-initial-next').addEventListener('click', () => cycleInitialChar('DOWN'));
  
  document.getElementById('btn-initial-prev-slot').addEventListener('click', () => {
    currentInitialSlot = (currentInitialSlot - 1 + 3) % 3;
    updateInitialsDisplay();
  });
  document.getElementById('btn-initial-next-slot').addEventListener('click', () => {
    currentInitialSlot = (currentInitialSlot + 1) % 3;
    updateInitialsDisplay();
  });

  // Keyboard support for initial entry
  window.addEventListener('keydown', (e) => {
    const overlay = document.getElementById('highscore-overlay');
    if (overlay && !overlay.classList.contains('hidden')) {
      if (e.key === 'ArrowUp') {
        cycleInitialChar('UP');
      } else if (e.key === 'ArrowDown') {
        cycleInitialChar('DOWN');
      } else if (e.key === 'ArrowLeft') {
        currentInitialSlot = (currentInitialSlot - 1 + 3) % 3;
        updateInitialsDisplay();
      } else if (e.key === 'ArrowRight') {
        currentInitialSlot = (currentInitialSlot + 1) % 3;
        updateInitialsDisplay();
      } else if (e.key === 'Enter') {
        document.getElementById('btn-initial-confirm').click();
      } else if (/^[a-zA-Z0-9]$/.test(e.key)) {
        initialsState[currentInitialSlot] = e.key.toUpperCase();
        currentInitialSlot = (currentInitialSlot + 1) % 3;
        updateInitialsDisplay();
      }
    }
  });

  // High Score Confirmation
  document.getElementById('btn-initial-confirm').addEventListener('click', () => {
    const initials = initialsState.join('');
    const targetCount = AppState.targetCountSetting;
    const leaderboard = getLeaderboard(targetCount);
    const now = new Date();
    const dateStr = now.toLocaleDateString('en-US', { month: 'short', day: '2-digit', year: 'numeric' }).toUpperCase();
    
    leaderboard.push({ rank: 0, initials, score: AppState.score, date: dateStr });
    leaderboard.sort((a, b) => b.score - a.score);
    saveLeaderboard(leaderboard.slice(0, 5), targetCount);

    document.getElementById('highscore-overlay').classList.add('hidden');
    switchScreen('LEADERBOARD');
  });

  // Canvas Touch / Mouse Fallback Input
  canvas.addEventListener('mousedown', (e) => {
    const rect = canvas.getBoundingClientRect();
    const scaleX = 1920 / rect.width;
    const scaleY = 1080 / rect.height;
    const clickX = (e.clientX - rect.left) * scaleX;
    const clickY = (e.clientY - rect.top) * scaleY;

    if (AppState.currentScreen === 'PLAYING') {
      AppState.totalPunches++;
      const hit = targetManager.checkPunchCollision(clickX, clickY, AppState.gloveRadius, 'ANY', 85);
      if (!hit && AppState.gameMode === 'COMBO') {
        soundEngine.playMissSound();
        AppState.combo = 0;
      }
    }
  });
});