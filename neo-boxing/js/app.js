import { THEMES, DEFAULT_LEADERBOARD } from './config.js';
import { AppState } from './state.js';
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

/* =========================================================================
   CANVAS RENDER & HUD
   ========================================================================= */
function renderGameLoop() {
  ctx.clearRect(0, 0, 1920, 1080);

  const activeTheme = THEMES[AppState.activeThemeKey];

  const bgGrad = ctx.createLinearGradient(0, 0, 0, 1080);
  bgGrad.addColorStop(0, activeTheme.bgGrad[0]);
  bgGrad.addColorStop(1, activeTheme.bgGrad[1]);
  ctx.fillStyle = bgGrad;
  ctx.fillRect(0, 0, 1920, 1080);

  if (AppState.currentScreen === 'ATTRACT') {
    renderAttractVisuals(ctx, activeTheme);
  } else if (AppState.currentScreen === 'CALIBRATION') {
    renderCalibrationGrid(ctx);
    
    if (AppState.calibrationStage === 2) {
      targetManager.drawReachTargets(ctx);
    }

    poseTracker.drawSkeleton(ctx, 0.9);
    poseTracker.drawGloveIndicators(ctx, 1.0);
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

function renderCalibrationGrid(ctx) {
  ctx.save();
  ctx.strokeStyle = 'rgba(6, 182, 212, 0.15)';
  ctx.lineWidth = 1;
  const gridSize = 80;

  for (let x = 0; x < 1920; x += gridSize) {
    ctx.beginPath();
    ctx.moveTo(x, 0); ctx.lineTo(x, 1080);
    ctx.stroke();
  }
  for (let y = 0; y < 1080; y += gridSize) {
    ctx.beginPath();
    ctx.moveTo(0, y); ctx.lineTo(1920, y);
    ctx.stroke();
  }
  ctx.restore();
}

function renderArcadeHUD(ctx, theme) {
  ctx.save();

  ctx.fillStyle = '#ffffff';
  ctx.font = '900 60px Orbitron';
  ctx.textAlign = 'left';
  ctx.fillText(`SCORE: ${AppState.score.toLocaleString()}`, 80, 90);

  ctx.textAlign = 'right';
  const timeStr = `00:${AppState.timeRemaining < 10 ? '0' : ''}${AppState.timeRemaining}`;
  ctx.fillStyle = AppState.timeRemaining <= 10 ? '#ef4444' : theme.accent;
  ctx.fillText(timeStr, 1840, 90);

  if (AppState.combo > 1) {
    ctx.textAlign = 'center';
    ctx.fillStyle = '#f59e0b';
    ctx.font = '900 48px Orbitron';
    ctx.fillText(`COMBO x${AppState.combo}`, 960, 90);
  }

  drawCircularGauge(ctx, 150, 960, 55, AppState.recentPower, 100, "POWER", "#ef4444");
  
  const accuracy = AppState.totalPunches > 0 ? Math.floor((AppState.successfulHits / AppState.totalPunches) * 100) : 100;
  drawCircularGauge(ctx, 290, 960, 55, accuracy, 100, "ACCURACY", "#10b981");

  ctx.restore();
}

function drawCircularGauge(ctx, x, y, radius, value, maxVal, label, color) {
  const pct = Math.min(1.0, value / maxVal);
  const startAngle = Math.PI * 0.75;
  const endAngle = Math.PI * 2.25;
  const currentAngle = startAngle + (endAngle - startAngle) * pct;

  ctx.save();
  ctx.lineWidth = 10;

  ctx.strokeStyle = 'rgba(255,255,255,0.1)';
  ctx.beginPath();
  ctx.arc(x, y, radius, startAngle, endAngle);
  ctx.stroke();

  ctx.strokeStyle = color;
  ctx.shadowColor = color;
  ctx.shadowBlur = 12;
  ctx.beginPath();
  ctx.arc(x, y, radius, startAngle, currentAngle);
  ctx.stroke();

  ctx.fillStyle = '#ffffff';
  ctx.font = '700 12px Orbitron';
  ctx.textAlign = 'center';
  ctx.fillText(label, x, y + 5);
  ctx.font = '900 18px Orbitron';
  ctx.fillText(`${value}${label === 'ACCURACY' ? '%' : ''}`, x, y - 14);

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

function getLeaderboard() {
  const stored = localStorage.getItem('neoBoxingLeaderboard');
  return stored ? JSON.parse(stored) : DEFAULT_LEADERBOARD;
}

function saveLeaderboard(data) {
  localStorage.setItem('neoBoxingLeaderboard', JSON.stringify(data));
}

function renderLeaderboardUI() {
  const leaderboard = getLeaderboard();
  const listEl = document.getElementById('leaderboard-list');
  listEl.innerHTML = '';

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

/* =========================================================================
   GAME CONTROLLER
   ========================================================================= */
function switchScreen(targetScreen) {
  AppState.currentScreen = targetScreen;

  const overlays = ['attract-overlay', 'menu-overlay', 'calibration-overlay', 'countdown-overlay', 'gameover-overlay', 'highscore-overlay', 'leaderboard-overlay'];
  overlays.forEach(id => document.getElementById(id).classList.add('hidden'));

  if (targetScreen === 'ATTRACT') {
    document.getElementById('attract-overlay').classList.remove('hidden');
  } else if (targetScreen === 'MENU') {
    document.getElementById('menu-overlay').classList.remove('hidden');
  } else if (targetScreen === 'CALIBRATION') {
    AppState.calibrationStage = 1;
    AppState.reachCorners = { topLeft: false, topRight: false, bottomLeft: false, bottomRight: false };
    document.getElementById('calib-stage1-container').classList.remove('hidden');
    document.getElementById('calib-stage2-container').classList.add('hidden');
    document.getElementById('calib-stage-badge').textContent = "STAGE 1 / 2: DISTANCE & ALIGNMENT";
    document.getElementById('calib-status-text').textContent = "STEP INTO POSITION";
    document.getElementById('calib-sub-text').textContent = "Stand back until your shoulders and extended wrists fit within the target boundary boxes.";
    
    document.getElementById('calibration-overlay').classList.remove('hidden');
    announcer.speak("Step into position and extend both arms");
  } else if (targetScreen === 'COUNTDOWN') {
    document.getElementById('countdown-overlay').classList.remove('hidden');
  } else if (targetScreen === 'GAMEOVER') {
    document.getElementById('gameover-overlay').classList.remove('hidden');
    renderGameOverStats();
  } else if (targetScreen === 'LEADERBOARD') {
    renderLeaderboardUI();
    document.getElementById('leaderboard-overlay').classList.remove('hidden');
  }
}

function startCountdown() {
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
      soundEngine.playCountdownBeep(true);
      announcer.speak("Fight!");
      setTimeout(launchGameplay, 800);
    }
  }, 1000);
}

function launchGameplay() {
  switchScreen('PLAYING');
  AppState.score = 0;
  AppState.combo = 0;
  AppState.totalPunches = 0;
  AppState.successfulHits = 0;
  AppState.timeRemaining = 60;
  targetManager.reset(AppState.targetCountSetting);

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

function finishGame() {
  announcer.speak("Game Over");
  
  const leaderboard = getLeaderboard();
  const isHighScore = leaderboard.some(entry => AppState.score > entry.score) || leaderboard.length < 5;

  if (isHighScore) {
    initialsState = ['A', 'A', 'A'];
    currentInitialSlot = 0;
    updateInitialsDisplay();
    document.getElementById('final-score-display').textContent = `FINAL SCORE: ${AppState.score.toLocaleString()}`;
    document.getElementById('highscore-overlay').classList.remove('hidden');
  } else {
    switchScreen('GAMEOVER');
  }
}

function renderGameOverStats() {
  document.getElementById('stat-final-score').textContent = AppState.score.toLocaleString();
  const accuracy = AppState.totalPunches > 0 ? Math.floor((AppState.successfulHits / AppState.totalPunches) * 100) : 100;
  document.getElementById('stat-final-accuracy').textContent = `${accuracy}%`;
  document.getElementById('stat-final-combo').textContent = `x${AppState.maxCombo}`;
  document.getElementById('stat-final-power').textContent = AppState.maxPower;
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
  poseTracker.init(() => startCountdown());
  renderGameLoop();

  // Attract Start Button
  document.getElementById('btn-start-game').addEventListener('click', () => {
    soundEngine.ensureContext();
    switchScreen('MENU');
  });

  // Proceed to Calibration
  document.getElementById('btn-proceed-calibration').addEventListener('click', () => {
    switchScreen('CALIBRATION');
  });

  // Bypass Calibration Button
  document.getElementById('btn-bypass-calib').addEventListener('click', () => {
    announcer.speak("Position Confirmed");
    startCountdown();
  });

  document.getElementById('btn-cancel-calib').addEventListener('click', () => {
    switchScreen('MENU');
  });

  // Fullscreen Toggle Button
  document.getElementById('btn-fullscreen').addEventListener('click', () => {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen().catch(err => console.log(err));
    } else {
      document.exitFullscreen();
    }
  });

  // Glove Size Adjustment Buttons
  document.getElementById('btn-glove-minus').addEventListener('click', () => {
    AppState.gloveRadius = Math.max(30, AppState.gloveRadius - 5);
    document.getElementById('glove-size-display').textContent = AppState.gloveRadius;
  });
  document.getElementById('btn-glove-plus').addEventListener('click', () => {
    AppState.gloveRadius = Math.min(100, AppState.gloveRadius + 5);
    document.getElementById('glove-size-display').textContent = AppState.gloveRadius;
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
    });
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
  setupSelectionGroup('audio-profile-selector', 'audio-btn', (val) => AppState.audioProfile = val);

  // Voice Announcer Checkbox
  document.getElementById('chk-announcer').addEventListener('change', (e) => {
    AppState.announcerEnabled = e.target.checked;
  });

  // Leaderboard Modal Buttons
  document.getElementById('btn-show-leaderboard').addEventListener('click', () => switchScreen('LEADERBOARD'));
  document.getElementById('btn-close-leaderboard').addEventListener('click', () => switchScreen('MENU'));

  // Game Over Screen Buttons
  document.getElementById('btn-restart-game').addEventListener('click', () => switchScreen('CALIBRATION'));
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
    const leaderboard = getLeaderboard();
    const now = new Date();
    const dateStr = now.toLocaleDateString('en-US', { month: 'short', day: '2-digit', year: 'numeric' }).toUpperCase();
    
    leaderboard.push({ rank: 0, initials, score: AppState.score, date: dateStr });
    leaderboard.sort((a, b) => b.score - a.score);
    saveLeaderboard(leaderboard.slice(0, 5));

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

    if (AppState.currentScreen === 'CALIBRATION' && AppState.calibrationStage === 2) {
      targetManager.checkReachTouch(clickX, clickY, AppState.gloveRadius);
      poseTracker.checkStage2Calibration();
    } else if (AppState.currentScreen === 'PLAYING') {
      AppState.totalPunches++;
      const hit = targetManager.checkPunchCollision(clickX, clickY, AppState.gloveRadius, 'ANY', 85);
      if (!hit && AppState.gameMode === 'COMBO') {
        soundEngine.playMissSound();
        AppState.combo = 0;
      }
    }
  });
});
