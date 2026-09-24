export const AppState = {
  currentScreen: 'ATTRACT', // ATTRACT, MENU, CALIBRATION, COUNTDOWN, PLAYING, GAMEOVER, HIGH_SCORE, LEADERBOARD
  gameMode: 'RUSH', // RUSH, COMBO, POWER, DRILL
  targetCountSetting: 5,
  activeThemeKey: 'NEO_BLUE',
  audioProfile: 'REVV', // AMBIENT or REVV
  announcerEnabled: true,
  gloveRadius: 55, // Configurable glove boundary radius
  
  // Custom Target & Glove Colors
  targetColor: '#00f0ff',
  leftGloveColor: '#ef4444',
  rightGloveColor: '#06b6d4',
  
  // Live Gameplay State
  score: 0,
  combo: 0,
  maxCombo: 0,
  totalPunches: 0,
  successfulHits: 0,
  recentPower: 0,
  maxPower: 0,
  timeRemaining: 60,
  currentDrillPunch: 'JAB', // Required punch in Drill mode
  
  // Tracking & 2-Stage Calibration State
  calibrationStage: 1, // 1: Distance, 2: Reach 4 Corners
  isCalibrated: false,
  calibHoldTimer: 0,
  reachCorners: {
    topLeft: false,
    topRight: false,
    bottomLeft: false,
    bottomRight: false
  },
  // Calibrated Motion Bounds (in normalized video space 0..1)
  reachBounds: {
    minX: 0.1,
    maxX: 0.9,
    minY: 0.1,
    maxY: 0.9
  }
};

export function hexToRgba(hex, alpha = 0.35) {
  if (!hex) return `rgba(6, 182, 212, ${alpha})`;
  let c = hex.replace('#', '');
  if (c.length === 3) c = c.split('').map(x => x + x).join('');
  const num = parseInt(c, 16);
  if (isNaN(num)) return `rgba(6, 182, 212, ${alpha})`;
  return `rgba(${(num >> 16) & 255}, ${(num >> 8) & 255}, ${num & 255}, ${alpha})`;
}