export const AppState = {
  currentScreen: 'ATTRACT', // ATTRACT, MENU, COUNTDOWN, PLAYING, GAMEOVER, HIGH_SCORE, LEADERBOARD
  gameMode: 'RUSH', // RUSH, COMBO, POWER, DRILL
  targetCountSetting: 5,
  leaderboardTargetCount: 5,
  activeThemeKey: 'NEO_BLUE',
  audioProfile: 'REVV', // AMBIENT or REVV
  announcerEnabled: true,
  gloveRadius: 55, // Configurable glove boundary radius
  
  // Custom Target & Glove Colors
  // null = follow the active arena theme's target color
  targetColor: null,
  leftGloveColor: '#ef4444',
  rightGloveColor: '#06b6d4',

  // Uploaded image drawn over each gameplay target
  targetImage: null, // Downscaled canvas (or image) drawn over each target
  targetImageName: '',
  
  // Live Gameplay State
  score: 0,
  combo: 0,
  maxCombo: 0,
  totalPunches: 0,
  successfulHits: 0,
  recentPower: 0,
  maxPower: 0,
  timeRemaining: 60,
  currentDrillPunch: 'JAB' // Required punch in Drill mode
};

export function hexToRgba(hex, alpha = 0.35) {
  if (!hex) return `rgba(6, 182, 212, ${alpha})`;
  let c = hex.replace('#', '');
  if (c.length === 3) c = c.split('').map(x => x + x).join('');
  const num = parseInt(c, 16);
  if (isNaN(num)) return `rgba(6, 182, 212, ${alpha})`;
  return `rgba(${(num >> 16) & 255}, ${(num >> 8) & 255}, ${num & 255}, ${alpha})`;
}