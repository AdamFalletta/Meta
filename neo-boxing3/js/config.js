export const THEMES = {
  NEO_BLUE: {
    bgGrad: ['#030712', '#0f172a'],
    targetColor: '#00f0ff',
    accent: '#06b6d4',
    accentGlow: 'rgba(6, 182, 212, 0.4)',
    particleColors: ['#00f0ff', '#38bdf8', '#818cf8', '#ffffff']
  },
  REVV_RED: {
    bgGrad: ['#180202', '#2a0808'],
    targetColor: '#ff2a2a',
    accent: '#ef4444',
    accentGlow: 'rgba(239, 68, 68, 0.4)',
    particleColors: ['#ff2a2a', '#fca5a5', '#f87171', '#ffffff']
  },
  PURPLE_VOID: {
    bgGrad: ['#090214', '#1e0b36'],
    targetColor: '#e024ff',
    accent: '#c084fc',
    accentGlow: 'rgba(192, 132, 252, 0.4)',
    particleColors: ['#e024ff', '#c084fc', '#e879f9', '#ffffff']
  },
  CYBER_GREEN: {
    bgGrad: ['#02140a', '#082a18'],
    targetColor: '#00ff88',
    accent: '#34d399',
    accentGlow: 'rgba(52, 211, 153, 0.4)',
    particleColors: ['#00ff88', '#34d399', '#6ee7b7', '#ffffff']
  },
  GOLDEN_ARENA: {
    bgGrad: ['#140d02', '#2a1a08'],
    targetColor: '#ffaa00',
    accent: '#fbbf24',
    accentGlow: 'rgba(251, 191, 36, 0.4)',
    particleColors: ['#ffaa00', '#fbbf24', '#fde047', '#ffffff']
  },
  NEON_ORANGE: {
    bgGrad: ['#160800', '#2b1203'],
    targetColor: '#ff7a00',
    accent: '#fb923c',
    accentGlow: 'rgba(251, 146, 60, 0.4)',
    particleColors: ['#ff7a00', '#fb923c', '#fdba74', '#ffffff']
  },
  ICE_WHITE: {
    bgGrad: ['#050b16', '#0c1c30'],
    targetColor: '#dff6ff',
    accent: '#bae6fd',
    accentGlow: 'rgba(186, 230, 253, 0.4)',
    particleColors: ['#dff6ff', '#bae6fd', '#7dd3fc', '#ffffff']
  },
  TOXIC_LIME: {
    bgGrad: ['#0a1402', '#1c2a05'],
    targetColor: '#b6ff1a',
    accent: '#a3e635',
    accentGlow: 'rgba(163, 230, 53, 0.4)',
    particleColors: ['#b6ff1a', '#a3e635', '#d9f99d', '#ffffff']
  }
};

export const PUNCH_TYPES = ['JAB', 'HOOK', 'UPPERCUT'];

export const DEFAULT_LEADERBOARD = [
  { rank: 1, initials: "NEO", score: 12450, date: "SEP 08 2026" },
  { rank: 2, initials: "REVV", score: 9820, date: "SEP 07 2026" },
  { rank: 3, initials: "BOX", score: 7400, date: "SEP 06 2026" },
  { rank: 4, initials: "MAX", score: 6250, date: "SEP 05 2026" },
  { rank: 5, initials: "ACE", score: 4800, date: "SEP 04 2026" }
];

// Separate boards so the 3/5/7/9-target modes each have their own rankings.
export const DEFAULT_LEADERBOARDS = {
  3: [
    { rank: 1, initials: "JAB", score: 9820, date: "SEP 08 2026" },
    { rank: 2, initials: "KAI", score: 8140, date: "SEP 07 2026" },
    { rank: 3, initials: "ROX", score: 6890, date: "SEP 06 2026" },
    { rank: 4, initials: "MIA", score: 5520, date: "SEP 05 2026" },
    { rank: 5, initials: "DEX", score: 4310, date: "SEP 04 2026" }
  ],
  5: DEFAULT_LEADERBOARD,
  7: [
    { rank: 1, initials: "REV", score: 18450, date: "SEP 08 2026" },
    { rank: 2, initials: "ZEN", score: 15920, date: "SEP 07 2026" },
    { rank: 3, initials: "KNO", score: 13280, date: "SEP 06 2026" },
    { rank: 4, initials: "FLA", score: 10840, date: "SEP 05 2026" },
    { rank: 5, initials: "TOR", score: 8960, date: "SEP 04 2026" }
  ],
  9: [
    { rank: 1, initials: "MAX", score: 26780, date: "SEP 08 2026" },
    { rank: 2, initials: "ACE", score: 22140, date: "SEP 07 2026" },
    { rank: 3, initials: "NVA", score: 18760, date: "SEP 06 2026" },
    { rank: 4, initials: "BLZ", score: 15420, date: "SEP 05 2026" },
    { rank: 5, initials: "STL", score: 12180, date: "SEP 04 2026" }
  ]
};