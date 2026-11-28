import { AppState } from './state.js';

class SoundEngine {
  constructor() {
    this.ctx = null;
    this.masterGain = null;
    this.crowdHumSource = null;
    this.crowdHumGain = null;
    this.crowdHumBandGain = null;
  }

  createNoiseBuffer(seconds = 1) {
    const length = Math.max(1, Math.floor(this.ctx.sampleRate * seconds));
    const buffer = this.ctx.createBuffer(1, length, this.ctx.sampleRate);
    const data = buffer.getChannelData(0);
    for (let i = 0; i < length; i++) {
      data[i] = Math.random() * 2 - 1;
    }
    return buffer;
  }

  init() {
    if (this.ctx) return;
    const AudioCtx = window.AudioContext || window.webkitAudioContext;
    this.ctx = new AudioCtx();
    this.masterGain = this.ctx.createGain();
    this.masterGain.gain.value = 0.8;
    this.masterGain.connect(this.ctx.destination);
  }

  ensureContext() {
    if (!this.ctx) this.init();
    if (this.ctx.state === 'suspended') {
      this.ctx.resume();
    }
  }

  // Target Spawn Tone
  playTargetSpawn() {
    this.ensureContext();
    const osc = this.ctx.createOscillator();
    const gain = this.ctx.createGain();
    
    osc.type = 'sine';
    osc.frequency.setValueAtTime(220, this.ctx.currentTime);
    osc.frequency.exponentialRampToValueAtTime(660, this.ctx.currentTime + 0.12);
    
    gain.gain.setValueAtTime(0.15, this.ctx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.01, this.ctx.currentTime + 0.12);

    osc.connect(gain);
    gain.connect(this.masterGain);

    osc.start();
    osc.stop(this.ctx.currentTime + 0.12);
  }

  // Dynamic Impact Audio Generator based on punch velocity & audio mode
  playTargetHit(power = 50) {
    this.ensureContext();
    const now = this.ctx.currentTime;
    const isRevV = AppState.audioProfile === 'REVV';
    
    const osc = this.ctx.createOscillator();
    const oscGain = this.ctx.createGain();
    
    const startFreq = isRevV ? 160 + power * 2.5 : 120 + power;
    osc.type = isRevV ? 'triangle' : 'sine';
    osc.frequency.setValueAtTime(startFreq, now);
    osc.frequency.exponentialRampToValueAtTime(25, now + 0.28);
    
    const maxVol = Math.min(1.0, 0.35 + (power / 100) * 0.65);
    oscGain.gain.setValueAtTime(maxVol, now);
    oscGain.gain.exponentialRampToValueAtTime(0.001, now + 0.28);

    osc.connect(oscGain);

    // Noise Snap Layer
    const bufferSize = this.ctx.sampleRate * 0.1;
    const buffer = this.ctx.createBuffer(1, bufferSize, this.ctx.sampleRate);
    const data = buffer.getChannelData(0);
    for (let i = 0; i < bufferSize; i++) {
      data[i] = Math.random() * 2 - 1;
    }

    const noise = this.ctx.createBufferSource();
    noise.buffer = buffer;

    const filter = this.ctx.createBiquadFilter();
    filter.type = 'bandpass';
    filter.frequency.value = isRevV ? 1400 : 900;

    const noiseGain = this.ctx.createGain();
    noiseGain.gain.setValueAtTime(maxVol * 0.6, now);
    noiseGain.gain.exponentialRampToValueAtTime(0.01, now + 0.12);

    noise.connect(filter);
    filter.connect(noiseGain);

    if (isRevV && power > 50) {
      const distortion = this.ctx.createWaveShaper();
      distortion.curve = this.makeDistortionCurve(power * 2);
      oscGain.connect(distortion);
      noiseGain.connect(distortion);
      distortion.connect(this.masterGain);
    } else {
      oscGain.connect(this.masterGain);
      noiseGain.connect(this.masterGain);
    }

    osc.start(now);
    noise.start(now);
    osc.stop(now + 0.28);
    noise.stop(now + 0.12);

    // Big punches get a white-noise crowd roar.
    if (power >= 60) {
      this.playCrowdCheer(power);
    }
  }

  // Boxing bell — three bright metallic strikes.
  playBellStrike(delay = 0, volume = 1) {
    this.ensureContext();
    const start = this.ctx.currentTime + delay;
    const partials = [
      { freq: 660, gain: 0.2, decay: 1.05, type: 'sine' },
      { freq: 1320, gain: 0.1, decay: 0.85, type: 'sine' },
      { freq: 1980, gain: 0.06, decay: 0.65, type: 'triangle' },
      { freq: 2640, gain: 0.03, decay: 0.45, type: 'sine' }
    ];

    partials.forEach(partial => {
      const osc = this.ctx.createOscillator();
      const gain = this.ctx.createGain();
      const peak = Math.max(0.0002, partial.gain * volume);

      osc.type = partial.type;
      osc.frequency.setValueAtTime(partial.freq, start);

      gain.gain.setValueAtTime(0.0001, start);
      gain.gain.exponentialRampToValueAtTime(peak, start + 0.012);
      gain.gain.exponentialRampToValueAtTime(0.0001, start + partial.decay);

      osc.connect(gain);
      gain.connect(this.masterGain);

      osc.start(start);
      osc.stop(start + partial.decay + 0.05);
    });
  }

  playDingDingDing(volume = 1) {
    this.playBellStrike(0, volume);
    this.playBellStrike(0.45, volume);
    this.playBellStrike(0.9, volume);
  }

  // Looping crowd murmur for the arena.
  startCrowdHum() {
    this.ensureContext();
    if (this.crowdHumSource) return;

    const now = this.ctx.currentTime;
    const source = this.ctx.createBufferSource();
    source.buffer = this.createNoiseBuffer(3);
    source.loop = true;

    const lowpass = this.ctx.createBiquadFilter();
    lowpass.type = 'lowpass';
    lowpass.frequency.value = 520;
    lowpass.Q.value = 0.7;

    const murmur = this.ctx.createBiquadFilter();
    murmur.type = 'bandpass';
    murmur.frequency.value = 950;
    murmur.Q.value = 0.55;

    const gain = this.ctx.createGain();
    gain.gain.setValueAtTime(0.0001, now);
    gain.gain.exponentialRampToValueAtTime(0.07, now + 1.2);

    const bandGain = this.ctx.createGain();
    bandGain.gain.setValueAtTime(0.0001, now);
    bandGain.gain.exponentialRampToValueAtTime(0.022, now + 1.2);

    source.connect(lowpass);
    lowpass.connect(gain);
    gain.connect(this.masterGain);

    source.connect(murmur);
    murmur.connect(bandGain);
    bandGain.connect(this.masterGain);

    source.start(now);

    this.crowdHumSource = source;
    this.crowdHumGain = gain;
    this.crowdHumBandGain = bandGain;
  }

  stopCrowdHum() {
    if (!this.crowdHumSource) return;
    const now = this.ctx.currentTime;
    const source = this.crowdHumSource;

    [this.crowdHumGain, this.crowdHumBandGain].forEach(gain => {
      if (!gain) return;
      gain.gain.cancelScheduledValues(now);
      gain.gain.setValueAtTime(Math.max(0.0001, gain.gain.value), now);
      gain.gain.exponentialRampToValueAtTime(0.0001, now + 0.5);
    });

    source.stop(now + 0.6);

    this.crowdHumSource = null;
    this.crowdHumGain = null;
    this.crowdHumBandGain = null;
  }

  // White-noise crowd cheer for powerful hits and the fight bell.
  playCrowdCheer(power = 70) {
    this.ensureContext();
    const now = this.ctx.currentTime;
    const intensity = Math.max(0.25, Math.min(1, power / 100));

    const source = this.ctx.createBufferSource();
    source.buffer = this.createNoiseBuffer(1.4);

    const bandpass = this.ctx.createBiquadFilter();
    bandpass.type = 'bandpass';
    bandpass.frequency.value = 1000 + power * 8;
    bandpass.Q.value = 0.45;

    const highpass = this.ctx.createBiquadFilter();
    highpass.type = 'highpass';
    highpass.frequency.value = 480;

    const gain = this.ctx.createGain();
    gain.gain.setValueAtTime(0.0001, now);
    gain.gain.exponentialRampToValueAtTime(0.1 * intensity + 0.04, now + 0.09);
    gain.gain.exponentialRampToValueAtTime(0.05 * intensity + 0.02, now + 0.45);
    gain.gain.exponentialRampToValueAtTime(0.0001, now + 1.25);

    source.connect(bandpass);
    bandpass.connect(highpass);
    highpass.connect(gain);
    gain.connect(this.masterGain);

    source.start(now);
    source.stop(now + 1.3);
  }

  playMissSound() {
    this.ensureContext();
    const now = this.ctx.currentTime;
    const osc = this.ctx.createOscillator();
    const gain = this.ctx.createGain();
    
    osc.type = 'sawtooth';
    osc.frequency.setValueAtTime(140, now);
    osc.frequency.exponentialRampToValueAtTime(35, now + 0.2);

    gain.gain.setValueAtTime(0.18, now);
    gain.gain.exponentialRampToValueAtTime(0.01, now + 0.2);

    osc.connect(gain);
    gain.connect(this.masterGain);

    osc.start(now);
    osc.stop(now + 0.2);
  }

  playComboBeep(comboCount) {
    this.ensureContext();
    const now = this.ctx.currentTime;
    const osc = this.ctx.createOscillator();
    const gain = this.ctx.createGain();
    
    const scale = [261.63, 293.66, 329.63, 392.00, 440.00, 523.25, 587.33, 659.25];
    const note = scale[comboCount % scale.length];

    osc.type = 'sine';
    osc.frequency.setValueAtTime(note, now);
    
    gain.gain.setValueAtTime(0.2, now);
    gain.gain.exponentialRampToValueAtTime(0.01, now + 0.15);

    osc.connect(gain);
    gain.connect(this.masterGain);

    osc.start(now);
    osc.stop(now + 0.15);
  }

  playCountdownBeep(isFinal = false) {
    this.ensureContext();
    const now = this.ctx.currentTime;
    const osc = this.ctx.createOscillator();
    const gain = this.ctx.createGain();

    osc.type = 'sine';
    osc.frequency.setValueAtTime(isFinal ? 1200 : 600, now);

    gain.gain.setValueAtTime(0.3, now);
    gain.gain.exponentialRampToValueAtTime(0.01, now + (isFinal ? 0.4 : 0.2));

    osc.connect(gain);
    gain.connect(this.masterGain);

    osc.start(now);
    osc.stop(now + (isFinal ? 0.4 : 0.2));
  }

  makeDistortionCurve(amount) {
    const k = typeof amount === 'number' ? amount : 50;
    const n_samples = 44100;
    const curve = new Float32Array(n_samples);
    const deg = Math.PI / 180;
    for (let i = 0; i < n_samples; ++i) {
      const x = (i * 2) / n_samples - 1;
      curve[i] = ((3 + k) * x * 20 * deg) / (Math.PI + k * Math.abs(x));
    }
    return curve;
  }
}

export const soundEngine = new SoundEngine();