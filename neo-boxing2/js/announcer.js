import { AppState } from './state.js';

class AnnouncerSystem {
  constructor() {
    this.synth = window.speechSynthesis;
  }

  speak(phrase) {
    if (!AppState.announcerEnabled || !this.synth) return;
    this.synth.cancel();
    const utterance = new SpeechSynthesisUtterance(phrase);
    utterance.rate = 1.1;
    utterance.pitch = 0.9;
    utterance.volume = 0.9;
    this.synth.speak(utterance);
  }
}

export const announcer = new AnnouncerSystem();