"""speech.py — pluggable speech-to-text for the voice action-labeling button.

LOCAL-ONLY: no network / cloud STT is ever used. Detects a backend at
construction (none are hard dependencies) and reports available() so the GUI
can disable the mic button until one is installed. Backends tried, in order:
  1. sounddevice + faster-whisper   (recommended local)
  2. sounddevice + openai-whisper   (local, reuses torch)
  3. SpeechRecognition + a LOCAL engine (whisper / vosk)   — never Google/cloud

transcribe_once() BLOCKS on audio capture + recognition, so callers must run it
off the GUI thread (the GUI uses a QThread worker).

Install (recommended):
    pip install faster-whisper sounddevice
"""

from __future__ import annotations

import importlib
import os

os.environ.setdefault("HF_HUB_DISABLE_SYMLINKS_WARNING", "1")


def _have(mod: str) -> bool:
    try:
        importlib.import_module(mod)
        return True
    except Exception:
        return False


class SpeechToText:
    def __init__(self, language: str = "en", whisper_model: str = "base.en", device=None):
        self.language = language
        self.whisper_model = whisper_model
        self.device = device          # sounddevice input index; None = system default
        self._fw = None
        self._w = None
        # Local-only: prefer direct offline whisper capture; use SpeechRecognition
        # ONLY if a LOCAL engine (whisper/vosk) is present — never cloud/Google.
        if _have("sounddevice") and _have("faster_whisper"):
            self._backend = "faster_whisper"
        elif _have("sounddevice") and _have("whisper"):
            self._backend = "whisper"
        elif (_have("speech_recognition") and (_have("pyaudio") or _have("sounddevice"))
              and (_have("whisper") or _have("vosk"))):
            self._backend = "speech_recognition"
        else:
            self._backend = None

    def available(self) -> bool:
        return self._backend is not None

    @property
    def name(self) -> str:
        return self._backend or "unavailable"

    def warmup(self) -> None:
        """Preload the model (downloads on first use) so the first live transcribe
        isn't a surprise wait. Safe to call from a background thread."""
        if self._backend == "faster_whisper" and self._fw is None:
            from faster_whisper import WhisperModel
            self._fw = WhisperModel(self.whisper_model, device="cpu", compute_type="int8")
        elif self._backend == "whisper" and self._w is None:
            import whisper
            self._w = whisper.load_model(self.whisper_model)

    def transcribe_once(self, seconds: float = 5.0) -> str:
        if self._backend == "speech_recognition":
            return self._sr_transcribe(seconds)
        if self._backend in ("faster_whisper", "whisper"):
            return self._whisper_transcribe(seconds)
        raise RuntimeError("no speech-to-text backend available")

    # -- backends --
    def _sr_transcribe(self, seconds: float) -> str:
        import speech_recognition as sr
        r = sr.Recognizer()
        with sr.Microphone() as source:
            r.adjust_for_ambient_noise(source, duration=0.3)
            audio = r.listen(source, timeout=max(1.0, seconds), phrase_time_limit=seconds)
        for fn in ("recognize_whisper", "recognize_vosk"):   # local engines only
            recognizer = getattr(r, fn, None)
            if recognizer is None:
                continue
            try:
                return str(recognizer(audio)).strip()
            except Exception:
                continue
        return ""

    def _whisper_transcribe(self, seconds: float) -> str:
        import numpy as np
        import sounddevice as sd
        hz = 16000
        audio = sd.rec(int(seconds * hz), samplerate=hz, channels=1, dtype="float32",
                       device=self.device)
        sd.wait()
        audio = np.asarray(audio, dtype="float32").reshape(-1)

        # Whisper HALLUCINATES text ("you", "thank you") on near-silent / very
        # quiet audio (verified: silence -> " You"). Gate on silence and
        # normalize quiet-but-real speech to a usable level before transcribing.
        peak = float(np.max(np.abs(audio))) if audio.size else 0.0
        if peak < 0.02:
            return ""                       # essentially silent -> no speech
        audio = audio * (0.8 / peak)

        if self._backend == "faster_whisper":
            from faster_whisper import WhisperModel
            if self._fw is None:
                self._fw = WhisperModel(self.whisper_model, device="cpu", compute_type="int8")
            segs, _ = self._fw.transcribe(
                audio, language=self.language, beam_size=5,
                vad_filter=True,                 # drop non-speech -> kills hallucinations
                condition_on_previous_text=False,
                no_speech_threshold=0.6,
                temperature=0.0,
            )
            return " ".join(s.text for s in segs).strip()
        import whisper
        if self._w is None:
            self._w = whisper.load_model(self.whisper_model)
        res = whisper.transcribe(self._w, audio, language=self.language,
                                 condition_on_previous_text=False, temperature=0.0)
        return str(res.get("text", "")).strip()


__all__ = ["SpeechToText"]
