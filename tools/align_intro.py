#!/usr/bin/env python3
"""Offline word timing for the prerecorded introduction (optional faster-whisper).

Example: python align_intro.py web/public/avatar/introduction.mp3 /tmp/intro-words.json
Downloads the Whisper model on first run. Audio is processed locally, never uploaded.
Review output against the script before updating web/src/avatar/intro-words.ts.
"""
import argparse
import json
import subprocess


def main():
    import numpy as np
    from faster_whisper import WhisperModel
    ap = argparse.ArgumentParser()
    ap.add_argument('audio')
    ap.add_argument('output')
    args = ap.parse_args()
    pcm = subprocess.run(['ffmpeg', '-v', 'error', '-i', args.audio, '-ar', '16000',
                          '-ac', '1', '-f', 'f32le', '-'], check=True, capture_output=True).stdout
    audio = np.frombuffer(pcm, dtype=np.float32)
    model = WhisperModel('small', device='cpu', compute_type='int8', cpu_threads=6)
    segments, _ = model.transcribe(audio, language='ko', word_timestamps=True, beam_size=5,
        initial_prompt="막내 MAKNAE. Mobile Assistant for Kin's Needs And Errands. 저비용 모바일 매니퓰레이터.")
    words = [{'start': round(w.start, 3), 'end': round(w.end, 3), 'word': w.word.strip()}
             for segment in segments for w in segment.words]
    with open(args.output, 'w') as out:
        json.dump(words, out, ensure_ascii=False, indent=2)


if __name__ == '__main__':
    main()
