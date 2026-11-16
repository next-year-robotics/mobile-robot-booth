#!/usr/bin/env python3
"""config/clips.yaml의 고정 대사를 소리 파일로 만든다. 이미 있는 것은 건너뛴다(문장이 바뀌면 다시 만든다).

    python3 tools/make_clips.py                         # ElevenLabs Hina (유료 요금제 필요). 기본값
    python3 tools/make_clips.py --provider gemini       # Gemini TTS (하루 100회 한도)
    python3 tools/make_clips.py --provider live         # Gemini Live로 읽힌다 (받아쓰기가 안 와서 확인이 안 된다)
    python3 tools/make_clips.py --only greet,attract --force

결과: web/public/avatar/clips/<키>_<번호>.(mp3|wav), manifest.json. 이 폴더는 git에 올리지 않는다.
키 파일: ~/.config/mobile-robot/gemini.env (GEMINI_API_KEY), elevenlabs.env (ELEVENLABS_API_KEY)
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
import wave
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'web' / 'public' / 'avatar' / 'clips'
KEYS = Path('~/.config/mobile-robot').expanduser()


def env(file: str, name: str) -> str:
    for line in (KEYS / file).read_text().splitlines():
        if line.startswith(name + '='):
            return line.split('=', 1)[1].strip().strip('"\'')
    raise SystemExit(f'{KEYS / file}에 {name}가 없다')


def post(url: str, body: dict, headers: dict, timeout: float = 90) -> bytes:
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={'Content-Type': 'application/json', **headers})
    for attempt in range(8):
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            text = e.read().decode(errors='replace')
            if e.code == 429 and attempt < 7 and 'per_day' not in text.lower() and 'per day' not in text.lower():
                time.sleep(25)                      # 분당 요청 한도. 기다렸다가 다시 한다
                continue
            raise RuntimeError(f'HTTP {e.code}: {" ".join(text.split())[:700]}') from None
    raise RuntimeError('재시도 초과')


def transcribe(wav_bytes: bytes, mime: str = 'audio/wav') -> str:
    data = json.loads(post(
        'https://generativelanguage.googleapis.com/v1beta/models/gemini-3.8-flash:generateContent',
        {'contents': [{'parts': [{'inlineData': {'mimeType': mime, 'data': base64.b64encode(wav_bytes).decode()}},
                                 {'text': '이 오디오에서 들리는 말을 들리는 그대로만 받아 적어라. 설명이나 괄호를 붙이지 않는다.'}]}],
         'generationConfig': {'temperature': 0}},
        {'x-goog-api-key': env('gemini.env', 'GEMINI_API_KEY')}))
    return data['candidates'][0]['content']['parts'][0]['text'].strip()


def same_words(a: str, b: str) -> float:
    import difflib
    import re
    norm = lambda t: re.sub(r'[^0-9a-z가-힣]', '', t.lower())  # noqa: E731
    return difflib.SequenceMatcher(None, norm(a), norm(b)).ratio()


def gemini(cfg: dict, style: str, say: str, tone: str, path: Path) -> Path:
    """Gemini TTS로 읽고, 받아쓰기로 문장이 맞는지 확인한다.

    'Say <말투>: <문장>' 꼴로 말투를 줄 수 있지만 가끔 지시문까지 소리 내어 읽는다. 그래서 만든 소리를
    다시 받아 적어 원문과 견주고, 다르면 다시 만든다. 끝까지 안 되면 말투 지시 없이 문장만 읽힌다.
    """
    v = cfg['voice']['gemini']
    prompts = [f'Say {tone + ", " if tone else ""}{style}: {say}'] * 3 + [f'Say {tone}: {say}' if tone else say, say, say]
    best = None
    wav = path.with_suffix('.wav')
    for prompt in prompts:
        data = json.loads(post(
            f"https://generativelanguage.googleapis.com/v1beta/models/{v['model']}:generateContent",
            {'contents': [{'parts': [{'text': prompt}]}],
             'generationConfig': {'responseModalities': ['AUDIO'],
                                  'speechConfig': {'voiceConfig': {'prebuiltVoiceConfig': {'voiceName': v['voice']}}}}},
            {'x-goog-api-key': env('gemini.env', 'GEMINI_API_KEY')}))
        part = data['candidates'][0]['content']['parts'][0]['inlineData']
        rate = 24000
        for field in part.get('mimeType', '').split(';'):
            if field.strip().startswith('rate='):
                rate = int(field.split('=')[1])
        with wave.open(str(wav), 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate)
            w.writeframes(base64.b64decode(part['data']))
        heard = transcribe(wav.read_bytes())
        score = same_words(heard, say)
        if best is None or score > best[0]:
            best = (score, wav.read_bytes(), heard)
        if score >= 0.85:
            break
        print(f'  다시 만든다 ({score:.2f}): "{heard[:50]}"', flush=True)
    if best[0] < 0.85:
        wav.unlink(missing_ok=True)
        raise RuntimeError(f'받아쓰기가 원문과 다르다 ({best[0]:.2f}): "{best[2][:60]}"')
    wav.write_bytes(best[1])
    return to_mp3(wav)


def live(cfg: dict, style: str, say: str, tone: str, path: Path) -> Path:
    """Gemini Live 세션에 문장을 주고 읽힌 소리를 받는다. 실시간 대화와 같은 모델·목소리라 이어 들어도 어색하지 않다.

    모델이 문장을 바꿔 말할 수 있어서, 같이 오는 받아쓰기를 원문과 견주고 다르면 다시 시킨다.
    """
    import asyncio

    import websockets
    v = cfg['voice']['live']
    url = ('wss://generativelanguage.googleapis.com/ws/google.ai.generativelanguage.v1beta.GenerativeService.'
           'BidiGenerateContent?key=' + env('gemini.env', 'GEMINI_API_KEY'))
    system = ('너는 성우다. 사용자가 보낸 "대사:" 뒤의 문장을 한 글자도 바꾸지 말고 그대로 소리 내어 읽기만 한다. '
              '대답하거나 덧붙이지 않는다. "말투:"에 적힌 감정으로 연기한다. 목소리는 ' + cfg['style_ko'])

    async def once() -> tuple[bytes, str]:
        async with websockets.connect(url, max_size=None) as ws:
            await ws.send(json.dumps({'setup': {
                'model': 'models/' + v['model'],
                'generationConfig': {'responseModalities': ['AUDIO'], 'speechConfig': {
                    'voiceConfig': {'prebuiltVoiceConfig': {'voiceName': v['voice']}}}},
                'systemInstruction': {'parts': [{'text': system}]},
                'outputAudioTranscription': {}}}))
            await ws.recv()
            await ws.send(json.dumps({'clientContent': {'turns': [{'role': 'user', 'parts': [
                {'text': f'말투: {tone or "밝게"}\n대사: {say}'}]}], 'turnComplete': True}}))
            pcm, heard = b'', ''
            while True:
                sc = json.loads(await asyncio.wait_for(ws.recv(), 30)).get('serverContent') or {}
                for part in (sc.get('modelTurn') or {}).get('parts', []):
                    if 'inlineData' in part:
                        pcm += base64.b64decode(part['inlineData']['data'])
                heard += (sc.get('outputTranscription') or {}).get('text', '')
                if sc.get('turnComplete'):
                    return pcm, heard.strip()

    best = None
    for _ in range(4):
        pcm, heard = asyncio.run(once())
        score = same_words(heard, say)
        if best is None or score > best[0]:
            best = (score, pcm, heard)
        if score >= 0.9:
            break
        print(f'  다시 읽힌다 ({score:.2f}): "{heard[:50]}"', flush=True)
        time.sleep(0.5)
    if best[0] < 0.8:
        raise RuntimeError(f'읽은 말이 원문과 다르다 ({best[0]:.2f}): "{best[2][:60]}"')
    wav = path.with_suffix('.wav')
    with wave.open(str(wav), 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000)
        w.writeframes(best[1])
    return to_mp3(wav)


def elevenlabs(cfg: dict, style: str, say: str, tone: str, path: Path) -> Path:
    """ElevenLabs v3/v4는 문장 앞 [태그]로 말투를 받는다. 태그를 소리 내어 읽거나 문장을 흘리는 일이 있어서
    받아쓰기로 원문과 견주고, 다르면 다시 만든다. 마지막 시도는 태그 없이 읽힌다."""
    v = cfg['voice']['elevenlabs']
    best = None
    for text in [f'[{tone}] {say}' if tone else say] * 3 + [say]:
        audio = post(
            f"https://api.elevenlabs.io/v1/text-to-speech/{v['voice_id']}?output_format=mp3_44100_128",
            {'text': text, 'model_id': v['model'], 'language_code': 'ko',
             'voice_settings': {k: v[k] for k in ('stability', 'similarity_boost', 'speed') if k in v}},
            {'xi-api-key': env('elevenlabs.env', 'ELEVENLABS_API_KEY')})
        try:
            heard = transcribe(audio, 'audio/mp3')
        except Exception as e:  # noqa: BLE001  받아쓰기가 안 되면 확인 없이 쓴다
            print(f'  받아쓰기 실패, 확인 없이 쓴다: {e}', flush=True)
            best = (1.0, audio, '')
            break
        score = same_words(heard, say)
        if best is None or score > best[0]:
            best = (score, audio, heard)
        if score >= 0.85:
            break
        print(f'  다시 만든다 ({score:.2f}): "{heard[:50]}"', flush=True)
    if best[0] < 0.85:
        raise RuntimeError(f'받아쓰기가 원문과 다르다 ({best[0]:.2f}): "{best[2][:60]}"')
    raw = path.with_suffix('.raw.mp3')
    raw.write_bytes(best[1])
    return to_mp3(raw, path.with_suffix('.mp3'))


def to_mp3(wav: Path, mp3: Path | None = None) -> Path:
    """앞뒤 무음을 자르고 음량을 맞춘다. ffmpeg가 없으면 받은 그대로 쓴다."""
    mp3 = mp3 or wav.with_suffix('.mp3')
    if not shutil.which('ffmpeg'):
        return wav if wav.suffix == '.wav' else wav.replace(mp3)
    trim = 'silenceremove=start_periods=1:start_threshold=-45dB:start_silence=0.05'
    r = subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', str(wav), '-af',
                        f'{trim},areverse,{trim},areverse,loudnorm=I=-16:TP=-1.5', '-ar', '44100', '-b:a', '128k', str(mp3)])
    if r.returncode != 0:
        return wav if wav.suffix == '.wav' else wav.replace(mp3)
    wav.unlink()
    return mp3


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--provider', choices=['live', 'gemini', 'elevenlabs'], default='elevenlabs')
    ap.add_argument('--only', default='', help='쉼표로 나눈 키')
    ap.add_argument('--force', action='store_true')
    args = ap.parse_args()
    cfg = yaml.safe_load((ROOT / 'config' / 'clips.yaml').read_text(encoding='utf-8'))
    OUT.mkdir(parents=True, exist_ok=True)
    manifest_path = OUT / 'manifest.json'
    old = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    make = {'live': live, 'gemini': gemini, 'elevenlabs': elevenlabs}[args.provider]
    only = {k for k in args.only.split(',') if k}
    manifest: dict[str, list] = {}
    made = failed = 0
    for key, variants in cfg['clips'].items():
        manifest[key] = []
        for i, c in enumerate(variants):
            sig = hashlib.sha1(f"{args.provider}|{json.dumps(cfg['voice'][args.provider], sort_keys=True)}|{c['say']}|{c.get('tone', '')}".encode()).hexdigest()[:12]
            prev = next((e for e in old.get(key, []) if e.get('text') == c['say']), None)
            keep = prev is not None and (OUT / prev['file']).exists() and \
                (prev.get('sig') == sig or (only and key not in only)) and not (args.force and (not only or key in only))
            if keep or (only and key not in only and prev and (OUT / prev['file']).exists()):
                manifest[key].append(prev)
                continue
            if only and key not in only:
                continue
            try:
                path = make(cfg, cfg['style'], c['say'], c.get('tone_ko' if args.provider == 'live' else 'tone', ''), OUT / f'{key}_{i}')
                manifest[key].append({'file': path.name, 'text': c['say'], 'sig': sig})
                made += 1
                print(f'{key}_{i}: {c["say"]}', flush=True)
                time.sleep(0.3)
            except Exception as e:  # noqa: BLE001
                failed += 1
                print(f'{key}_{i} 실패: {e}', file=sys.stderr, flush=True)
                if prev and (OUT / prev['file']).exists():
                    manifest[key].append(prev)
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=1))
    print(f'새로 만든 것 {made}개, 실패 {failed}개 → {OUT}')


if __name__ == '__main__':
    main()
