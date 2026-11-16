// 실제 Gemini Live와 음성 왕복을 본다. 마이크 대신 녹음 파일을 넣는다(Chrome 가짜 캡처).
//   VOICE_WAV=/path/ask.wav node tests/voice.browser.cjs            # 떠 있는 대시보드(8080)에 붙는다
//   HOLD_S=5 WAIT_S=20 VIEW=avatar|dashboard|errand SHOT=/path/out.png CHAT=hina|gemini(대화 목소리)
// 인터넷과 API 키가 필요하고 로봇 도구가 실제로 불린다. 자동 시험 묶음에는 넣지 않는다.
const { chromium } = require('playwright');
const wav = process.env.VOICE_WAV;
if (!wav) throw new Error('VOICE_WAV가 필요하다');
const port = Number(process.env.BOOTH_QA_PORT || 8080);
const hold = Number(process.env.HOLD_S || 5), wait = Number(process.env.WAIT_S || 18);
(async () => {
  const browser = await chromium.launch({ executablePath: '/usr/bin/google-chrome', headless: process.env.HEADFUL !== '1',
    args: ['--no-sandbox', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream',
      '--use-fake-device-for-media-stream', `--use-file-for-fake-audio-capture=${wav}%noloop`] });
  const page = await browser.newPage({ viewport: { width: 1920, height: 1200 } });
  const errors = []; page.on('pageerror', e => errors.push(e.message));
  page.on('response', r => { if (r.status() >= 400) errors.push(r.status() + ' ' + r.url()); });
  if (process.env.CHAT) await page.addInitScript(v => { localStorage.setItem('maknae.settings', JSON.stringify({ chatVoice: v })); }, process.env.CHAT);
  await page.addInitScript(() => {
    window.qa = { live: [], tools: [], t0: 0, audio: 0 };
    const WS = window.WebSocket;
    window.WebSocket = new Proxy(WS, { construct(T, args) {
      const ws = new T(...args);
      if (String(args[0]).includes('generativelanguage')) {
        ws.addEventListener('message', async e => {
          const text = typeof e.data === 'string' ? e.data : new TextDecoder().decode(e.data);
          const m = JSON.parse(text), t = Math.round(performance.now() - window.qa.t0);
          if (m.setupComplete) window.qa.live.push([t, 'setupComplete']);
          if (m.toolCall) window.qa.tools.push([t, m.toolCall.functionCalls.map(c => `${c.name}(${JSON.stringify(c.args)})`).join(', ')]);
          if (m.serverContent?.modelTurn?.parts?.some(p => p.inlineData) && !window.qa.audio) { window.qa.audio = t; window.qa.live.push([t, 'first audio']); }
          if (m.serverContent?.turnComplete) { window.qa.live.push([t, 'turnComplete']); window.qa.audio = 0; }
        });
        const send = ws.send.bind(ws);
        ws.send = d => { const m = JSON.parse(d); if (m.toolResponse) window.qa.tools.push([Math.round(performance.now() - window.qa.t0), 'response ' + JSON.stringify(m.toolResponse.functionResponses.map(r => r.response))]); send(d); };
      }
      return ws;
    } });
  });
  await page.goto(`http://127.0.0.1:${port}/#${process.env.VIEW || 'avatar'}`);
  const chip = () => page.evaluate(() => [document.querySelector('#voice-chip')?.textContent, document.querySelector('#voice-text')?.textContent]);
  await page.waitForTimeout(4000);
  console.log('열린 뒤', await chip(), await page.evaluate(() => window.qa.live));
  await page.evaluate(() => { window.qa.t0 = performance.now(); window.qa.live = []; });
  await page.keyboard.down('KeyV');
  const seen = new Set();
  const poll = async (ms) => { const end = Date.now() + ms; while (Date.now() < end) { const c = (await chip()).join(' | '); if (!seen.has(c)) { seen.add(c); console.log(`  ${((Date.now() - start) / 1000).toFixed(1)}s`, c); } await page.waitForTimeout(150); } };
  const start = Date.now();
  await poll(hold * 1000);
  await page.keyboard.up('KeyV');
  await page.evaluate(() => { window.qa.released = Math.round(performance.now() - window.qa.t0); });
  await poll(wait * 1000);
  const qa = await page.evaluate(() => window.qa);
  console.log('뗀 시각 ms', qa.released, '\nLive', JSON.stringify(qa.live), '\n도구', JSON.stringify(qa.tools, null, 1));
  if (process.env.SHOT) await page.screenshot({ path: process.env.SHOT });
  console.log('오류', errors.slice(0, 5));
  await browser.close();
})().catch(e => { console.error(e); process.exit(1); });
