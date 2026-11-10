const { chromium } = require('playwright');
const assert = require('node:assert/strict');
(async () => {
  const port = Number(process.env.BOOTH_QA_PORT || 8087);
  assert.ok(Number.isInteger(port) && port >= 1024 && port <= 65535);
  const browser = await chromium.launch({ executablePath: '/usr/bin/google-chrome', headless: process.env.BOOTH_QA_HEADLESS === '1',
    args: ['--no-sandbox', '--ozone-platform=x11', '--force-device-scale-factor=1', '--autoplay-policy=no-user-gesture-required', '--mute-audio'] });
  try {
    const page = await browser.newPage({ viewport: { width: 1920, height: 1200 } });
    const errors = []; page.on('pageerror', e => errors.push(e.message));
    await page.addInitScript(() => {
      const Original = window.Audio;
      window.Audio = new Proxy(Original, { construct(T, args) {
        const audio = new T(...args);
        if (String(args[0]).includes('introduction')) window.introQA = audio;
        return audio;
      } });
    });
    await page.goto(`http://127.0.0.1:${port}/#avatar`);
    await page.waitForFunction(() => document.querySelector('#avatar-badge')?.textContent === '대기');
    assert.equal(await page.locator('.avatar-title').count(), 0);
    assert.equal(await page.locator('#avatar-name-card').getAttribute('aria-hidden'), 'true');
    await page.locator('#avatar-intro').click();
    for (const time of [2.5, 7.4, 29, 42.5]) {
      await page.waitForFunction(t => window.introQA.currentTime >= t, time, { timeout: 45000 });
      if (time === 2.5) assert.equal(await page.locator('.name-word.revealed').count(), 0);
      if (time === 7.4) {
        assert.equal(await page.locator('.name-word.revealed').count(), 7);
        assert.equal(await page.locator('#avatar-name-card').getAttribute('aria-hidden'), 'false');
      }
      if (time === 42.5) assert.ok(Number(await page.locator('#avatar-name-card').evaluate(e => e.style.opacity)) < 0.4);
      await page.screenshot({ path: `/tmp/maknae-intro-${time}.png` });
    }
    await page.waitForFunction(() => window.introQA.ended, null, { timeout: 5000 });
    await page.waitForTimeout(500);
    assert.equal(await page.locator('#avatar-name-card').getAttribute('aria-hidden'), 'true');
    assert.equal(await page.locator('#avatar-intro').textContent(), '자기소개 듣기');
    await page.locator('#avatar-intro').click();
    await page.waitForTimeout(5000);
    assert.ok(await page.locator('.name-word.revealed').count() >= 2);
    await page.locator('#avatar-intro').click();
    assert.equal(await page.locator('#avatar-name-card').getAttribute('aria-hidden'), 'true');
    await page.locator('#avatar-intro').click();
    await page.waitForTimeout(1000);
    assert.equal(await page.locator('.name-word.revealed').count(), 0);
    await page.keyboard.press('1');
    assert.ok(await page.evaluate(() => window.introQA.paused));
    assert.deepEqual(errors, []);
    console.log('Introduction: real audio clock, word reveals, end fade, stop/replay, tab cancellation passed.');
  } finally { await browser.close(); }
})().catch(e => { console.error(e); process.exit(1); });
