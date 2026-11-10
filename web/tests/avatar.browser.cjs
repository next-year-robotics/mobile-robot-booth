const { chromium } = require('playwright');
const assert=require('node:assert/strict');
const port = Number(process.env.BOOTH_QA_PORT || 8087);
if (!Number.isInteger(port) || port < 1024 || port > 65535) throw new Error('Invalid BOOTH_QA_PORT');
const base = `http://127.0.0.1:${port}`;
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:false,args:['--no-sandbox','--autoplay-policy=no-user-gesture-required']});
 try {
 const page=await browser.newPage({viewport:{width:1920,height:1200}});const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.addInitScript(()=>{
  window.qa={decoded:0,states:0,maxGap:0,last:0,sent:[],sockets:[]};
  const decode=window.createImageBitmap;window.createImageBitmap=(...a)=>{window.qa.decoded++;return decode(...a)};
  const WS=window.WebSocket;window.WebSocket=new Proxy(WS,{construct(T,args){const ws=new T(...args);window.qa.sockets.push(ws);const send=ws.send.bind(ws);ws.send=d=>{window.qa.sent.push(d);send(d)};ws.addEventListener('message',e=>{if(typeof e.data==='string'&&JSON.parse(e.data).type==='state'){const t=performance.now();if(window.qa.last)window.qa.maxGap=Math.max(window.qa.maxGap,t-window.qa.last);window.qa.last=t;window.qa.states++}});return ws}});
 });
 await page.goto(base + '/');await page.waitForTimeout(2500);
 assert.ok(await page.evaluate(()=>window.qa.decoded>5));
 await page.keyboard.press('2');await page.waitForFunction(()=>document.querySelector('#avatar-badge')?.textContent==='대기');
 await page.waitForTimeout(1000);const decoded=await page.evaluate(()=>window.qa.decoded);
 await page.waitForTimeout(2000);assert.equal(await page.evaluate(()=>window.qa.decoded),decoded);
 const status = await page.locator('#avatar-robot').textContent();
 assert.match(status, /시뮬레이션/, 'Refusing control test outside simulator');
 console.log('sim',status);
 await page.keyboard.press('Space');assert.ok(await page.evaluate(()=>window.qa.sent.some(s=>s.includes('op:stop'))));
 await page.locator('#avatar-intro').click();await page.waitForTimeout(5000);
 await page.screenshot({path:'/tmp/booth-avatar-preview.png'});
 await page.waitForFunction(()=>document.querySelector('#avatar-intro')?.textContent==='자기소개 듣기',null,{timeout:45000});
 console.log('full introduction finished');
 for(let i=0;i<8;i++){await page.keyboard.press('1');await page.waitForTimeout(350);await page.keyboard.press('2');await page.waitForTimeout(350)}
 assert.equal(await page.locator('#avatar-stage canvas').count(),1);
 await page.waitForTimeout(2200);
 console.log('metrics',await page.locator('#avatar-stats').textContent(),await page.evaluate(()=>({states:window.qa.states,maxStateGapMs:window.qa.maxGap,heapMB:performance.memory.usedJSHeapSize/1048576})));
 await page.evaluate(()=>window.qa.sockets.at(-1).close());await page.waitForTimeout(100);
 assert.ok(await page.locator('#offline').isVisible());await page.waitForTimeout(1500);
 const missing=await browser.newPage();await missing.route('**/avatar/platinum.vrm',r=>r.fulfill({status:404,body:'not found'}));
 await missing.goto(base + '/#avatar');await missing.waitForFunction(()=>document.querySelector('#avatar-badge')?.textContent==='모델 로드 실패');
 await missing.keyboard.press('1');assert.equal(await missing.locator('#app').evaluate(e=>getComputedStyle(e).visibility),'visible');
 console.log('missing model fallback, offline overlay, Space stop, 8 tab cycles passed');assert.deepEqual(errors,[]);
 } finally { await browser.close(); }
})().catch(e=>{console.error(e);process.exit(1)});
