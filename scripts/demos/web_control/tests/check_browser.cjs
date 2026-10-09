// Opt-in browser integration check. Requires Playwright and a running demo.
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const base=process.env.WEB_CONTROL_URL || 'http://127.0.0.1:8080';
const artifacts=process.env.WEB_CONTROL_ARTIFACTS || '/tmp/isaac-web-browser-check';
fs.mkdirSync(artifacts,{recursive:true});
(async()=>{
 const browser=await chromium.launch({...(process.env.BROWSER_EXECUTABLE ? {executablePath:process.env.BROWSER_EXECUTABLE} : {}),headless:true,args:['--no-sandbox','--disable-gpu']});
 try {
  const context=await browser.newContext({viewport:{width:1440,height:1100},acceptDownloads:true});
  const page=await context.newPage(), errors=[];
  page.on('pageerror', e=>errors.push(e.message));
  await page.request.post(base+'/api/command',{data:{action:'resume'}});
  await page.goto(base);
  await page.waitForFunction(()=>document.querySelectorAll('.image-wrap.ready:not(.stale)').length===3);
  assert.equal(await page.locator('#joints input').count(),7);
  await page.waitForFunction(()=>document.querySelector('#mode').textContent==='运行中');
  await page.locator('#pause').click();
  await page.waitForFunction(()=>document.querySelector('#mode').textContent==='已暂停');
  const timeBefore=await page.locator('#sim-time').textContent();
  await page.waitForTimeout(500);
  assert.equal(await page.locator('#sim-time').textContent(),timeBefore);
  await page.locator('#step').click();
  await page.waitForTimeout(400);
  await page.locator('#reset').click();
  await page.waitForFunction(()=>document.querySelector('#sim-time').textContent==='0.00 s');
  await page.locator('#joint-0').focus();
  await page.request.post(base+'/api/command',{data:{action:'joint',joint:0,value:.4}});
  await page.waitForFunction(()=>Math.abs(Number(document.querySelector('#joint-0').value)-.4)<.001);
  await page.locator('#joints .joint').first().getByRole('button',{name:'panda_joint1 增大目标'}).click();
  await page.waitForTimeout(300);
  const state=await (await page.request.get(base+'/api/status')).json();
  assert.ok(Math.abs(state.joints[0].target-.45)<.001);
  const downloadPromise=page.waitForEvent('download');
  await page.locator('#snapshot').click();
  const download=await downloadPromise;
  await download.saveAs(path.join(artifacts,'ui-snapshot.zip'));
  await page.screenshot({path:path.join(artifacts,'desktop.png'),fullPage:true});
  // Camera requests fail while the state connection remains healthy.
  await page.route('**/api/camera/**',route=>route.abort());
  await page.waitForFunction(()=>document.querySelectorAll('.image-wrap.stale').length===3);
  assert.match(await page.locator('#connection').textContent(),/已连接/);
  await page.unroute('**/api/camera/**');
  await page.waitForFunction(()=>document.querySelectorAll('.image-wrap.ready:not(.stale)').length===3);
  // State connection failure disables all controls and recovers.
  await page.route('**/api/status',route=>route.abort());
  await page.waitForFunction(()=>document.querySelector('#pause').disabled);
  await page.unroute('**/api/status');
  await page.waitForFunction(()=>!document.querySelector('#pause').disabled);
  // A queued command from before a disconnect must not execute after recovery.
  const initialTarget=(await (await page.request.get(base+'/api/status')).json()).joints[0].target;
  let releaseCommand, firstSeen;
  const gate=new Promise(resolve=>releaseCommand=resolve);
  const intercepted=new Promise(resolve=>firstSeen=resolve);
  let first=true;
  await page.route('**/api/command',async route=>{
    if(first){first=false;firstSeen();await gate;}
    await route.continue();
  });
  const plus=page.getByRole('button',{name:'panda_joint1 增大目标'});
  await plus.click();
  await intercepted;
  await page.locator('#reset').click();
  await page.route('**/api/status',route=>route.abort());
  await page.waitForFunction(()=>document.querySelector('#pause').disabled);
  await page.unroute('**/api/status');
  await page.waitForFunction(()=>!document.querySelector('#pause').disabled);
  releaseCommand();
  await page.waitForTimeout(800);
  const recovered=(await (await page.request.get(base+'/api/status')).json()).joints[0].target;
  assert.ok(Math.abs(recovered-(initialTarget+.05))<.001, 'old queued command survived reconnect');
  await page.unroute('**/api/command');
  await page.setViewportSize({width:390,height:844});
  await page.screenshot({path:path.join(artifacts,'mobile.png'),fullPage:true});
  assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth));
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'PASS',checks:['images','7 joints','pause','step','reset','cross-page target','increment','download','camera failure/recovery','status failure/recovery','queued command invalidation','mobile width'],pageErrors:errors},null,2));
 } finally {await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
