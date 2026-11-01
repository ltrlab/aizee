// Offline real-browser contract. Supply PLAYWRIGHT_MODULE and optionally BROWSER_EXECUTABLE.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import {createRequire} from 'node:module';
const require = createRequire(import.meta.url);
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.resolve(path.dirname(new URL(import.meta.url).pathname.replace(/^\/([A-Z]:)/i, '$1')), '../..');
const evidence = path.join(root, 'build/pedal-ui'); await fs.mkdir(evidence, {recursive:true});
const state = {mode:'robot',action_token:'offline-fixture',robot:{model:'Minerva',asset_id:'fixture',address:'offline',connection:'tether',connections:[]},
  feed:{state:'disconnected'},limits:{rows:[],path:'fixture/limits.json',pending:0},session:{},link:{running:false},
  teleop:{phase:'stopped',state:'DISABLED',joints:[],target:'jetson',receiver_choice:{mode:'virtual'}},telemetry:{rows:[]}};
const writes=[], reads=[], errors=[], pedalRequests=[];
let delayA=false, rejectState=false;
const offers=()=>{
 const t=state.teleop,p=t.pedals;
 if(!p)return;
 const a=t.task?.recording?.active?'stop_record':'start_record';
 p.actions={a:{action:a,label:a==='stop_record'?'Finish recording':'Start recording',enabled:!t.task?.last_command || t.task.last_command.state!=='pending',revision:20},
 b:{action:['TELEOP','ENGAGING'].includes(t.state)?'pause':'resume',label:['TELEOP','ENGAGING'].includes(t.state)?'Pause':'Resume',enabled:['TELEOP','ENGAGING'].includes(t.state)||(t.state==='HOLD'&&p.paused),revision:p.revision},
 c:{action:{DISABLED:'idle',IDLE:'enable',HOLD:p.paused?'shutdown':'engage',TELEOP:'shutdown'}[t.state]||'none',
 label:{DISABLED:'Idle',IDLE:'Enable',HOLD:p.paused?'Shutdown':'Engage',TELEOP:'Shutdown'}[t.state]||'Wait',
 enabled:['DISABLED','IDLE','HOLD','TELEOP'].includes(t.state),revision:p.revision,
 warning:t.task?.recording?.active?'C aborts unfinished recording. A then Complete saves it.':''}};
};
const setState=(value,extra={})=>{
 Object.assign(state.teleop,{state:value,status_at:(state.teleop.status_at||0)+1,observation_age_ms:1},extra);
 if(state.teleop.pedals)state.teleop.pedals.revision++;
 offers();
};
const server=http.createServer(async(req,res)=>{
  try {
    if(req.method!=='GET') writes.push(req.url);
    reads.push(req.url);
    if(req.url==='/api/helm/teleop/start'){
      state.teleop.phase='starting';setState('DISABLED');res.setHeader('Content-Type','application/json');res.end(JSON.stringify(state));return;
    }
    if(req.url==='/api/helm/teleop/pedal'){
      let raw='';for await(const chunk of req)raw+=chunk;
      const message=JSON.parse(raw);pedalRequests.push(message);
      const t=state.teleop;
      if(message.pedal==='a'){
        t.task={recording:{active:true},last_command:{action:'stop_record',state:'pending'}};
        t.status_at++;offers();
        const old=JSON.stringify(state);
        if(delayA){setTimeout(()=>{res.setHeader('Content-Type','application/json');res.end(old);},1000);return;}
      } else if(message.pedal==='b'){
        t.pedals.paused=['TELEOP','ENGAGING'].includes(t.state);setState(t.pedals.paused?'HOLD':'ENGAGING');
      } else setState({DISABLED:'IDLE',IDLE:'HOLD',HOLD:t.pedals.paused?'SHUTDOWN':'ENGAGING',TELEOP:'SHUTDOWN'}[t.state]||t.state);
      res.setHeader('Content-Type','application/json');res.end(JSON.stringify(state));return;
    }
    if(req.url==='/api/helm/teleop/key'){let raw='';for await(const chunk of req)raw+=chunk;writes.push(JSON.parse(raw).key);res.setHeader('Content-Type','application/json');res.end(JSON.stringify(state));return;}
    if(req.url==='/api/helm/state' && rejectState){res.writeHead(503);res.end('{}');return;}
    if(req.url==='/api/helm/state'){res.setHeader('Content-Type','application/json');res.end(JSON.stringify(state));return;}
    if(req.url==='/api/helm/settings'){res.setHeader('Content-Type','application/json');res.end(JSON.stringify({sections:[],revision:1,session_id:'fixture',can_apply:false}));return;}
    if(req.url==='/api/helm/cameras'){res.setHeader('Content-Type','application/json');res.end(JSON.stringify({can_apply:true,cameras:{right_wrist:{available:true,controls:{gain:{value:30,min:0,max:100,step:1}}}}}));return;}
    if(req.url.startsWith('/viewer')){res.end('<body style="background:#15191e;color:white">Offline model fixture</body>');return;}
    const name=req.url==='/'?'index.html':req.url.replace(/^\/helm\//,'');
    if(!/^[a-z_.-]+$/.test(name)){res.writeHead(404);res.end();return;}
    res.setHeader('Content-Type',name.endsWith('.js')?'text/javascript':name.endsWith('.css')?'text/css':'text/html');
    res.end(await fs.readFile(path.join(root,'src/robot_platform/helm/static',name)));
  } catch {res.writeHead(404);res.end();}
}).listen(0,'127.0.0.1');
await new Promise(resolve=>server.once('listening',resolve));
const browser=await chromium.launch({headless:true,...(process.env.BROWSER_EXECUTABLE?{executablePath:process.env.BROWSER_EXECUTABLE}:{})});
const report={fixture:'Actual Helm modules in Chromium; mocked lifecycle/recording API and camera pixels; no hardware or native motion qualification',checks:[]};
try {
  const context=await browser.newContext({viewport:{width:800,height:480},hasTouch:true});
  await context.addInitScript(()=>{
    localStorage.setItem('helm-touch-layout','on');localStorage.setItem('helm-tab','teleop');
    window.fixtureSockets=[];window.fixtureUrls=new Set();window.fixtureCreated=0;window.fixtureHidden=false;
    Object.defineProperty(document,'hidden',{get:()=>window.fixtureHidden});
    const create=URL.createObjectURL.bind(URL),revoke=URL.revokeObjectURL.bind(URL);
    URL.createObjectURL=blob=>{const url=create(blob);fixtureUrls.add(url);fixtureCreated++;return url;};
    URL.revokeObjectURL=url=>{fixtureUrls.delete(url);revoke(url);};
    window.WebSocket=class {
      static OPEN=1;
      constructor(url){this.url=String(url);this.readyState=0;this.messages=[];fixtureSockets.push(this);setTimeout(()=>{if(this.readyState===0){this.readyState=1;this.onopen?.();}},0);}
      send(text){const message=JSON.parse(text);if(message.type!=='heartbeat')throw Error('Unexpected write');this.messages.push(message);this.heartbeat=message.sequence;}
      close(){this.readyState=3;setTimeout(()=>this.onclose?.(),0);}
      emit(data){if(this.readyState===1)this.onmessage?.({data});}
    };
    const decode=HTMLImageElement.prototype.decode;
    HTMLImageElement.prototype.decode=function(){const delay=window.fixtureDecodeDelay||0;return new Promise(resolve=>setTimeout(resolve,delay)).then(()=>decode.call(this));};
    window.fixtureFrame=(role,sequence,age_s=.02,heartbeat)=>{
      const socket=fixtureSockets.at(-1),canvas=document.createElement('canvas');canvas.width=640;canvas.height=480;
      const ctx=canvas.getContext('2d');ctx.fillStyle={left_wrist:'#168b90',right_wrist:'#cb8523',head:'#375eb4'}[role];ctx.fillRect(0,0,640,480);ctx.fillStyle='white';ctx.font='36px sans-serif';ctx.fillText(role,24,80);
      const jpeg=Uint8Array.from(atob(canvas.toDataURL('image/jpeg').split(',')[1]),c=>c.charCodeAt(0));
      const header=new TextEncoder().encode(JSON.stringify({role,sequence,age_s,source_timestamp:1234,viewer_heartbeat:heartbeat??socket.heartbeat}));
      const packet=new Uint8Array(4+header.length+jpeg.length);new DataView(packet.buffer).setUint32(0,header.length);packet.set(header,4);packet.set(jpeg,4+header.length);socket.emit(packet.buffer);
    };
    window.fixtureStatus=(fresh=true)=>{const s=fixtureSockets.at(-1);s.emit(JSON.stringify({type:'status',stale_after_s:1,target_fps:10,viewer_heartbeat:s.heartbeat,cameras:Object.fromEntries(['left_wrist','right_wrist','head'].map(role=>[role,{fresh,age_s:.02,sequence:1,error:fresh?null:'Camera disconnected'}]))}));};
  });
  const page=await context.newPage();page.on('pageerror',e=>errors.push(String(e)));

  await page.goto('http://127.0.0.1:'+server.address().port+'/');
  await page.waitForFunction(()=>document.getElementById('pedal-test').hidden===false);
  await page.locator('#pedal-test').tap();
  for(const key of ['a','b','c'])await page.keyboard.press(key);
  assert.equal(pedalRequests.length,0);
  assert.match(await page.locator('#pedal-test-status').textContent(),/Test only.*Last: C/);
  for(const key of ['a','b','c'])assert.equal(await page.locator('[data-pedal='+key+'] small').textContent(),'1');
  for(const [width,height] of [[800,480],[1024,600]]){
    await page.setViewportSize({width,height});
    const startBox=await page.locator('#teleop-session').boundingBox();
    assert(startBox&&startBox.height>=48&&startBox.y+startBox.height<=height);
    await page.screenshot({path:path.join(evidence,'stopped-test-'+width+'x'+height+'.png')});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
  }
  report.checks.push('Stopped tester counts all three pedals without any API mutation');
  state.teleop.pedals={session_id:'browser-fixture',revision:1,paused:false};
  await page.locator('#teleop-session').tap();assert(await page.locator('#pedal-test-status').isHidden());
  await page.waitForFunction(()=>document.getElementById('pedal-test').hidden);
  assert(await page.locator('#pedal-test-status').isHidden());
  await page.keyboard.press('c');assert.equal(pedalRequests.length,0);
  state.teleop.phase='running';setState('DISABLED');await page.waitForFunction(()=>!document.querySelector('[data-pedal=c]').disabled);
  await page.keyboard.down('c');await page.waitForFunction(()=>document.getElementById('pedal-phase-label').textContent==='Idle');
  await page.keyboard.down('c');assert.equal(pedalRequests.length,1);await page.keyboard.up('c');
  await page.keyboard.press('c');await page.waitForFunction(()=>document.body.dataset.touchPhase==='hold');
  await page.keyboard.press('c');await page.waitForFunction(()=>document.body.dataset.touchPhase==='engaging');
  assert.match(await page.locator('#pedal-phase-label').textContent(),/aligning/);
  assert(await page.locator('[data-pedal=c]').isDisabled());
  await page.waitForTimeout(350);assert.equal(await page.locator('body').getAttribute('data-touch-phase'),'engaging');
  report.checks.push('C follows actual Idle, Hold and Engaging ACK; held key is suppressed; no invented alignment completion');
  setState('TELEOP');await page.waitForFunction(()=>document.body.dataset.touchPhase==='teleop');
  assert.notEqual(await page.evaluate(()=>getComputedStyle(document.body,'::after').boxShadow),'none');
  for(const [width,height] of [[800,480],[1024,600]]){
    await page.setViewportSize({width,height});
    for(const id of ['global-hold','global-shutdown','global-disable','teleop-session']){
      const box=await page.locator('#'+id).boundingBox();assert(box&&box.height>=48&&box.x>=0&&box.x+box.width<=width&&box.y+box.height<=height);
    }
    for(const pedal of ['a','b','c']){
      const box=await page.locator('[data-pedal='+pedal+']').boundingBox();assert(box&&box.height>=48&&box.y+box.height<=height);
    }
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    await page.screenshot({path:path.join(evidence,'teleop-'+width+'x'+height+'.png')});
  }
  report.checks.push('800x480 and1024x600 rendered touch layouts: phase, pedals and existing H/X controls visible');
  await page.locator('.tab[data-tab=settings]').tap();const before=pedalRequests.length;await page.keyboard.press('b');assert.equal(pedalRequests.length,before);
  assert.equal(await page.locator('body').getAttribute('data-touch-phase'),'teleop');
  await page.locator('.tab[data-tab=teleop]').tap();
  await page.evaluate(()=>{const i=document.createElement('input');i.id='pedal-edit-fixture';document.body.append(i);i.focus();});
  await page.keyboard.press('a');await page.evaluate(()=>document.getElementById('pedal-edit-fixture').remove());
  await page.keyboard.press('Shift+A');assert.equal(pedalRequests.length,before);
  report.checks.push('Pedals ignore Settings, editable targets and modifiers; actual touch border persists across tabs');
  state.teleop.task={recording:{active:true}};setState('TELEOP');delayA=true;
  await page.waitForFunction(()=>document.querySelector('[data-pedal=a] span').textContent==='Finish recording');
  await page.keyboard.press('a');await page.waitForFunction(()=>document.querySelector('[data-pedal=a]').disabled);
  await page.keyboard.press('b');await page.waitForFunction(()=>document.body.dataset.touchPhase==='hold');
  assert.match(await page.locator('#pedal-phase-detail').textContent(),/motors stay enabled/);
  await page.keyboard.press('c');await page.waitForFunction(()=>document.body.dataset.touchPhase==='shutdown');
  await page.keyboard.press('h');await page.keyboard.press('x');assert(writes.includes('H')&&writes.includes('X'));
  setState('FAULT',{fault:'fixture fault'});
  await page.waitForFunction(()=>document.body.dataset.touchPhase==='fault');await page.waitForTimeout(1200);
  assert.equal(await page.locator('body').getAttribute('data-touch-phase'),'fault');
  assert(!/saved/.test(await page.locator('#pedal-recording').textContent()));
  report.checks.push('A pending does not block B, C, H or X; late old action response cannot restore green after a newer fault');
  state.teleop.pedals.session_id='browser-after-fault';
  for(const [stateName,tone] of [['DISABLED','off'],['IDLE','off'],['HOLD','hold'],['ENGAGING','engaging'],['TELEOP','teleop'],['SHUTDOWN','shutdown']]){
    setState(stateName,{fault:null});await page.waitForFunction(t=>document.body.dataset.touchPhase===t,tone);
    if(tone==='off')assert.match(await page.evaluate(()=>getComputedStyle(document.body,'::after').boxShadow),/rgba\(0, 0, 0, 0\)/);
  }
  setState('ENGAGING');await page.waitForFunction(()=>document.body.dataset.touchPhase==='engaging');
  assert.equal(await page.evaluate(()=>getComputedStyle(document.body,'::after').animationName),'touch-state-pulse');
  await page.emulateMedia({reducedMotion:'reduce'});assert.equal(await page.evaluate(()=>getComputedStyle(document.body,'::after').animationName),'none');
  await page.locator('header [data-touch-toggle]').tap();assert.equal(await page.evaluate(()=>getComputedStyle(document.body,'::after').content),'none');
  await page.locator('header [data-touch-toggle]').tap();
  state.teleop.phase='stopped';state.teleop.receiver_choice={mode:'live',arm:'both'};setState('DISABLED');
  await page.waitForFunction(()=>document.getElementById('teleop-session').textContent==='Start LIVE session');
  const startPosts=writes.filter(x=>x==='/api/helm/teleop/start').length;
  await page.locator('#pedal-test').tap();await page.locator('#teleop-session').tap();
  assert(await page.locator('#t-confirm').isVisible());assert(await page.locator('#pedal-test-status').isHidden());
  assert.equal(writes.filter(x=>x==='/api/helm/teleop/start').length,startPosts);
  await page.locator('#t-confirm-cancel').tap();
  state.teleop.phase='running';state.teleop.live={arm:'both'};setState('HOLD');
  await page.waitForFunction(()=>document.getElementById('teleop-session').textContent==='Stop session');
  await page.locator('#teleop-session').tap();assert(await page.locator('#t-confirm').isVisible());
  assert.equal(writes.filter(x=>x==='/api/helm/teleop/stop').length,0);await page.locator('#t-confirm-cancel').tap();
  report.checks.push('Moved Start/Stop remains immediately visible and preserves both live start and powered stop confirmations');
  setState('TELEOP');await page.waitForFunction(()=>document.body.dataset.touchPhase==='teleop');rejectState=true;
  await page.waitForFunction(()=>document.body.dataset.touchPhase==='fault');assert(await page.locator('[data-pedal=c]').isDisabled());
  assert.match(await page.locator('#pedal-phase-label').textContent(),/unavailable/);
  assert.match(await page.locator('#t-state').textContent(),/unavailable/);
  assert.equal(await page.locator('body').getAttribute('data-teleop-state'),'STALE');
  assert.equal(await page.locator('#t-state').evaluate(el=>getComputedStyle(el).color),'rgb(255, 139, 152)');
  report.checks.push('All requested border colors/stages, touch toggle, reduced motion, and stale-server red verified');
  assert.deepEqual(errors,[]);report.passed=true;
} finally {await browser.close();await new Promise(resolve=>server.close(resolve));await fs.writeFile(path.join(evidence,'report.json'),JSON.stringify(report,null,2));}
console.log(JSON.stringify(report));
