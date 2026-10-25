import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
const source=await fs.readFile(new URL('../../src/robot_platform/helm/static/pedals.js',import.meta.url),'utf8');
const {PedalControls,presentation,recordingLabel,recordingMode,StateOrder}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
const clone=x=>structuredClone(x),action=(name,revision=3)=>({action:name,label:name,enabled:true,reason:'',revision});
const base={teleop:{phase:'running',state:'TELEOP',status_at:20,observation_age_ms:10,pedals:{session_id:'one',revision:3,actions:{
  a:action('stop_record',9),b:action('pause'),c:action('shutdown')}}},tune:{phase:'stopped'}};
for(const [state,tone] of Object.entries({DISABLED:'off',IDLE:'off',HOLD:'hold',ENGAGING:'engaging',TELEOP:'teleop',SHUTDOWN:'shutdown',FAULT:'fault'})){
  const s=clone(base);s.teleop.state=state;assert.equal(presentation(s).tone,tone);
}
assert.equal(presentation({teleop:{phase:'stopped',state:'TELEOP'}}).tone,'off');
for(const fault of ['fault','recording_fault']){const s=clone(base);s.teleop[fault]='disk';assert.equal(presentation(s).tone,'fault');}
assert.equal(presentation(base,false).tone,'fault');
const ended=clone(base);ended.teleop.state='DISABLED';ended.teleop.pedals.ended=true;
ended.teleop.pedals.actions.c={enabled:false,reason:'Stop this session and start a new session.'};
assert.match(presentation(ended).detail,/Stop this session/);
assert.equal(presentation({...base,teleop:{...base.teleop,observation_age_ms:750}}).tone,'fault');
// Idle's three receiver exchanges leave no cycle time to observe, so joint observations go stale while the loop's
// status keeps arriving (2026-10-18: the C pedal locked in Idle). Pedals go by the status's age when Helm reports it.
const idleQuiet={...base,teleop:{...base.teleop,state:'IDLE',observation_age_ms:35000,status_age_ms:120}};
assert.equal(presentation(idleQuiet).label,'Idle');
assert.equal(presentation({...idleQuiet,teleop:{...idleQuiet.teleop,status_age_ms:750}}).label,'State unavailable');
assert.equal(presentation({...base,teleop:{...base.teleop,status_age_ms:10,observation_age_ms:20}}).tone,'teleop');
assert.match(recordingLabel({recording:{active:true},last_command:{state:'pending',action:'stop_record'}}),/Finishing/);
assert.equal(recordingLabel({recording:{state:'complete',complete:false}}),'Not recording');
assert.equal(recordingLabel({recording:{complete:true}}),'Complete · saved');
assert.equal(recordingLabel({recording:{complete:true,error:'disk'}}),'Recording incomplete');
// Helm reports task: null until a recording task is set up. The label threw on it, which stopped every render and
// the session button's handler before its request (2026-10-17): Start session did nothing.
assert.equal(recordingLabel(null),'Not recording');
assert.equal(recordingLabel(undefined),'Not recording');
let time=1000,ctx={tab:'teleop',hidden:false,confirming:false},requests=[],errors=[];
const controls=new PedalControls({now:()=>time,context:()=>ctx,onError:e=>errors.push(e),onState:s=>controls.update(s),
  request:(url,payload)=>new Promise((resolve,reject)=>requests.push({url,payload,resolve,reject}))});
const flush=()=>new Promise(resolve=>setImmediate(resolve));
const key=(name,extra={})=>({key:name,target:{closest:()=>null},preventDefault(){this.prevented=true;},...extra});
controls.update(clone(base));
const a=controls.press('a');assert.equal(requests[0].payload.revision,9);
assert.equal(controls.offer('a').enabled,false);
assert.equal(controls.offer('b').enabled,true);
const b=controls.press('b');assert.equal(requests[1].payload.revision,3);
assert.equal(controls.offer('c').enabled,false); // Motion waits for its own ACK, not recorder finalization.
requests[1].resolve(clone(base));await b;
assert.equal(controls.offer('c').enabled,true);
const c=controls.press('c');requests[2].resolve(clone(base));await c;
requests[0].resolve(clone(base));await a;
assert.deepEqual(requests.map(r=>r.payload.press_id),[1,2,3]);
requests=[];
controls.keydown(key('C'));assert.equal(requests.length,1);
controls.keydown(key('C'));controls.keydown(key('C',{repeat:true}));assert.equal(requests.length,1);
requests[0].resolve(clone(base));await flush();
controls.keydown(key('C'));assert.equal(requests.length,1);
controls.keyup(key('C'));controls.keydown(key('C'));assert.equal(requests.length,2);
requests[1].resolve(clone(base));await flush();controls.keyup(key('C'));requests=[];
for(const flag of ['ctrlKey','metaKey','altKey','shiftKey','isComposing','repeat']){
  controls.keydown(key('a',{[flag]:true}));controls.keyup(key('a'));
}
controls.keydown(key('a',{keyCode:229}));controls.keyup(key('a'));
controls.keydown(key('a',{target:{closest:()=>({})}}));controls.keyup(key('a'));
assert.equal(requests.length,0);
for(const patch of [{tab:'settings'},{hidden:true},{confirming:true}]){
 ctx={tab:'teleop',hidden:false,confirming:false,...patch};controls.keydown(key('b'));controls.keyup(key('b'));
}
assert.equal(requests.length,0);ctx={tab:'teleop',hidden:false,confirming:false};
time+=740;assert.equal(controls.offer('c').enabled,false); // Includes original worker age.
controls.update(clone(base),time-750);assert.equal(controls.offer('c').enabled,false); // Slow request does not renew freshness.
controls.update(clone(base));controls.disconnect();assert.equal(controls.offer('c').enabled,false);
controls.update(clone(base));const rejected=controls.press('a');requests.at(-1).reject(Error('unknown outcome'));await rejected;
assert.equal(controls.offer('a').enabled,false);assert.equal(errors.at(-1),'unknown outcome');
controls.update(clone(base));assert.equal(controls.offer('a').enabled,true);
requests=[];
const old=controls.press('a');const newer=clone(base);newer.teleop.pedals.session_id='two';newer.teleop.state='HOLD';controls.update(newer);
requests[0].resolve(base);await old;assert.equal(controls.state.teleop.pedals.session_id,'two');
// The stopped tester (Settings -> Pedals) consumes input without ever making a request; switching to start clears it.
requests=[];const stopped=clone(base);stopped.teleop.phase='stopped';controls.update(stopped);controls.setTest(true);
ctx={...ctx,pedalTest:true};
for(const pedal of ['a','b','c']){controls.keydown(key(pedal));controls.keydown(key(pedal));controls.keyup(key(pedal));}
assert.deepEqual(controls.testCounts,{a:1,b:1,c:1});assert.equal(requests.length,0);assert.equal(controls.testLast,'c');
// Off its page the tester counts nothing.
ctx={...ctx,pedalTest:false};controls.keydown(key('a'));controls.keyup(key('a'));
assert.deepEqual(controls.testCounts,{a:1,b:1,c:1});assert.equal(requests.length,0);ctx={...ctx,pedalTest:true};
controls.setTest(false);assert.deepEqual(controls.testCounts,{a:0,b:0,c:0});
controls.setTest(true);controls.update({...stopped,teleop:{...stopped.teleop,phase:'starting'}});
assert.equal(controls.testMode,false);assert.deepEqual(controls.testCounts,{a:0,b:0,c:0});
controls.update({...stopped,teleop:{...stopped.teleop,fault:'reset'}});controls.setTest(true);assert.equal(controls.testMode,false);
// In Idle the pedal offers stay usable while status arrives, however old the joint observations are.
const idle=clone(base);idle.teleop.state='IDLE';idle.teleop.observation_age_ms=35000;idle.teleop.status_age_ms=120;
idle.teleop.pedals.session_id='idle';idle.teleop.pedals.actions.c=action('enable',4);
controls.update(idle);assert.equal(controls.fresh(),true);assert.equal(controls.offer('c').enabled,true);
time+=640;assert.equal(controls.offer('c').enabled,false); // The status's age still counts, plus the time since.
controls.update({...idle,teleop:{...idle.teleop,status_age_ms:760}});assert.equal(controls.offer('c').enabled,false);
const order=new StateOrder();assert.equal(order.accept(base,1),true);
const fault=clone(base);fault.teleop.state='FAULT';fault.teleop.status_at=21;assert.equal(order.accept(fault,3),true);
assert.equal(order.accept(base,2),false); // Old response arrives after new poll.
assert.equal(order.accept(base,4),false); // Later request, but cached older worker status.
const sameStamp=clone(base);sameStamp.teleop.status_at=21;
assert.equal(order.accept(sameStamp,4),false); // Parent-side fault can retain the old worker clock.
assert.equal(order.accept(fault,3),false); // Local re-render cannot refresh age.
const restarted=clone(base);restarted.teleop.pedals.session_id='new';restarted.teleop.status_at=1;assert.equal(order.accept(restarted,5),true);
assert.equal(order.accept(fault,4),false);
// The recording indicator's mode follows the same facts as the label.
assert.equal(recordingMode(null),'off');
assert.equal(recordingMode({last_command:{state:'pending',action:'start_record'}}),'starting');
assert.equal(recordingMode({recording:{state:'starting'}}),'starting');
assert.equal(recordingMode({recording:{active:true,state:'recording'}}),'recording');
assert.equal(recordingMode({recording:{active:true},last_command:{state:'pending',action:'stop_record'}}),'finishing');
assert.equal(recordingMode({recording:{state:'finishing'}}),'finishing');
assert.equal(recordingMode({recording:{complete:true,state:'idle'}}),'saved');
assert.equal(recordingMode({recording:{state:'failed',error:'camera worker exited'}}),'failed');
assert.equal(recordingMode({error:'task failed',recording:{active:true}}),'failed');
console.log('Pedal offers, independent safety input, held/editable keys, stale/order guards, true state and stopped tester passed.');
