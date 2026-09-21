import test from 'node:test';
import assert from 'node:assert/strict';
import {PoseCollisionChecker} from '../../src/robot_platform/viewer/static/collision-checks.js';

const pose=(position,id='robot')=>({id,positions:{joint:position}});
const tick=()=>new Promise(resolve=>setImmediate(resolve));
function fixture() {
  const requests=[],states=[],timers=new Map();let timerId=0;
  const checker=new PoseCollisionChecker({
    request:p=>new Promise((resolve,reject)=>requests.push({pose:p,resolve,reject})),
    onState:s=>states.push(s),
    setTimer:fn=>{timers.set(++timerId,fn);return timerId;},
    clearTimer:id=>timers.delete(id),
  });
  const flush=()=>{const callbacks=[...timers.values()];timers.clear();callbacks.forEach(fn=>fn());};
  return {checker,requests,states,flush};
}

test('rapid edits debounce to the latest complete copied pose',async()=>{
  const {checker,requests,states,flush}=fixture();
  checker.setPose(pose(1));checker.setPose(pose(2));
  const last=pose(3);checker.setPose(last);last.positions.joint=99;
  assert.equal(requests.length,0);flush();
  assert.deepEqual(requests[0].pose,pose(3));
  requests[0].resolve({contacts:['hit']});await tick();
  assert.deepEqual(states.at(-1),{state:'checked',result:{contacts:['hit']}});
});

test('only one request runs; obsolete collision result cannot color a newer pose',async()=>{
  const {checker,requests,states,flush}=fixture();
  checker.setPose(pose(1));flush();
  checker.setPose(pose(2));flush();checker.setPose(pose(3));flush();
  assert.equal(requests.length,1);
  requests[0].resolve({contacts:['old collision']});await tick();
  assert.equal(states.some(s=>s.state==='checked'),false);
  assert.equal(requests.length,2);assert.deepEqual(requests[1].pose,pose(3));
  requests[1].resolve({contacts:[]});await tick();
  assert.deepEqual(states.at(-1),{state:'checked',result:{contacts:[]}});
});

test('switching models invalidates old results and errors',async()=>{
  const {checker,requests,states,flush}=fixture();
  checker.setPose(pose(1,'first'));flush();checker.setPose(null);
  requests[0].reject(new Error('old failure'));await tick();
  assert.equal(states.at(-1).state,'loading');
  checker.setPose(pose(0,'second'));flush();
  assert.equal(requests[1].pose.id,'second');
  requests[1].resolve({contacts:[]});await tick();
  assert.equal(states.at(-1).state,'checked');
});

test('disabling cancels queued checks; manual checks work and errors never mean clear',async()=>{
  const {checker,requests,states,flush}=fixture();
  checker.setPose(pose(1));checker.setEnabled(false);flush();
  assert.equal(requests.length,0);assert.equal(states.at(-1).state,'unchecked');
  checker.checkNow();assert.equal(requests.length,1);
  requests[0].reject(new Error('backend missing'));await tick();
  assert.equal(states.at(-1).state,'unavailable');assert.equal(requests.length,1);
  checker.setPose(pose(2));flush();assert.equal(requests.length,1);
  checker.setEnabled(true);flush();assert.equal(requests.length,2);
  requests[1].resolve({contacts:[]});await tick();
  assert.equal(states.at(-1).state,'checked');
});

test('disabling during a request discards its result',async()=>{
  const {checker,requests,states,flush}=fixture();
  checker.setPose(pose(1));flush();checker.setEnabled(false);
  requests[0].resolve({contacts:['old collision']});await tick();
  assert.equal(states.at(-1).state,'unchecked');
});

test('changing collision scope discards a result for the same pose in the old scope',async()=>{
  const {checker,requests,states,flush}=fixture();
  checker.setPose({...pose(0),scope:'all'});flush();
  checker.setPose({...pose(0),scope:'inter_branch'});flush();
  requests[0].resolve({contacts:['same-arm contact'],query_scope:'all'});await tick();
  assert.equal(states.some(s=>s.state==='checked'),false);
  assert.equal(requests[1].pose.scope,'inter_branch');
  requests[1].resolve({contacts:[],query_scope:'inter_branch'});await tick();
  assert.equal(states.at(-1).result.query_scope,'inter_branch');
});

test('changing geometry policy and inspection gap invalidates the previous result',async()=>{
  const {checker,requests,states,flush}=fixture();
  checker.setPose({...pose(0),mode:'runtime',clearance_m:0});flush();
  checker.setPose({...pose(0),mode:'keepout',clearance_m:.005});flush();
  requests[0].resolve({contacts:['CAD contact']});await tick();
  assert.equal(states.some(s=>s.state==='checked'),false);
  assert.equal(requests[1].pose.mode,'keepout');
  checker.setPose({...pose(0),mode:'keepout',clearance_m:.01});flush();
  requests[1].resolve({contacts:[]});await tick();
  assert.equal(states.some(s=>s.state==='checked'),false);
  assert.equal(requests[2].pose.clearance_m,.01);
  requests[2].resolve({contacts:['keep-out boundary']});await tick();
  assert.equal(states.at(-1).state,'checked');
});
