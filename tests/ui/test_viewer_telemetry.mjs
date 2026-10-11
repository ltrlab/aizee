import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

const stage=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
const source=await fs.readFile(path.join(stage,'src/robot_platform/viewer/static/joint-telemetry.js'),'utf8');
const {JointTelemetry,calloutColumns,describeJoint,rotationArc,overlaySettings,trustedOverlayMessage}=await import(
  `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const model={id:'minerva',urdf_sha256:'test-urdf',joints:[
  {name:'left_arm_j1',type:'revolute'}, {name:'left_gripper',type:'prismatic'}, {name:'base',type:'fixed'}]};
const row={motor:'left_arm_j1',joint:'left_arm_j1',label:'Left J1',position:-.2,position_unit:'rad',
  velocity:.3,velocity_unit:'rad/s',effort:-.4,effort_unit:'Nm',effort_label:'Motor torque',temperature_c:30,
  age_seconds:.02,source:'simulated',mapping_verified:true};
const packet=(changes={})=>({model:'minerva',urdf_sha256:'test-urdf',stream_id:'simulated-test',sequence:1,
  age_seconds:.01,stale_after_s:.1,stale:false,rows:[{...row}],...changes});
let passed=0;
function test(name,fn){fn();passed++;console.log(`PASS ${name}`);}

test('fresh signed readings retain explicit simulated and motor torque source',()=>{
  const state=new JointTelemetry();assert.equal(state.accept(packet(),1000,model),true);
  const reading=state.snapshot(1010).rows[0], display=describeJoint(reading);
  assert.equal(display.source,'Simulated');assert.equal(display.direction,1);
  assert.equal(display.lines[0].value,'−0.200 rad');assert.equal(display.lines[1].value,'−0.40 Nm');
  assert.equal(display.lines[1].label,'Motor torque');assert.equal(display.lines[2].value,'30.0 °C');
});
test('stale samples retain labeled last values and suppress motion',()=>{
  const state=new JointTelemetry();state.accept(packet(),1000,model);
  const display=describeJoint(state.snapshot(1101).rows[0]);assert.equal(display.direction,0);
  assert.match(display.status,/STALE/);assert.match(display.lines.at(-1).value,/direction hidden/);
});
test('repeated frames cannot refresh age or substitute changed payload',()=>{
  const state=new JointTelemetry();state.accept(packet({rows:[{...row,age_seconds:.08}]}),1000,model);
  assert.equal(state.accept(packet({age_seconds:0,rows:[{...row,position:9,age_seconds:0}]}),1030,model),false);
  const value=state.snapshot(1030).rows[0];assert.equal(value.position,-.2);assert.ok(value.age_seconds>=.11);
  assert.equal(value.stale,true);
});
test('a repeated frame can make existing readings staler but never fresher',()=>{
  const state=new JointTelemetry();state.accept(packet(),1000,model);
  state.accept(packet({rows:[{...row,age_seconds:.5,stale:true}]}),1010,model);
  const reading=state.snapshot(1010).rows[0];assert.equal(reading.stale,true);assert.ok(reading.age_seconds>=.5);
});
test('out-of-order identity cannot replace newer source; new empty owner clears it',()=>{
  const state=new JointTelemetry();state.accept(packet({sequence:4}),1000,model);
  state.accept(packet({sequence:3,rows:[{...row,position:9}]}),1010,model);
  assert.equal(state.snapshot(1010).rows[0].position,-.2);
  state.accept(packet({stream_id:'new-owner',sequence:1,rows:[]}),1020,model);
  assert.equal(state.snapshot(1020).rows.length,0);
});
test('mismatched model or malformed identity clears prior readings',()=>{
  const state=new JointTelemetry();state.accept(packet(),0,model);
  assert.equal(state.accept(packet({urdf_sha256:'wrong'}),10,model),false);
  assert.equal(state.snapshot(10).rows.length,0);assert.match(state.snapshot(10).reason,/mismatch/);
  state.accept(packet(),20,model);state.accept(packet({sequence:null}),30,model);
  assert.equal(state.snapshot(30).rows.length,0);
});
test('missing/invalid numbers stay unavailable, never zero or fabricated torque',()=>{
  const state=new JointTelemetry();state.accept(packet({age_seconds:undefined,rows:[{...row,
    position:null,velocity:NaN,effort:undefined,temperature_c:Infinity,age_seconds:undefined}]}),0,model);
  const value=state.snapshot(0).rows[0],display=describeJoint(value);
  assert.equal(value.stale,true);assert.equal(display.direction,0);
  assert.deepEqual(display.lines.slice(0,3).map(line=>line.value),['—','—','—']);
});
test('current stays amperes and unknown effort units are visibly unknown',()=>{
  const state=new JointTelemetry();state.accept(packet({rows:[{...row,effort:1.2,effort_unit:'A',effort_label:'Motor current'}]}),0,model);
  let effort=describeJoint(state.snapshot(0).rows[0]).lines[1];assert.equal(effort.value,'+1.20 A');
  state.accept(packet({sequence:2,rows:[{...row,effort_unit:null}]}),10,model);
  effort=describeJoint(state.snapshot(10).rows[0]).lines[1];assert.match(effort.value,/units unknown/);
});
test('unverified mapping and commanded source cannot imply measured rotation',()=>{
  for(const change of [{mapping_verified:false},{mapping_verified:undefined},{source:'commanded'}]) {
    const state=new JointTelemetry();state.accept(packet({rows:[{...row,...change}]}),0,model);
    assert.equal(describeJoint(state.snapshot(0).rows[0]).direction,0);
  }
});
test('prismatic units require mapped metres and linear velocity',()=>{
  const state=new JointTelemetry();state.accept(packet({rows:[{...row,motor:'gripper',joint:'left_gripper',
    position:.01,position_unit:'m',velocity:-.02,velocity_unit:'m/s'}]}),0,model);
  let display=describeJoint(state.snapshot(0).rows[0]);assert.equal(display.lines[0].value,'+0.010 m');assert.equal(display.direction,-1);
  state.accept(packet({sequence:2,rows:[{...row,joint:'left_gripper'}]}),10,model);
  display=describeJoint(state.snapshot(10).rows[0]);assert.equal(display.lines[0].value,'—');assert.equal(display.direction,0);
});
test('unknown/fixed joint anchors are not invented and selected is the default',()=>{
  const state=new JointTelemetry();state.accept(packet({rows:[{...row,joint:'missing'},{...row,joint:'base'}]}),0,model);
  assert.equal(state.snapshot(0).rows.length,0);assert.equal(overlaySettings().scope,'selected');
  const settings=overlaySettings(undefined,{fields:{effort:false},scope:'all'});
  assert.equal(settings.fields.effort,false);assert.equal(settings.fields.position,true);assert.equal(settings.scope,'all');
  assert.equal(overlaySettings(settings,{scope:'invalid',enabled:'true'}).scope,'all');
});
test('parent messages require exact parent, origin, type and version',()=>{
  const parent={},message={source:parent,origin:'http://127.0.0.1:8760',data:{type:'helm-viewer-overlay',version:1}};
  assert.equal(trustedOverlayMessage(message,parent,message.origin),true);
  assert.equal(trustedOverlayMessage({...message,source:{}},parent,message.origin),false);
  assert.equal(trustedOverlayMessage({...message,origin:'https://other.invalid'},parent,message.origin),false);
  assert.equal(trustedOverlayMessage({...message,data:{...message.data,version:2}},parent,message.origin),false);
});
test('rotation geometry obeys right-hand rule on every URDF axis and reverses sign',()=>{
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const dot=(a,b)=>a.reduce((sum,x,i)=>sum+x*b[i],0);
  for(const axis of [[1,0,0],[0,1,0],[0,0,1],[1,-2,3]]) for(const sign of [-1,1]) {
    const arc=rotationArc(axis,sign,.1),point=arc.points.at(-1);
    assert.ok(Math.abs(dot(point,axis))<1e-10);assert.ok(Math.abs(Math.hypot(...point)-.1)<1e-10);
    assert.equal(Math.sign(dot(cross(point,arc.tangent),axis)),sign);
  }
  assert.equal(rotationArc([0,0,0],1,.1),null);
});
test('all-joint labels split into edge columns that keep anchor order and never overlap',()=>{
  const items=[];
  for(const [side,x] of [['right',380],['left',120]]) for(let i=0;i<7;i++)
    items.push({id:`${side}${i}`,anchorX:x+i*3,anchorY:200+i*12,height:36});  // Joints closer together than labels.
  const placed=calloutColumns(items,{width:500,top:145,bottom:600,cardWidth:170});
  assert.equal(placed.length,14);
  for(const side of ['left','right']) {
    const column=placed.filter(item=>item.side===side);
    assert.ok(column.every(item=>item.id.startsWith(side)));
    assert.ok(column.every(item=>item.x===(side==='left'?12:500-170-12)));
    assert.deepEqual(column.map(item=>item.id),column.map((_,i)=>`${side}${i}`));
    for(let i=1;i<column.length;i++) assert.ok(column[i].y>=column[i-1].y+36+4);
    assert.ok(column[0].y>=145 && column.at(-1).y+36<=600);
  }
  const crowded=calloutColumns(items.slice(0,7),{width:500,top:145,bottom:300,cardWidth:170});
  assert.ok(crowded.every(item=>item.y>=145 && item.y+36<=300));  // Too many to fit: they overlap but stay in view.
  assert.deepEqual(calloutColumns([],{width:500,top:0,bottom:100,cardWidth:100}),[]);
});
// Minimal DOM plus real vendored Three objects exercise frame anchoring and the
// parent-message/display lifecycle without creating WebGL, opening a browser or
// reading hardware. All fixture values are explicitly simulated.
class Element {
  constructor(tag) {
    this.tagName=tag;this.children=[];this.parentNode=null;this.hidden=false;this.offsetHeight=140;
    this.style={setProperty(key,value){this[key]=value;}};this.attributes={};this.listeners={};
    const classes=new Set();this.classList={add:name=>classes.add(name),contains:name=>classes.has(name),
      toggle:(name,enabled)=>enabled ? classes.add(name) : classes.delete(name)};
  }
  append(...children){for(const child of children){child.parentNode=this;this.children.push(child);}}
  replaceChildren(...children){for(const child of this.children)child.parentNode=null;this.children=[];this.append(...children);}
  remove(){if(this.parentNode)this.parentNode.children=this.parentNode.children.filter(child=>child!==this);this.parentNode=null;}
  setAttribute(key,value){this.attributes[key]=String(value);}
  addEventListener(name,fn){this.listeners[name]=fn;}
  querySelector(){return {offsetHeight:100};}
}
const messages=[],listeners={};
const parent={postMessage:(value,origin)=>messages.push({value,origin})};
globalThis.document={createElement:tag=>new Element(tag),createElementNS:(_,tag)=>new Element(tag)};
globalThis.window={parent,addEventListener:(name,fn)=>{listeners[name]=fn;}};
globalThis.location={origin:'http://127.0.0.1:8760'};
const vendor=path.resolve(process.env.AIZEE_VIEWER_THREE ||
  'C:/Users/ltr/.codex/worktrees/dev-baseline/aizee/src/robot_platform/viewer/static/vendor/three.module.js');
const threeURL=`data:text/javascript;base64,${Buffer.from(await fs.readFile(vendor,'utf8')).toString('base64')}`;
const telemetryURL=`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
const overlaySource=(await fs.readFile(path.join(stage,'src/robot_platform/viewer/static/joint-overlay.js'),'utf8'))
  .replace("from 'three'",`from '${threeURL}'`).replace("from './joint-telemetry.js'",`from '${telemetryURL}'`);
const THREE=await import(threeURL);
const {JointOverlay}=await import(`data:text/javascript;base64,${Buffer.from(overlaySource).toString('base64')}`);
function makeOverlay() {
  const viewport=new Element('section');viewport.clientWidth=1000;viewport.clientHeight=800;
  const camera=new THREE.PerspectiveCamera(45,1.25,.01,100);camera.position.set(0,0,4);camera.lookAt(0,0,0);camera.updateMatrixWorld(true);
  const renderer={domElement:new Element('canvas')},scene=new THREE.Scene();
  const frame=new THREE.Group(),motion=new THREE.Group();frame.rotation.z=Math.PI/2;frame.add(motion);scene.add(frame);
  const detailed={...model,joints:model.joints.map(j=>({...j,axis:[1,0,0]}))};
  const overlay=new JointOverlay({viewport,camera,renderer,scene});
  overlay.setModel(detailed,new Map([['left_arm_j1',motion]]),[],1);
  overlay.telemetry.accept(packet(),1000,detailed);return {overlay,scene,frame};
}
test('real Three arrow follows transformed URDF axis and remains at joint origin',()=>{
  const {overlay,scene,frame}=makeOverlay();scene.updateMatrixWorld(true);overlay.update(1000);scene.updateMatrixWorld(true);
  const card=overlay.cards.get('left_arm_j1');assert.equal(card.source.textContent,'Simulated · left_arm_j1');
  assert.equal(card.arrow.parent,frame);assert.equal(card.arrow.visible,true);
  const worldAxis=new THREE.Vector3(1,0,0).transformDirection(card.arrow.matrixWorld);
  assert.ok(worldAxis.distanceTo(new THREE.Vector3(0,1,0))<1e-10);
  const origin=card.arrow.getWorldPosition(new THREE.Vector3());assert.ok(origin.distanceTo(frame.getWorldPosition(new THREE.Vector3()))<1e-10);
  overlay.update(1200);assert.equal(card.arrow.visible,false);assert.ok(card.node.classList.contains('stale'));
  assert.match(card.age.textContent,/STALE/);
});
test('parent preferences control display and selection emits display-only message',()=>{
  const {overlay}=makeOverlay();overlay.update(1000);
  listeners.message({source:parent,origin:location.origin,data:{type:'helm-viewer-overlay',version:1,
    settings:{enabled:true,scope:'selected',fields:{effort:false,temperature:false,direction:false}}}});
  overlay.update(1010);const card=overlay.cards.get('left_arm_j1');
  assert.equal(card.readings.children.length,2);assert.equal(card.arrow.visible,false);
  overlay.choose('left_arm_j1');assert.deepEqual(messages.at(-1).value,{type:'helm-viewer-selection',version:1,motor:'left_arm_j1'});
  listeners.message({source:parent,origin:location.origin,data:{type:'helm-viewer-overlay',version:1,settings:{enabled:false}}});
  overlay.update(1020);assert.equal(overlay.container.hidden,true);assert.equal(overlay.toolbar.hidden,true);
});
test('all-joint scope shows compact readings and marks the selected joint',()=>{
  const {overlay}=makeOverlay();
  listeners.message({source:parent,origin:location.origin,data:{type:'helm-viewer-overlay',version:1,
    settings:{enabled:true,scope:'all',selected_motor:'left_arm_j1'}}});
  overlay.update(1000);const card=overlay.cards.get('left_arm_j1');
  assert.ok(card.node.classList.contains('compact'));assert.ok(card.node.classList.contains('selected'));
  assert.equal(card.values.textContent,'−0.200 rad · −0.40 Nm · 30.0 °C');
  listeners.message({source:parent,origin:location.origin,data:{type:'helm-viewer-overlay',version:1,
    settings:{enabled:true,scope:'selected'}}});
  overlay.update(1010);assert.equal(card.node.classList.contains('compact'),false);
});
test('model teardown removes labels, arrows and leader lines',()=>{
  const {overlay,frame}=makeOverlay();overlay.update(1000);assert.equal(overlay.cards.size,1);
  overlay.clearModel();assert.equal(overlay.cards.size,0);assert.equal(overlay.leaders.children.length,0);
  assert.equal(frame.children.length,1); // original motion node only
});
console.log(`${passed} viewer telemetry tests passed; no browser, receiver or network used.`);
