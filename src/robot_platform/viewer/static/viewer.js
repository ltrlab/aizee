import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {PoseCollisionChecker} from './collision-checks.js';

const api = new URL('../api/models', import.meta.url).pathname;
const $ = id => document.getElementById(id);
const viewport = $('viewport');
const renderer = new THREE.WebGLRenderer({antialias:true, alpha:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.prepend(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45,1,.001,1000);
camera.up.set(0,0,1);
const orbit = new OrbitControls(camera,renderer.domElement);
orbit.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xd3e8f9,0x475365,2));
const light = new THREE.DirectionalLight(0xffffff,2.4); light.position.set(2,-3,5);scene.add(light);
const fill = new THREE.DirectionalLight(0xaecbff,.8);fill.position.set(-2,1,3);scene.add(fill);
let robot=null, model=null, q={}, generation=0, groups=new Map(), motions=new Map(), linkAxes=[], meshes=[], grid=null;
let collisionReady=false, previewLinks=new Set();
let linkNodes=new Map(),keepoutLoaded=false,keepoutPromise=null;
let center=new THREE.Vector3(), radius=1;
const resize=()=>{const w=viewport.clientWidth,h=viewport.clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();};
new ResizeObserver(resize).observe(viewport);resize();
function animate(){requestAnimationFrame(animate);orbit.update();renderer.render(scene,camera);}animate();
function message(text,error=false){$('notice').textContent=text;$('render-error').hidden=!error;$('render-error').textContent=error?text:'';}
async function json(url,options){const response=await fetch(url,options);const data=await response.json();if(!response.ok)throw new Error(data.error||`HTTP ${response.status}`);return data;}
const collisionChecks=new PoseCollisionChecker({
  request:({id,positions,mode,scope,clearance_m})=>json(`${api}/${encodeURIComponent(id)}/collisions`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({positions,mode,scope,clearance_m})}),
  onState:collisionState,
  delay:30,
});
function paintContacts(contacts=[]) {
  const hitLinks=new Set(contacts.filter(c=>!c.provisional_interface).flatMap(c=>c.shapes.map(s=>s.link)));
  const hitShapes=new Set(contacts.filter(c=>!c.provisional_interface).flatMap(c=>c.shapes.map(s=>s.id)));
  for(const mesh of meshes) {
    const hit=hitLinks.has(mesh.userData.link),collision=mesh.userData.role!=='visual';
    mesh.material.emissive.setHex(hit?0xb51c26:previewLinks.has(mesh.userData.link)?0x443016:0);
    if(collision) {
      const partHit=hitShapes.has(mesh.userData.collisionId);
      mesh.material.color.setHex(partHit?0xff3548:0xf5ba66);
      mesh.material.opacity=partHit?.65:.2;
    }
  }
}
function collisionState({state,result,error}) {
  paintContacts(state==='checked'?result.contacts:[]);
  $('pairs').replaceChildren();
  $('runtime-summary').textContent='';
  let text={loading:'Loading collision model…',pending:'Pose changed · collision check pending…',checking:'Checking collisions…',unchecked:'Live checks off · pose not checked',unavailable:`Collision check unavailable: ${error?.message??''}`}[state];
  let tone=state;
  if(state==='checked') {
    const keepout=result.mode==='keepout';
    $('runtime-summary').textContent=keepout?`Keep-out boundaries · ${(result.threshold_m*1000).toFixed(1)} mm gap · ${result.compute_ms.toFixed(2)} ms`:result.mode==='runtime'?`CAD-confirmed contacts · ${result.compute_ms.toFixed(2)} ms`:'';
    if(result.stage_ms)$('runtime-summary').textContent+=keepout?` · ${result.candidate_pairs} / ${result.eligible_pairs} pairs within boundary`:` · coarse ${result.stage_ms.coarse.toFixed(2)} ms · CAD ${result.stage_ms.fine.toFixed(2)} ms · ${result.candidate_pairs} / ${result.eligible_pairs} candidate pairs`;
    const contacts=result.contacts.filter(c=>!c.provisional_interface),interfaces=result.contacts.filter(c=>c.provisional_interface);
    const pairs=new Map(contacts.map(c=>{const links=c.shapes.map(s=>s.link).sort();return [JSON.stringify(links),links];}));
    text=keepout?(pairs.size?`Keep-out boundary reached · ${pairs.size} link pair${pairs.size===1?'':'s'}`:'Outside keep-out boundaries'):(pairs.size?`${pairs.size} colliding link pair${pairs.size===1?'':'s'}`:interfaces.length?'No other contacts detected':'No contacts detected');
    if(interfaces.length)text+=` · ${interfaces.length} provisional interface contacts`;
    if(!result.whole_robot_checked)text+=` · selected scope only (${result.query_scope.replaceAll('_',' ')})`;
    if(keepout&&Object.keys(result.ignored_links).length)text+=` · ${Object.keys(result.ignored_links).length} links excluded`;
    tone=pairs.size?'collision':interfaces.length?'interfaces':'clear';
    for(const pair of pairs.values()){const line=document.createElement('div');line.textContent=pair.join(' ↔ ');$('pairs').append(line);}
    if(interfaces.length) {
      const details=document.createElement('details'),summary=document.createElement('summary');
      details.className='interface-contacts';summary.textContent=`${interfaces.length} provisional assembly interface contacts`;details.append(summary);
      for(const c of interfaces){const line=document.createElement('p');line.textContent=`${c.shapes.map(s=>s.id).join(' ↔ ')} — ${c.reason}`;details.append(line);}
      $('pairs').append(details);
    }
  }
  for(const id of ['collision-status','live-collision-status']){$(id).textContent=text;$(id).dataset.state=tone;}
  $('check').disabled=!collisionReady||state==='checking';
}
function transform(object,origin){object.position.fromArray(origin.xyz);object.rotation.set(...origin.rpy,'ZYX');}
function frame(direction=[1.35,-1.7,.95]){const delta=new THREE.Vector3(...direction).normalize().multiplyScalar(radius*2.8);camera.position.copy(center).add(delta);orbit.target.copy(center);camera.near=Math.max(radius/1000,.0001);camera.far=Math.max(radius*100,10);camera.updateProjectionMatrix();orbit.update();}
$('fit').onclick=()=>frame();
$('front').onclick=()=>frame(model?.front_direction??[1,0,0]);
$('top').onclick=()=>{
  const [x,y]=model?.front_direction??[1,0,0];
  // Approach the top from the front to preserve the Front view's left/right.
  // A tiny horizontal offset keeps Z-up orbit controls away from the pole.
  frame([x*.001,y*.001,1]);
};
$('axes').onchange=()=>linkAxes.forEach(a=>a.visible=$('axes').checked);
function showCollisionGeometry(){
  const keepout=$('collision-mode').value==='keepout';
  $('geometry-label').textContent=keepout?'Keep-out envelopes':'Collision geometry';
  $('keepout-gap-row').hidden=!keepout;$('keepout-policy').hidden=!keepout;
  const ignored=Object.entries(model?.keepout_ignored_links??{});
  $('keepout-policy').textContent=`Envelopes restrict motion before CAD contact. Inspection gap is a viewing setting, not a calibrated stopping margin. ${ignored.length?'Excluded links: '+ignored.map(([n])=>n).join(', ')+'. Wrist and other links remain checked.':'No excluded links.'}`;
  meshes.forEach(m=>m.visible=m.userData.role==='visual'||m.userData.role===(keepout?'keepout':'collision')&&$('collision-geometry').checked);
}
function collisionPose(){return {id:model.id,positions:q,mode:$('collision-mode').value,scope:$('collision-scope').value,clearance_m:$('collision-mode').value==='keepout'?$('keepout-gap').valueAsNumber/1000:0};}
async function ensureKeepout(){
  if(keepoutLoaded)return;
  if(keepoutPromise)return keepoutPromise;
  const token=generation,id=model.id;
  keepoutPromise=(async()=>{
    const data=await json(`${api}/${encodeURIComponent(id)}/runtime-model`);
    if(token!==generation)return;
    for(const desc of data.bounds){
      if(Object.hasOwn(data.keepout_ignored_links,desc.link))continue;
      let geom;
      if(desc.shape==='convex'){
        geom=new THREE.BufferGeometry();geom.setAttribute('position',new THREE.Float32BufferAttribute(desc.vertices.flat(),3));geom.setIndex(desc.faces.flat());geom.computeVertexNormals();
      }else if(desc.shape==='box')geom=new THREE.BoxGeometry(...desc.size);
      else if(desc.shape==='sphere')geom=new THREE.SphereGeometry(desc.radius,20,12);
      else {geom=new THREE.CylinderGeometry(desc.radius,desc.radius,desc.length,20);geom.rotateX(Math.PI/2);}
      const material=new THREE.MeshStandardMaterial({color:0xf5ba66,flatShading:true,transparent:true,opacity:.2,depthWrite:false,side:THREE.DoubleSide});
      const object=new THREE.Mesh(geom,material);transform(object,desc.origin);object.userData={link:desc.link,role:'keepout',collisionId:desc.id};object.visible=false;linkNodes.get(desc.link).add(object);meshes.push(object);
    }
    keepoutLoaded=true;
  })();
  try{await keepoutPromise;}catch(e){if(token===generation)keepoutPromise=null;throw e;}
}
$('collision-geometry').onchange=()=>{showCollisionGeometry();if($('collision-geometry').checked)collisionChecks.checkNow();};
$('collision-mode').onchange=async()=>{
  const token=generation;
  if($('collision-mode').value==='keepout')$('collision-scope').value='inter_branch';
  showCollisionGeometry();applyPose(q);
  try{if($('collision-mode').value==='keepout')await ensureKeepout();if(token!==generation)return;showCollisionGeometry();applyPose(q);collisionChecks.checkNow();}
  catch(error){if(token===generation)collisionState({state:'unavailable',error});}
};
$('collision-scope').onchange=()=>{applyPose(q);collisionChecks.checkNow();};
$('keepout-gap').onchange=()=>{applyPose(q);collisionChecks.checkNow();};
$('live-collisions').onchange=()=>collisionChecks.setEnabled($('live-collisions').checked);
function allPositions(positions){
  const result={...positions};
  for(const name of model.mimic_order){const m=model.joints.find(j=>j.name===name).mimic;result[name]=result[m.joint]*m.multiplier+m.offset;}
  for(const joint of model.joints){if(joint.type==='fixed')continue;const value=result[joint.name];if(!Number.isFinite(value)||joint.bounds&&(value<joint.bounds[0]-1e-10||value>joint.bounds[1]+1e-10))throw new Error(`Pose outside URDF bounds: ${joint.name}`);}
  for(const joint of model.joints){if(joint.limit_review&&(result[joint.name]<joint.control_bounds[0]-1e-10||result[joint.name]>joint.control_bounds[1]+1e-10))throw new Error(`Pose outside candidate viewing range: ${joint.name}`);}
  return result;
}
function applyPose(positions){
  const values=allPositions(positions);q={...positions};previewLinks.clear();
  for(const joint of model.joints){const object=motions.get(joint.name);if(joint.type==='prismatic')object.position.fromArray(joint.axis).multiplyScalar(values[joint.name]);else if(joint.type!=='fixed')object.quaternion.setFromAxisAngle(new THREE.Vector3(...joint.axis),values[joint.name]);}
  collisionChecks.setPose(collisionReady&&($('collision-mode').value!=='keepout'||keepoutLoaded)?collisionPose():null);
  for(const name of model.independent){const row=groups.get(name);if(row){row.range.value=q[name];row.number.value=q[name].toFixed(row.digits);}}
}
function controls(){
  $('joints').replaceChildren();groups=new Map();
  for(const [label,names] of Object.entries(model.groups)){
    const section=document.createElement('details');section.className='joint-group';section.open=true;
    const heading=document.createElement('summary');heading.textContent=label;section.append(heading);
    for(const name of names){
      const joint=model.joints.find(j=>j.name===name), [lo,hi]=joint.control_bounds;
      const row=document.createElement('div');row.className='joint-row';
      const top=document.createElement('div');top.className='joint-top';
      const label=document.createElement('label');label.className='joint-name';label.textContent=name;
      const number=document.createElement('input');number.type='number';number.className='number';number.min=lo;number.max=hi;number.step=joint.unit==='m'?.0001:.001;number.setAttribute('aria-label',`${name} value (${joint.unit})`);
      const inputWrap=document.createElement('div');const unit=document.createElement('span');unit.className='unit';unit.textContent=joint.unit;inputWrap.append(number,unit);top.append(label,inputWrap);
      const range=document.createElement('input');range.type='range';range.min=lo;range.max=hi;range.step='any';range.setAttribute('aria-label',`${name} slider`);range.disabled=lo===hi;
      const digits=joint.unit==='m'?4:3;
      const set=value=>{try{if(!Number.isFinite(value)||value<lo||value>hi)throw new Error(`Enter a value between ${lo.toFixed(digits)} and ${hi.toFixed(digits)}`);applyPose({...q,[name]:value});message('');}catch(e){message(e.message);range.value=q[name];number.value=q[name].toFixed(digits);}};
      range.oninput=()=>set(Number(range.value));number.onchange=()=>set(number.value===''?NaN:Number(number.value));
      const bottom=document.createElement('div');bottom.className='joint-bottom';
      const limits=document.createElement('span');limits.textContent=`${lo.toFixed(digits)} to ${hi.toFixed(digits)} ${joint.unit}`;
      const jog=document.createElement('span');jog.className='jog';
      for(const [text,step] of [['−',-1],['+',1]]){const button=document.createElement('button');button.textContent=text;button.setAttribute('aria-label',`${name} jog ${step<0?'minus':'plus'}`);button.onclick=()=>set(Math.max(lo,Math.min(hi,q[name]+step*(joint.unit==='m'?.001:.02))));jog.append(button);}
      bottom.append(limits,jog);row.append(top,range,bottom);
      if(joint.limit_review){
        const result=joint.limit_review, badge=document.createElement('div');badge.className='badge';
        badge.textContent=result.status!=='candidate'?'Collision unresolved · range locked':`Collision candidate · ${joint.unit==='rad'?`${(lo*180/Math.PI).toFixed(1)}° to ${(hi*180/Math.PI).toFixed(1)}°`:`${(lo*1000).toFixed(2)} to ${(hi*1000).toFixed(2)} mm`}`;
        row.append(badge);
        const endpoints=document.createElement('div');endpoints.className='endpoint-buttons';
        for(const [side,label] of [['lower','Lower'],['upper','Upper']]){const button=document.createElement('button');button.textContent=label;button.setAttribute('aria-label',`${name} preview ${side} limit`);button.disabled=result.status!=='candidate';button.onclick=()=>previewLimit(name,side);endpoints.append(button);}
        row.append(endpoints);
        if(result.lower?.status==='search_bound'||result.upper?.status==='search_bound'){const note=document.createElement('div');note.className='hint';note.textContent='One or both ends reached the search bound without contact.';row.append(note);}
      }else if(model.provisional_limits.includes(name)||joint.type==='continuous'){const badge=document.createElement('div');badge.className='badge';badge.textContent=joint.type==='continuous'?'Continuous joint · viewing window':'Provisional exporter range';row.append(badge);}
      section.append(row);groups.set(name,{range,number,digits});
    }
    $('joints').append(section);
  }
  $('mimics').replaceChildren();
  for(const name of model.mimic_order){const j=model.joints.find(j=>j.name===name);const line=document.createElement('p');line.textContent=`${name} = ${j.mimic.multiplier} × ${j.mimic.joint} + ${j.mimic.offset} ${j.unit}`;$('mimics').append(line);}
  if(!model.mimic_order.length)$('mimics').textContent='No coupled joints in this model.';
}
function previewLimit(name,side){
  const joint=model.joints.find(j=>j.name===name), end=joint.limit_review[side];
  applyPose({...model.limit_review.reference_positions,[name]:end.candidate});
  const lookup=new Map(model.limit_review.shape_metadata.map(s=>[s.id,s]));
  previewLinks=new Set(end.pairs.flat().map(id=>lookup.get(id)?.link));
  paintContacts();
  const contact=end.pairs.map(pair=>pair.map(id=>lookup.get(id)?.source?.replace(/^.*[\\/]/,'')||id).join(' ↔ ')).join('; ');
  message(`${name} ${side}: ${end.status==='contact'?'inward from sampled contact; amber marks the limit-driving links. '+contact:'search bound reached without contact; mechanical endpoint unknown.'}`);
}
$('sweep-reference').onclick=()=>{applyPose(model.limit_review.reference_positions);message('Sweep reference pose restored');};
function dispose(){if(robot){scene.remove(robot);robot.traverse(o=>{o.geometry?.dispose();if(o.material)o.material.dispose();});}if(grid){scene.remove(grid);grid.geometry.dispose();grid.material.dispose();grid=null;}meshes=[];linkAxes=[];}
async function loadModel(id){
  const token=++generation;collisionReady=false;previewLinks.clear();collisionChecks.setPose(null);dispose();model=null;robot=new THREE.Group();scene.add(robot);groups=new Map();motions=new Map();
  keepoutLoaded=false;keepoutPromise=null;linkNodes=new Map();
  $('joints').replaceChildren();$('pairs').replaceChildren();$('loading').textContent='Reading model…';message('');$('check').disabled=true;
  $('limit-review').hidden=true;
  for(const n of ['reset','export','import'])$(n).disabled=true;
  try{
    const data=await json(`${api}/${encodeURIComponent(id)}`);if(token!==generation)return;model=data;
    $('collision-mode').querySelector('[value="runtime"]').disabled=!data.runtime_collision_available;
    $('collision-mode').querySelector('[value="keepout"]').disabled=!data.runtime_collision_available;
    if(!data.runtime_collision_available)$('collision-mode').value='source';
    $('runtime-summary').textContent='';
    document.title=`${model.title} · Robot model viewer`;$('model-title').textContent=model.title;$('summary').textContent=`${model.links.length} links · ${model.independent.length} controls · ${model.mimic_order.length} coupled joints`;
    if(model.limit_review){const r=model.limit_review;$('limit-review').hidden=false;$('limit-review-summary').textContent=`${r.source_version||'Current CAD revision'} · ${r.samples_checked} sampled poses. Each arm checked against itself and the body; the other arm excluded. ${r.assembly_interfaces.length} provisional part-interface exceptions. Viewer ranges only; no physical limits changed.`;$('sweep-report').href=`${api}/${encodeURIComponent(id)}/limit-review`;}
    $('collision-hint').textContent='Checks cover the selected scope. Red marks CAD contacts or keep-out boundary violations for the selected mode. Amber shows the checked geometry. Provisional CAD interfaces are listed separately. Jogging remains free and no hardware is connected.';
    $('notes-body').replaceChildren();for(const note of [...model.notes,'Joint values are mechanical CAD coordinates. Pose files do not change controller settings.']){const p=document.createElement('p');p.textContent=note;$('notes-body').append(p);}
    const links=linkNodes=new Map(model.links.map(n=>[n,new THREE.Group()]));robot.add(links.get(model.root));
    for(const joint of model.joints){const frame=new THREE.Group(),motion=new THREE.Group();transform(frame,joint.origin);links.get(joint.parent).add(frame);frame.add(motion);motion.add(links.get(joint.child));motions.set(joint.name,motion);}
    for(const link of links.values()){const axes=new THREE.AxesHelper(.06);axes.visible=$('axes').checked;link.add(axes);linkAxes.push(axes);}
    const geometries=new Map();
    async function meshGeometry(index){
      if(!geometries.has(index))geometries.set(index,(async()=>{const response=await fetch(`${api}/${encodeURIComponent(id)}/meshes/${index}`);if(!response.ok){const error=await response.json();throw new Error(error.error||'Mesh load failed');}const buffer=await response.arrayBuffer(),header=new DataView(buffer),nv=header.getUint32(0,true),nf=header.getUint32(4,true);if(buffer.byteLength!==8+nv*12+nf*12)throw new Error('Incomplete mesh response');const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(buffer,8,nv*3),3));geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(buffer,8+nv*12,nf*3),1));geometry.computeVertexNormals();return geometry;})());
      return geometries.get(index);
    }
    let done=0,next=0;const failures=[];
    async function worker(){while(token===generation&&next<data.geometry.length){const desc=data.geometry[next++];try{let geom;
      if(desc.shape==='mesh')geom=await meshGeometry(desc.asset);
      else if(desc.shape==='box')geom=new THREE.BoxGeometry(...desc.size);
      else if(desc.shape==='sphere')geom=new THREE.SphereGeometry(desc.radius,28,20);
      else {geom=new THREE.CylinderGeometry(desc.radius,desc.radius,desc.length,28);geom.rotateX(Math.PI/2);}
      if(token!==generation)return;
      const collision=desc.role!=='visual';const material=new THREE.MeshStandardMaterial({color:collision?0xf5ba66:new THREE.Color(...desc.color.slice(0,3)),flatShading:true,roughness:.8,metalness:0,transparent:collision||desc.color[3]<1,opacity:collision?.2:desc.color[3],depthWrite:!collision,side:THREE.DoubleSide});
      const object=new THREE.Mesh(geom,material);transform(object,desc.origin);if(desc.scale)object.scale.fromArray(desc.scale);object.userData={link:desc.link,role:desc.role,collisionId:desc.collision_id};object.visible=!collision||$('collision-geometry').checked;links.get(desc.link).add(object);meshes.push(object);
    }catch(e){failures.push(`${desc.link}: ${e.message}`);}finally{done++;if(token===generation)$('loading').textContent=`Loading geometry ${done}/${data.geometry.length}`;}}}
    await Promise.all(Array.from({length:4},worker));if(token!==generation)return;
    if($('collision-mode').value==='keepout')await ensureKeepout();if(token!==generation)return;
    controls();applyPose(Object.fromEntries(model.independent.map(n=>[n,model.joints.find(j=>j.name===n).default])));robot.updateMatrixWorld(true);
    // Frame visual geometry only; hidden collision shapes/axes cannot skew it.
    const box=new THREE.Box3();for(const m of meshes.filter(m=>m.userData.role==='visual')){m.geometry.computeBoundingBox();box.union(m.geometry.boundingBox.clone().applyMatrix4(m.matrixWorld));}
    if(box.isEmpty()){for(const m of meshes){m.geometry.computeBoundingBox();box.union(m.geometry.boundingBox.clone().applyMatrix4(m.matrixWorld));}if(box.isEmpty())box.setFromCenterAndSize(new THREE.Vector3(),new THREE.Vector3(1,1,1));}
    box.getCenter(center);radius=Math.max(box.getSize(new THREE.Vector3()).length()/2,.01);frame();
    linkAxes.forEach(a=>a.scale.setScalar(radius/3));grid=new THREE.GridHelper(radius*4,20,0x425665,0x273846);grid.rotation.x=Math.PI/2;grid.position.set(center.x,center.y,Math.min(0,box.min.z)-radius*.005);scene.add(grid);
    $('loading').textContent=failures.length?'Incomplete geometry':'Model ready';
    if(failures.length)message(`${failures.length} geometry load failures. ${failures[0]}`,true);
    for(const n of ['reset','export','import','check'])$(n).disabled=false;
    collisionReady=true;showCollisionGeometry();collisionChecks.setPose(collisionPose());
  }catch(e){if(token===generation){collisionReady=false;collisionChecks.setPose(null);collisionState({state:'unavailable',error:e});$('loading').textContent='Model unavailable';message(e.message,true);}}
}
$('reset').onclick=()=>{applyPose(Object.fromEntries(model.independent.map(n=>[n,model.joints.find(j=>j.name===n).default])));message('Pose reset');};
$('export').onclick=()=>{const pose={schema_version:1,model:{id:model.id,urdf_sha256:model.urdf_sha256,profile_sha256:model.profile_sha256},positions:q};const url=URL.createObjectURL(new Blob([JSON.stringify(pose,null,2)+'\n'],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download=`${model.id}-pose.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);message('Pose exported');};
$('import').onclick=()=>$('pose-file').click();
$('pose-file').onchange=async()=>{try{const pose=JSON.parse(await $('pose-file').files[0].text());if(pose.schema_version!==1||pose.model?.urdf_sha256!==model.urdf_sha256)throw new Error('Pose belongs to a different model revision');if(!pose.positions||Object.keys(pose.positions).sort().join('\0')!==[...model.independent].sort().join('\0')||Object.values(pose.positions).some(v=>typeof v!=='number'))throw new Error('Pose must contain every independent joint');applyPose(pose.positions);message('Pose loaded');}catch(e){message(e.message);}finally{$('pose-file').value='';}};
$('check').onclick=()=>collisionChecks.checkNow();
try{const catalog=await json(api);for(const entry of catalog.models){const option=document.createElement('option');option.value=entry.id;option.textContent=entry.title;$('model').append(option);}$('model').value=catalog.selected;$('model').onchange=()=>loadModel($('model').value);await loadModel(catalog.selected);}catch(e){message(e.message,true);}
