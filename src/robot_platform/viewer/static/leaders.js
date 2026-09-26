import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';

const $ = id => document.getElementById(id);
const api = new URL('../api/', import.meta.url);
let sides = [];
const staleMs = 200;
const pollIntervalMs = 1000 / 60;
const arms = new Map();
const axes = [], labels = [];
let profile, renderer, scene, camera, orbit, resizeObserver, fixture;
let state = null, receivedAt = 0, transportError = '', busy = false, fatal = false;
let actionGeneration = 0;
let firstPoseFitted = false;
const progress = new Map();
let deliveryWindowAt = performance.now(), deliveryCount = 0, deliveryHz = null;

function element(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

async function request(path, options = {}) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 1800);
  try {
    const response = await fetch(new URL(path, api), {...options, cache:'no-store', signal:controller.signal});
    const body = await response.json();
    if (!response.ok || body.ok === false) throw new Error(body.error || `HTTP ${response.status}`);
    return body;
  } catch (error) {
    if (error.name === 'AbortError') throw new Error('Server request timed out');
    throw error;
  } finally { clearTimeout(timeout); }
}

function finiteArray(value, count) {
  return Array.isArray(value) && value.length === count && value.every(Number.isFinite);
}

function validateProfile(value) {
  if (!value || typeof value.action_token !== 'string') throw new Error('Display profile is missing its action token');
  const names = Object.keys(value.leaders || {});
  if (!names.length) throw new Error('Display profile contains no leaders');
  for (const side of names) {
    const arm = value.leaders?.[side];
    if (!arm || !finiteArray(arm.base_xyz, 3) || !Array.isArray(arm.joints) || arm.joints.length !== 7) {
      throw new Error(`Invalid ${side} leader profile`);
    }
    if (arm.base_rpy !== undefined && !finiteArray(arm.base_rpy, 3)) throw new Error(`Invalid ${side} base orientation`);
    for (const joint of arm.joints) {
      if (!finiteArray(joint.axis, 3) || !joint.axis.some(v => v !== 0) || !finiteArray(joint.offset_xyz, 3)
          || !Number.isFinite(joint.length_m) || joint.length_m < 0 || !Number.isFinite(joint.zero_rad)
          || ![-1, 1].includes(joint.direction)) throw new Error(`Invalid joint geometry in ${side} leader profile`);
      if (joint.origin_rpy !== undefined && !finiteArray(joint.origin_rpy, 3)) throw new Error(`Invalid joint orientation in ${side} profile`);
      if (joint.link_xyz !== undefined && !finiteArray(joint.link_xyz, 3)) throw new Error(`Invalid link vector in ${side} profile`);
    }
  }
  if (value.front_direction !== undefined && (!finiteArray(value.front_direction, 3)
      || Math.hypot(value.front_direction[0], value.front_direction[1]) === 0)) throw new Error('Front direction needs a horizontal component');
  if (value.ground_z !== undefined && !Number.isFinite(value.ground_z)) throw new Error('Invalid ground height');
  if (value.fixture !== undefined) {
    if (!value.fixture || !Array.isArray(value.fixture.rods)) throw new Error('Fixture requires a rods array');
    for (const member of value.fixture.rods) {
      if (!finiteArray(member.start_xyz, 3) || !finiteArray(member.end_xyz, 3)
          || !Number.isFinite(member.radius_m) || member.radius_m <= 0) throw new Error('Invalid fixture member');
    }
  }
  return value;
}

function setOriginRotation(object, rpy = [0, 0, 0]) {
  // URDF fixed-axis roll/pitch/yaw: Rz(yaw) * Ry(pitch) * Rx(roll).
  object.rotation.set(rpy[0], rpy[1], rpy[2], 'ZYX');
}

function labelSprite(text, color = '#dceaf4', width = .095) {
  const canvas = document.createElement('canvas');
  canvas.width = 384; canvas.height = 128;
  const context = canvas.getContext('2d');
  context.fillStyle = '#12232eee';
  context.beginPath(); context.roundRect(4, 8, 376, 112, 24); context.fill();
  context.strokeStyle = color; context.lineWidth = 4; context.stroke();
  context.fillStyle = color; context.font = '600 58px system-ui';
  context.textAlign = 'center'; context.textBaseline = 'middle'; context.fillText(text, 192, 67);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({map:texture, depthTest:false, depthWrite:false}));
  sprite.scale.set(width, width / 3, 1); sprite.renderOrder = 10;
  labels.push(sprite);
  return sprite;
}

function rod(parent, start, end, radius, material) {
  const delta = new THREE.Vector3().subVectors(end, start);
  if (delta.length() < .000001) return;
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, delta.length(), 16), material);
  mesh.position.copy(start).add(end).multiplyScalar(.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), delta.normalize());
  parent.add(mesh);
}

function positiveAxis(axis, color) {
  const group = new THREE.Group();
  group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axis);
  group.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, -.035), .105, color, .016, .009));
  const radius = .038, end = Math.PI * 1.5;
  const points = Array.from({length:33}, (_, i) => {
    const angle = .25 + (end - .25) * i / 32;
    return new THREE.Vector3(radius * Math.cos(angle), radius * Math.sin(angle), 0);
  });
  group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), new THREE.LineBasicMaterial({color, transparent:true, opacity:.8})));
  group.add(new THREE.ArrowHelper(new THREE.Vector3(-Math.sin(end), Math.cos(end), 0), points.at(-1), .013, color, .012, .009));
  axes.push(group);
  return group;
}

function makeArm(side, specification) {
  const color = new THREE.Color(specification.color || (side === 'left' ? '#52d7c5' : '#f4ad66'));
  const tint = `#${color.getHexString()}`;
  const group = new THREE.Group(); group.position.fromArray(specification.base_xyz);
  setOriginRotation(group, specification.base_rpy); scene.add(group);
  const accent = new THREE.MeshStandardMaterial({color, roughness:.43, metalness:.32});
  const metal = new THREE.MeshStandardMaterial({color:0x90a3b0, roughness:.48, metalness:.45});
  const dark = new THREE.MeshStandardMaterial({color:0x273e4d, roughness:.7});
  const overhead = specification.mount === 'overhead';
  const base = new THREE.Mesh(overhead ? new THREE.BoxGeometry(.11, .09, .023) : new THREE.CylinderGeometry(.075, .095, .023, 48), dark);
  if (!overhead) base.rotation.x = Math.PI / 2;
  base.position.z = overhead ? .012 : -.012; group.add(base);
  const baseLabel = labelSprite(specification.title || `${side} leader`, tint, .21);
  baseLabel.position.set(0, -.11, overhead ? .055 : -.022); group.add(baseLabel);
  let parent = group;
  const joints = [];
  specification.joints.forEach((joint, index) => {
    // Translation and fixed rotation locate the joint relative to the preceding link end.
    const origin = new THREE.Group(); origin.position.fromArray(joint.offset_xyz); parent.add(origin);
    setOriginRotation(origin, joint.origin_rpy);
    rod(parent, new THREE.Vector3(), origin.position, .009, dark);
    const motion = new THREE.Group(); origin.add(motion);
    const axis = new THREE.Vector3(...joint.axis).normalize();
    const hub = new THREE.Mesh(new THREE.SphereGeometry(index === 6 ? .015 : .018, 20, 12), accent);
    origin.add(hub, positiveAxis(axis, color));
    const label = labelSprite(index === 6 ? 'G7' : `J${index + 1}`, tint, .0945);
    label.position.set(.07, -.018, .02); origin.add(label);
    const link = new THREE.Vector3(...(joint.link_xyz ?? [0, 0, joint.length_m]));
    const length = link.length();
    const linkFrame = new THREE.Group();
    if (length > .000001) linkFrame.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), link.clone().normalize());
    motion.add(linkFrame);
    if (index < 6) {
      rod(motion, new THREE.Vector3(), link, .011, metal);
      if (length > .034) rod(linkFrame, new THREE.Vector3(.009, 0, .017), new THREE.Vector3(.009, 0, length - .017), .003, accent);
    } else {
      // Channel seven is a trigger hinge, not another full arm link or a measured jaw opening.
      const trigger = new THREE.Mesh(new THREE.BoxGeometry(.014, .019, Math.max(length, .001)), accent);
      trigger.position.set(.022, 0, length / 2); linkFrame.add(trigger);
      rod(linkFrame, new THREE.Vector3(), new THREE.Vector3(.022, 0, 0), .007, accent);
      const fixedGrip = new THREE.Group(); fixedGrip.quaternion.copy(linkFrame.quaternion); origin.add(fixedGrip);
      rod(fixedGrip, new THREE.Vector3(-.022, 0, -.015), new THREE.Vector3(-.022, 0, length * .8), .007, dark);
    }
    const end = new THREE.Group(); end.position.copy(link); motion.add(end); parent = end;
    joints.push({motion, axis});
  });
  const card = makeCard(side, specification, tint);
  arms.set(side, {group, joints, positions:Array(7).fill(0), ...card});
}

function makeCard(side, specification, tint) {
  const card = element('section', 'leader-card'); card.style.setProperty('--leader-color', tint);
  const header = element('header'), heading = element('div', 'leader-heading');
  const title = element('div'); title.append(element('h3', '', specification.title || side));
  const meta = element('p', 'leader-meta', `USB …${specification.usb_serial.slice(-8)}`);
  meta.title = specification.usb_serial; title.append(meta);
  heading.append(element('span', 'leader-dot'), title);
  const badge = element('span', 'leader-badge', 'Waiting'); header.append(heading, badge); card.append(header);
  const table = element('table', 'leader-table');
  const tableHead = element('thead'), headRow = element('tr');
  for (const label of ['Channel', 'Raw rad', 'Display °', 'Sign']) headRow.append(element('th', '', label));
  tableHead.append(headRow); table.append(tableHead);
  const body = element('tbody'), rows = [];
  specification.joints.forEach((joint, index) => {
    const row = element('tr'), name = element('td');
    name.append(element('span', 'channel-number', index === 6 ? 'G7' : String(index + 1)), element('span', 'channel-name', joint.label));
    const raw = element('td', '', '—'), relative = element('td', 'relative-angle', '—'), sign = element('td');
    const flip = element('button', 'direction-button', joint.direction > 0 ? '+ ↔' : '− ↔');
    flip.title = `Flip ${specification.title} ${joint.label} display direction`;
    flip.setAttribute('aria-label', flip.title); flip.disabled = true;
    flip.onclick = () => action('flip', side, index);
    sign.append(flip); row.append(name, raw, relative, sign); body.append(row);
    rows.push({raw, relative, flip});
  });
  table.append(body); card.append(table);
  const footer = element('div', 'leader-footer');
  const reference = element('button', '', 'Capture reference pose'); reference.disabled = true;
  reference.title = `Set ${specification.title}'s current readings as its display zero`;
  reference.onclick = () => action('reference', side);
  const telemetry = element('span', '', 'No readings yet'); footer.append(reference, telemetry); card.append(footer);
  $('leaders').append(card);
  return {rows, badge, telemetry, reference};
}

function setupScene() {
  renderer = new THREE.WebGLRenderer({antialias:true, alpha:true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2)); renderer.outputColorSpace = THREE.SRGBColorSpace;
  $('viewport').prepend(renderer.domElement);
  scene = new THREE.Scene(); camera = new THREE.PerspectiveCamera(42, 1, .001, 100);
  camera.up.set(0, 0, 1);
  orbit = new OrbitControls(camera, renderer.domElement); orbit.enableDamping = true; orbit.minDistance = .15; orbit.maxDistance = 12;
  scene.add(new THREE.HemisphereLight(0xd8eeff, 0x294350, 2.4));
  const key = new THREE.DirectionalLight(0xffffff, 2.8); key.position.set(-1, -2, 3); scene.add(key);
  const fill = new THREE.DirectionalLight(0x99cfff, 1.2); fill.position.set(2, 1, 2); scene.add(fill);
  const grid = new THREE.GridHelper(3, 30, 0x496476, 0x2a414f); grid.rotation.x = Math.PI / 2; grid.position.z = profile.ground_z ?? -.027; scene.add(grid);
  fixture = new THREE.Group(); scene.add(fixture);
  const fixtureMaterial = new THREE.MeshStandardMaterial({color:0x324551, roughness:.65, metalness:.3});
  for (const member of profile.fixture?.rods ?? []) {
    rod(fixture, new THREE.Vector3(...member.start_xyz), new THREE.Vector3(...member.end_xyz), member.radius_m, fixtureMaterial);
  }
  for (const side of sides) makeArm(side, profile.leaders[side]);
  resizeObserver = new ResizeObserver(() => {
    const width = $('viewport').clientWidth, height = $('viewport').clientHeight;
    renderer.setSize(width, height); camera.aspect = width / Math.max(1, height); camera.updateProjectionMatrix();
  });
  resizeObserver.observe($('viewport'));
  frame();
  $('fit').onclick = () => frame(); $('front').onclick = () => frame(viewDirection('front'));
  $('top').onclick = () => frame(viewDirection('top'));
  $('axes').onchange = () => axes.forEach(axis => { axis.visible = $('axes').checked; });
  $('labels').onchange = () => labels.forEach(label => { label.visible = $('labels').checked; });
  $('reference-guide').onchange = () => {
    for (const arm of arms.values()) applyPose(arm);
    updateHealth(); frame();
  };
  function animate() {
    if (fatal) return;
    requestAnimationFrame(animate); orbit.update(); renderer.render(scene, camera);
  }
  animate();
}

function viewDirection(mode = 'fit') {
  const front = new THREE.Vector3(...(profile.front_direction ?? [0, -1, 0]));
  front.z = 0; front.normalize();
  if (mode === 'front') return [front.x, front.y, .08];
  // A small approach from the configured front fixes the top view's screen rotation.
  if (mode === 'top') return [front.x * .001, front.y * .001, 1];
  const right = new THREE.Vector3().crossVectors(new THREE.Vector3(0, 0, 1), front);
  return front.multiplyScalar(2.5).add(right).add(new THREE.Vector3(0, 0, 1.3)).toArray();
}

function frame(direction = viewDirection()) {
  const bounds = new THREE.Box3();
  if (fixture) bounds.expandByObject(fixture);
  for (const arm of arms.values()) bounds.expandByObject(arm.group);
  const center = bounds.getCenter(new THREE.Vector3()), size = bounds.getSize(new THREE.Vector3());
  // Reserve room for the viewport overlays and the narrower desktop viewport.
  const extent = Math.max(size.z, size.y, size.x / Math.max(.5, camera.aspect), .4);
  camera.position.copy(center).add(new THREE.Vector3(...direction).normalize().multiplyScalar(extent * 2.4));
  orbit.target.copy(center); orbit.update();
}

function validLeader(value) {
  return value && Number.isFinite(value.sequence) && Number.isFinite(value.age_ms) && value.age_ms >= 0
    && finiteArray(value.raw_rad, 7) && finiteArray(value.positions_rad, 7)
    && Array.isArray(value.joints) && value.joints.length === 7;
}

function applyPose(arm) {
  arm.joints.forEach((joint, index) => {
    joint.motion.quaternion.setFromAxisAngle(joint.axis, $('reference-guide').checked ? 0 : arm.positions[index]);
  });
}

function acceptState(value) {
  state = value; receivedAt = performance.now(); transportError = '';
  deliveryCount++;
  if (receivedAt - deliveryWindowAt >= 1000) {
    deliveryHz = deliveryCount * 1000 / (receivedAt - deliveryWindowAt);
    deliveryCount = 0; deliveryWindowAt = receivedAt;
  }
  for (const side of sides) {
    const value = state.leaders?.[side];
    if (!validLeader(value)) continue;
    const previous = progress.get(side);
    if (!previous || previous.sequence !== value.sequence) progress.set(side, {sequence:value.sequence, at:receivedAt});
  }
  if (feedHealth().status === 'live') {
    for (const side of sides) {
      const value = state.leaders[side], arm = arms.get(side);
      arm.positions = [...value.positions_rad]; applyPose(arm);
      arm.joints.forEach((joint, index) => {
        const row = arm.rows[index], data = value.joints[index];
        row.raw.textContent = value.raw_rad[index].toFixed(3);
        row.raw.title = `Encoder ticks: ${value.raw_ticks?.[index] ?? 'unavailable'}`;
        row.relative.textContent = (value.positions_rad[index] * 180 / Math.PI).toFixed(1);
        row.relative.title = `${value.positions_rad[index].toFixed(4)} rad; reference ${Number(data.zero_rad).toFixed(4)} rad`;
        row.flip.textContent = data.direction > 0 ? '+ ↔' : '− ↔';
      });
    }
    if (!firstPoseFitted) { firstPoseFitted = true; requestAnimationFrame(() => frame()); }
  }
  updateHealth();
}

function feedHealth() {
  if (fatal) return {status:'fault', title:'Viewer unavailable', detail:'Reload the page to initialize the view again.'};
  if (transportError) return {status:'fault', title:'Feed interrupted · pose frozen', detail:transportError};
  if (!state) return {status:'waiting', title:'Waiting for leaders', detail:'The schematic is showing its reference shape.'};
  if (state.status === 'stale') return {status:'stale', title:'Readings stale · pose frozen', detail:state.error || 'Waiting for fresh leader readings.'};
  if (state.status !== 'live') return {status:state.status === 'fault' ? 'fault' : 'waiting', title:'Feed stopped · pose frozen', detail:state.error || 'Waiting for a complete pair of fresh leader readings.'};
  const now = performance.now();
  for (const side of sides) {
    const value = state.leaders?.[side], seen = progress.get(side);
    if (!validLeader(value)) return {status:'fault', title:'Incomplete readings · pose frozen', detail:`Missing or invalid ${side} leader channels.`};
    const sourceLimit = Number.isFinite(profile.max_age_s) ? profile.max_age_s * 1000 : staleMs;
    if (value.age_ms > sourceLimit || now - receivedAt > staleMs || !seen || now - seen.at > staleMs) {
      return {status:'stale', title:'Readings stale · pose frozen', detail:`${profile.leaders[side].title} has no fresh sample. Last verified pose is retained.`};
    }
  }
  if (Number.isFinite(profile.max_skew_s) && (!Number.isFinite(state.skew_ms) || state.skew_ms > profile.max_skew_s * 1000)) {
    return {status:'stale', title:'Leaders out of sync · pose frozen', detail:'Waiting for paired readings within the source timing limit.'};
  }
  const rate = state.rate_hz ?? profile.rate_hz;
  const inputRate = Number.isFinite(rate) ? `${rate} Hz input target` : 'Live encoder input';
  const displayRate = deliveryHz === null ? 'display starting' : `${Math.round(deliveryHz)} Hz display`;
  return {status:'live', title:sides.length === 2 ? 'Both leaders streaming' : `${sides.length} leader(s) streaming`, detail:`${inputRate} · ${displayRate} · pair skew ${Number(state.skew_ms).toFixed(1)} ms`};
}

function updateHealth() {
  const health = feedHealth(), live = health.status === 'live';
  document.body.dataset.feed = health.status;
  $('stream-title').textContent = health.title; $('stream-detail').textContent = health.detail;
  $('guide-status').hidden = !$('reference-guide').checked;
  $('guide-status').textContent = live ? 'Reference guide — live values continue in the tables' : 'Reference guide — tables retain the last readings';
  for (const side of sides) {
    const arm = arms.get(side); if (!arm) continue;
    const value = state?.leaders?.[side];
    arm.badge.textContent = live ? 'Streaming' : state ? 'Frozen' : 'Waiting'; arm.badge.dataset.live = String(live);
    arm.reference.disabled = busy || !live;
    arm.rows.forEach(row => { row.flip.disabled = busy || !profile || fatal; });
    if (validLeader(value)) {
      const age = Math.max(value.age_ms + performance.now() - receivedAt, performance.now() - (progress.get(side)?.at ?? receivedAt));
      arm.telemetry.textContent = `${Math.round(age)} ms old · #${value.sequence}`;
      arm.telemetry.title = value.joystick_present ? 'Joystick data present' : 'No joystick data in this sample';
    }
  }
  $('save').disabled = busy || !profile || fatal; $('reset').disabled = busy || !profile || fatal;
}

async function action(name, leader, channel) {
  if (busy || fatal) return;
  busy = true; actionGeneration++; updateHealth();
  $('action-status').dataset.error = 'false'; $('action-status').textContent = 'Updating display settings…';
  try {
    const payload = {action:name};
    if (leader !== undefined) payload.leader = leader;
    if (channel !== undefined) payload.channel = channel;
    const result = await request('action', {method:'POST', headers:{'Content-Type':'application/json', 'X-Leader-Token':profile.action_token}, body:JSON.stringify(payload)});
    if (result.state) acceptState(result.state);
    profile = validateProfile(await request('profile'));
    const title = leader ? profile.leaders[leader].title : 'Both leaders';
    $('action-status').textContent = {
      reference:`${title}: current pose captured as display zero. Servo zeros unchanged.`,
      flip:`${title}: channel ${channel + 1} display direction flipped.`,
      reset:'Display mapping reset to its configured defaults.',
      save:'Display settings saved. No motor calibration was changed.',
    }[name];
  } catch (error) {
    $('action-status').dataset.error = 'true'; $('action-status').textContent = error.message;
  } finally { busy = false; updateHealth(); }
}

async function poll() {
  if (fatal) return;
  const startedAt = performance.now();
  const generation = actionGeneration;
  try {
    const value = await request('state');
    // A response issued before an edit must not overwrite the edited display pose.
    if (!busy && generation === actionGeneration) acceptState(value);
  } catch (error) {
    transportError = error.message; updateHealth();
  } finally {
    // One request in flight; USB sampling continues independently in the server.
    if (!fatal) setTimeout(poll, Math.max(0, pollIntervalMs - (performance.now() - startedAt)));
  }
}

function showFatal(error) {
  fatal = true; $('render-error').hidden = false;
  $('render-error').textContent = `Unable to initialize the leader viewer: ${error.message}`;
  updateHealth();
}

window.addEventListener('error', event => showFatal(event.error || new Error(event.message)));
window.addEventListener('unhandledrejection', event => showFatal(event.reason instanceof Error ? event.reason : new Error(String(event.reason))));
$('reset').onclick = () => action('reset'); $('save').onclick = () => action('save');
try {
  profile = validateProfile(await request('profile'));
  sides = Object.keys(profile.leaders);
  $('profile-id').textContent = `Profile: ${profile.id}`;
  $('reference-description').textContent = profile.reference_pose_description || 'Match the configured reference shape.';
  setupScene();
  $('action-status').textContent = 'Reference and direction edits affect this display only.';
  setInterval(updateHealth, 50);
  poll();
} catch (error) { showFatal(error); }
