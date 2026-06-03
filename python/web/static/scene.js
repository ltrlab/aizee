// three.js scene for the in-headset view.
//
// Phase 1: stub — empty scene with hand reticles.
// Phase 3: addCameraPanel() draws the gripper-cam JPEG on a floating plane.
// Phase 4: updateURDF() drives a URDFRobot from telem.qpos.
//
// Kept in one file so all the scene mutation goes through one set of
// helpers; main.js never touches the THREE.Scene directly except by
// passing it back here.

import * as THREE from 'three';

const _camState = {
  texture: null,    // THREE.VideoTexture wrapping the WebRTC <video> element
  mesh: null,       // THREE.Mesh — floating cam panel
  placeholder: null, // THREE.Mesh — "waiting for camera" card shown when stale
};

const _reticles = {
  right: null,
  left: null,
};

const _urdfState = {
  robot: null,            // THREE.Object3D root of the SOLID URDF — actual qpos
  jointMap: null,         // {name -> Object3D} for fast qpos -> rotation
  lastTelemTs: 0,
  targetQ: null,           // most recent qpos received from telem
  displayedQ: null,        // currently-rendered qpos; eased toward targetQ
  // Ghost URDF — same model, translucent yellow, driven by telem.qcmd.
  // Lets the operator see commanded-vs-actual divergence at a glance:
  // when the real arm keeps up, the ghost overlaps the solid; when the
  // arm lags (motor saturation, collision, slow drives), the ghost
  // floats ahead of the solid.
  ghost: null,
  ghostJointMap: null,
  ghostTargetQ: null,
  ghostDisplayedQ: null,
  lastFrameAt: 0,
};

// Layer assignment so the kinematic-preview window renders ONLY the URDF
// and its markers — not the HUD, hand reticles, fingertip cursor, VR UI,
// or camera panel.  Items on _PREVIEW_LAYER show up in the preview render
// target; other items stay on layer 0 (the default) which the main XR
// camera always sees.  URDF + markers get enabled on BOTH layers.
const _PREVIEW_LAYER = 1;

const _previewState = {
  rt: null,        // THREE.WebGLRenderTarget — the off-screen render target
  cam: null,       // THREE.PerspectiveCamera — fixed view of the URDF
  panel: null,     // THREE.Mesh — flat plane in main scene showing the texture
  enabled: true,   // toggled from the VR UI; OFF stops the 2nd scene render
  frameCounter: 0,
  everyN: 6,       // render the RT every Nth XR frame -> ~15 Hz at 90 Hz
};

// Per-joint visual offsets [rad] for the URDF mesh.  Signs already
// applied at the Python boundary (control-relevant) so qpos/qcmd are in
// URDF *control* frame; offsets are purely a visual correction for when
// the motor's mechanical zero doesn't visually match the URDF neutral
// pose.  Applied here on display only — they DO NOT affect anything sent
// to the motor.  Source: telem.align_offsets (live), with /api/joint_align
// as a fallback before the first telem frame arrives.
let _jointOffsets = [0, 0, 0, 0, 0, 0, 0];
export function setJointOffsetsFromTelem(arr) {
  if (Array.isArray(arr) && arr.length >= 7) {
    _jointOffsets = arr.slice(0, 7).map(Number);
  }
}
async function _loadJointOffsets() {
  try {
    const r = await fetch('/api/joint_align');
    if (r.ok) {
      const d = await r.json();
      if (Array.isArray(d.offsets)) _jointOffsets = d.offsets.slice(0, 7).map(Number);
    }
  } catch {}
}

// Ghost = the translucent REAL-arm overlay (telem.qpos), shown only when
// it diverges from the commanded pose.  Default ON so the operator can see
// where the real arm actually is vs. where they're commanding it.  Toggle
// OFF from the VR UI if the extra mesh render costs too much framerate.
let _ghostEnabled = true;

export function setPreviewEnabled(on) {
  _previewState.enabled = !!on;
  if (_previewState.panel) _previewState.panel.visible = !!on;
}
export function setGhostEnabled(on) {
  _ghostEnabled = !!on;
  if (_urdfState.ghost)     _urdfState.ghost.visible     = _ghostEnabled;
  if (_urdfState.ghostLine) _urdfState.ghostLine.visible = _ghostEnabled;
}

// Reposition the whole robot group (URDF + workspace box + markers, which
// are children of the URDF root) plus the ghost.  Used by the in-VR
// "Move robot" grab so the operator can place the virtual arm wherever is
// comfortable — purely cosmetic, doesn't affect IK or recording.
//
// Yaw is tracked separately because the URDF root carries a fixed -π/2
// rotation around local X (URDF Z-up → world Y-up); composing world-Y yaw
// with that base tilt has to be done as a quaternion pre-multiplication
// rather than just setting rotation.y.
let _robotYaw = 0;
const _ROBOT_BASE_QUAT = new THREE.Quaternion()
  .setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);

export function getRobotGroupPosition() {
  return _urdfState.robot ? _urdfState.robot.position.clone() : null;
}
export function getRobotGroupYaw() {
  return _urdfState.robot ? _robotYaw : null;
}
export function nudgeRobotGroup(worldPosArray, grabOffsetArray, yaw = null) {
  if (!_urdfState.robot) return;
  _urdfState.robot.position.set(
    worldPosArray[0] + grabOffsetArray[0],
    worldPosArray[1] + grabOffsetArray[1],
    worldPosArray[2] + grabOffsetArray[2],
  );
  if (yaw !== null) {
    _robotYaw = yaw;
    const qy = new THREE.Quaternion()
      .setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
    _urdfState.robot.quaternion.copy(qy).multiply(_ROBOT_BASE_QUAT);
    if (_urdfState.ghost) _urdfState.ghost.quaternion.copy(_urdfState.robot.quaternion);
  }
  if (_urdfState.ghost) _urdfState.ghost.position.copy(_urdfState.robot.position);
}

const _hudState = {
  panel: null,            // THREE.Mesh — floating HUD card
  texture: null,          // CanvasTexture backing the panel
  canvas: null,
  ctx: null,
  lastDrawn: '',
  workspaceBox: null,     // THREE.LineSegments — operator clutch clamp (thick green)
  reachableBox: null,     // THREE.LineSegments — IK hard reach AABB (thin gray)
  targetMarker: null,     // THREE.Mesh — sphere at the IK target (clamped) pose
  engageMarker: null,     // THREE.Mesh — sphere at engage_ee (clutch anchor)
  clampLine: null,        // THREE.Line — engage_ee -> target_ee_raw
  fkMarker: null,         // THREE.Mesh — blue sphere at Python FK(qpos): if
                          //   it doesn't overlap the URDF-rendered EE link,
                          //   the Python kinematics chain disagrees with
                          //   the URDF (axis sign, origin, or EE-link choice).
};

// Stash references so setPassthroughMode can toggle them without
// rebuilding the scene.
const _sceneRefs = { scene: null, grid: null };

/**
 * Toggle between VR (solid background, floor grid visible) and AR
 * passthrough (transparent background, no grid so the real room shows
 * through).  Called from main.js right before requesting the XR session.
 */
export function setPassthroughMode(scene, renderer, on) {
  if (on) {
    scene.background = null;
    renderer.setClearColor(0x000000, 0);
    if (_sceneRefs.grid) _sceneRefs.grid.visible = false;
  } else {
    scene.background = new THREE.Color(0x0a0d12);
    renderer.setClearColor(0x0a0d12, 1);
    if (_sceneRefs.grid) _sceneRefs.grid.visible = true;
  }
}

export function buildScene(renderer) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0d12);
  _sceneRefs.scene = scene;
  _initPreviewPanel(scene);
  // The XR camera is auto-managed by renderer.xr; this fallback camera
  // is only used outside an XR session.
  const camera = new THREE.PerspectiveCamera(75, 1, 0.01, 50);
  camera.position.set(0, 1.6, 0.5);

  // Floor grid for spatial reference when not in passthrough.
  const grid = new THREE.GridHelper(4, 16, 0x303642, 0x1a1f29);
  grid.position.y = 0;
  scene.add(grid);
  _sceneRefs.grid = grid;

  // Hemisphere light so we don't render pure black before any objects.
  const hemi = new THREE.HemisphereLight(0xb1c2d8, 0x202830, 1.0);
  // The kinematic-preview camera renders only layer 1 — so the URDF
  // (which uses MeshStandardMaterial) would otherwise be black inside
  // the preview panel.  Enable the light on both layers.
  hemi.layers.enable(_PREVIEW_LAYER);
  scene.add(hemi);

  // Reticles
  _reticles.right = _makeReticle(0xff5555);
  _reticles.left  = _makeReticle(0x5599ff);
  _reticles.right.visible = false;
  _reticles.left.visible = false;
  scene.add(_reticles.right, _reticles.left);

  return { scene, camera };
}

function _makeReticle(color) {
  // Small wireframe sphere with an arrow pointing along -Z to show heading.
  const g = new THREE.Group();
  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(0.02, 12, 8),
    new THREE.MeshBasicMaterial({ color, wireframe: true }),
  );
  const arrow = new THREE.Mesh(
    new THREE.ConeGeometry(0.012, 0.05, 12),
    new THREE.MeshBasicMaterial({ color }),
  );
  arrow.rotation.x = Math.PI / 2;
  arrow.position.z = -0.04;
  g.add(sphere, arrow);
  return g;
}

export function updateReticles(_scene, right, left) {
  if (right && _reticles.right) {
    _reticles.right.position.fromArray(right.pos);
    _reticles.right.quaternion.fromArray(right.quat);
    _reticles.right.visible = true;
  } else if (_reticles.right) {
    _reticles.right.visible = false;
  }
  if (left && _reticles.left) {
    _reticles.left.position.fromArray(left.pos);
    _reticles.left.quaternion.fromArray(left.quat);
    _reticles.left.visible = true;
  } else if (_reticles.left) {
    _reticles.left.visible = false;
  }
}

// ---- Camera panel (WebRTC video track) ------------------------------------

export function attachCameraVideo(scene, videoEl) {
  // Idempotent — only build the panel on first call.  THREE.VideoTexture
  // pulls from the <video> element each frame; no manual decode loop.
  if (_camState.mesh) return;
  if (!videoEl) return;
  const tex = new THREE.VideoTexture(videoEl);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.minFilter = THREE.LinearFilter;
  tex.magFilter = THREE.LinearFilter;
  _camState.texture = tex;
  // 1024x768 source -> 4:3 panel, 1.2 m wide, 1 m in front at eye height.
  const w = 1.2, h = w * 768 / 1024;
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(w, h),
    new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide }),
  );
  mesh.position.set(0, 1.55, -1.0);
  mesh.visible = false;   // hidden until a fresh camera frame is confirmed
  scene.add(mesh);
  _camState.mesh = mesh;

  // "Waiting for camera" placeholder shown in place of the panel when the
  // gripper cam is stale (otherwise the WebRTC black-frame fallback shows
  // as a big black square).  A small text card, not a full panel.
  const ph = document.createElement('canvas');
  ph.width = 512; ph.height = 128;
  const pctx = ph.getContext('2d');
  pctx.fillStyle = 'rgba(22,27,34,0.85)';
  _roundRectCtx(pctx, 0, 0, 512, 128, 16); pctx.fill();
  pctx.fillStyle = '#7d8590';
  pctx.font = '600 30px system-ui, sans-serif';
  pctx.textAlign = 'center'; pctx.textBaseline = 'middle';
  pctx.fillText('waiting for gripper camera…', 256, 64);
  const phTex = new THREE.CanvasTexture(ph);
  phTex.colorSpace = THREE.SRGBColorSpace;
  const phMesh = new THREE.Mesh(
    new THREE.PlaneGeometry(0.5, 0.125),
    new THREE.MeshBasicMaterial({ map: phTex, transparent: true }),
  );
  phMesh.position.set(0, 1.55, -1.0);
  scene.add(phMesh);
  _camState.placeholder = phMesh;
}

function _roundRectCtx(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

// Called from updateURDF with telem.cam_age — flips between the live cam
// panel and the "waiting" placeholder based on camera freshness.
export function setCameraFresh(fresh) {
  if (_camState.mesh) _camState.mesh.visible = fresh;
  if (_camState.placeholder) _camState.placeholder.visible = !fresh;
}

// ---- URDF mirror (Phase 4) -------------------------------------------------

// URDF joint names in AIZEE control-vector order — used to fan qpos[]
// into per-joint angles for the URDF mirror.  The 6th slot is `wrist_swivel`
// (URDF name) not `wrist_roll` (firmware name); they describe the same
// physical joint.  `gripper` is missing from the current URDF (no
// dof_gripper mate in OnShape); the lookup gracefully skips it.
const AIZEE_JOINT_NAMES = [
  'swivel', 'gantry_base', 'gantry_mid', 'gantry_end',
  'wrist_pitch', 'wrist_swivel', 'gripper',
];

// Lazy-loaded so the import doesn't block the lobby load on slow LANs.
let _URDFLoaderPromise = null;
async function _getURDFLoader() {
  if (!_URDFLoaderPromise) {
    _URDFLoaderPromise = import('https://cdn.jsdelivr.net/npm/urdf-loader@0.12.6/src/URDFLoader.js')
      .then(m => m.default ?? m.URDFLoader ?? m);
  }
  return _URDFLoaderPromise;
}

async function _ensureRobot(scene) {
  if (_urdfState.robot || _urdfState.loading) return;
  _urdfState.loading = true;
  _loadJointOffsets();   // fire-and-forget; telem.align_offsets supersedes
  try {
    const URDFLoader = await _getURDFLoader();
    const loader = new URDFLoader();
    // The URDF references meshes as "package://meshes\foo.stl" — we serve
    // them from /meshes/ on the host, with backslash -> slash normalisation.
    loader.packages = { meshes: '/meshes' };
    loader.loadMeshCb = (path, manager, onComplete) => {
      // Lazy STL loader — only imported the first time it's needed so
      // the rest of the scene works even without meshes available.
      import('three/addons/loaders/STLLoader.js').then(({ STLLoader }) => {
        const stl = new STLLoader(manager);
        const fixed = path.replace(/\\/g, '/');
        stl.load(fixed, (geom) => {
          const mat = new THREE.MeshStandardMaterial({
            color: 0xb1c2d8, metalness: 0.15, roughness: 0.7,
          });
          onComplete(new THREE.Mesh(geom, mat));
        }, undefined, (err) => {
          // Mesh missing — render a visible placeholder (5 cm box) so the
          // operator can still SEE the joint chain instead of guessing why
          // their URDF mirror appears empty.  Switch back to actual STLs by
          // dropping the .stl files into urdf/aizee/meshes/.
          console.warn('[urdf] mesh missing:', fixed);
          const ph = new THREE.Mesh(
            new THREE.BoxGeometry(0.05, 0.05, 0.05),
            new THREE.MeshBasicMaterial({ color: 0x556070, wireframe: true }),
          );
          onComplete(ph);
        });
      });
    };
    const res = await fetch('/aizee.urdf');
    if (!res.ok) {
      console.warn('[urdf] could not fetch /aizee.urdf; URDF mirror disabled');
      return;
    }
    // Normalize Windows-style backslashes in mesh paths — see preview.html
    // for the same fix.  urdf-loader's resolvePath splits package URIs on
    // forward slashes and silently drops anything with a backslash.
    const urdfText = (await res.text()).replace(
      /(filename="[^"]*)/g, (m) => m.replace(/\\/g, '/'),
    );
    const robot = loader.parse(urdfText);
    robot.scale.set(1, 1, 1);
    // URDF spec is Z-up, WebXR scene is Y-up.  Rotate -90° about X so
    // URDF +Z (robot up) maps to scene +Y (world up).  After this rotation
    // the rover body's bottom mesh (at URDF z=-0.167) sits at scene
    // y=-0.167 in robot-local frame, so we offset robot.position.y by the
    // same amount to plant it on the floor (y=0).
    robot.rotation.x = -Math.PI / 2;
    robot.position.set(0.8, 0.167, -0.6);  // off to the right of the operator
    scene.add(robot);
    // Tag the URDF tree so it also renders in the kinematic-preview window.
    robot.traverse((o) => o.layers.enable(_PREVIEW_LAYER));

    // GREEN SKELETON overlay — a SECOND parsed URDF (so FK runs independently
    // on telem.qpos) but with the meshes SUPPRESSED.  We hang a small green
    // sphere on each link and connect them with a green line.  This shows
    // the real arm's pose for divergence diagnostics at a tiny fraction of
    // the cost of a full mesh re-render (which choked the Quest GPU when
    // both arms were drawn simultaneously).
    const ghostLoader = new URDFLoader();
    ghostLoader.packages = { meshes: '/meshes' };
    ghostLoader.loadMeshCb = (_p, _m, done) => done(new THREE.Object3D());  // empty
    const ghost = ghostLoader.parse(urdfText);
    ghost.scale.copy(robot.scale);
    ghost.rotation.copy(robot.rotation);
    ghost.position.copy(robot.position);
    // Joint spheres: one bright green sphere parented to each URDF link
    // so it follows the link's FK transform automatically.
    const ghostMat = new THREE.MeshBasicMaterial({
      color: 0x22cc44, transparent: true, opacity: 0.9, depthTest: false,
    });
    const ghostSphereGeo = new THREE.SphereGeometry(0.020, 10, 8);
    for (const linkName in (ghost.links || {})) {
      const link = ghost.links[linkName];
      const s = new THREE.Mesh(ghostSphereGeo, ghostMat);
      s.renderOrder = 6;
      s.layers.enable(_PREVIEW_LAYER);
      link.add(s);
    }
    // Connecting line through the arm chain.  Built once with a dynamic
    // geometry; per-frame world positions are pushed in by animateURDF().
    const _chainNames = [
      'tophalfbase', 'part_40', 'part_22', 'part_33', 'part_42',
      '1_1_02_eb571_960_stp', '1_1_02_eb571_960_stp_2',
    ];
    const _chainLinks = _chainNames
      .map((n) => ghost.links?.[n])
      .filter(Boolean);
    const lineGeom = new THREE.BufferGeometry();
    lineGeom.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array(_chainLinks.length * 3), 3),
    );
    const ghostLine = new THREE.Line(
      lineGeom,
      new THREE.LineBasicMaterial({ color: 0x22cc44, linewidth: 2, depthTest: false }),
    );
    ghostLine.frustumCulled = false;
    ghostLine.renderOrder = 6;
    ghostLine.layers.enable(_PREVIEW_LAYER);
    scene.add(ghostLine);   // line lives in scene (world coords) — see animateURDF
    ghost.visible = _ghostEnabled;
    scene.add(ghost);
    ghost.traverse((o) => o.layers.enable(_PREVIEW_LAYER));
    _urdfState.ghost = ghost;
    _urdfState.ghostLine = ghostLine;
    _urdfState.ghostChainLinks = _chainLinks;
    _urdfState.ghostJointMap = {};
    for (const name of AIZEE_JOINT_NAMES) {
      const j = ghost.joints?.[name];
      if (j) _urdfState.ghostJointMap[name] = j;
    }
    _urdfState.robot = robot;
    _urdfState.jointMap = {};
    for (const name of AIZEE_JOINT_NAMES) {
      const j = robot.joints?.[name];
      if (j) _urdfState.jointMap[name] = j;
    }
  } catch (e) {
    console.warn('[urdf] load failed:', e);
  } finally {
    _urdfState.loading = false;
  }
}

export function updateURDF(scene, telem) {
  if (!telem || !Array.isArray(telem.qpos)) return;
  // Trigger lazy-load on first telem frame; subsequent frames just update.
  if (!_urdfState.robot) { _ensureRobot(scene); return; }
  // Snap the TARGET, not the displayed value — animateURDF() does the
  // per-render-frame easing so a missed telem (browser GC, network blip)
  // doesn't show as a position jump.  telem.qpos / qcmd are URDF
  // *control* frame (signs applied at the Python boundary).  Visual
  // offsets are applied here only — they're cosmetic, not control.
  // MAIN mirror shows the COMMANDED pose (telem.qcmd) so it follows the
  // operator's hand in real time — even when motors are disabled and the
  // real arm (telem.qpos) is static.  Falls back to qpos if no command is
  // available (e.g. no leader installed yet).
  if (Array.isArray(telem.align_offsets)) setJointOffsetsFromTelem(telem.align_offsets);
  const mainQ = Array.isArray(telem.qcmd) ? telem.qcmd : telem.qpos;
  _urdfState.targetQ = mainQ.map((v, i) => v + (_jointOffsets[i] || 0));
  _urdfState.lastTelemTs = (typeof performance !== 'undefined' && performance.now)
    ? performance.now() : Date.now();
  // Camera freshness: hide the cam panel (and show the placeholder) when
  // the gripper cam hasn't produced a frame recently — avoids the big
  // black square from WebRTC's black-frame fallback.  cam_age in seconds;
  // > 1 s = stale.  Absent (sim mode) => treat as fresh.
  if (typeof telem.cam_age === 'number') {
    setCameraFresh(telem.cam_age < 1.0);
  }
  // GREEN overlay = the REAL arm (telem.qpos).  Shown whenever enabled so
  // the operator can always see where the real arm is vs. the commanded
  // (grey) arm — when they coincide it's a green tint; when they diverge
  // (lag / saturation / motors off) the green arm separates visibly.
  if (Array.isArray(telem.qpos)) {
    _urdfState.ghostTargetQ = telem.qpos.map((v, i) => v + (_jointOffsets[i] || 0));
    if (_urdfState.ghost) _urdfState.ghost.visible = _ghostEnabled;
  }
  // Leader HUD: workspace box + status pills + IK markers.  Comes through
  // whenever the installed leader is QuestLeader (the host's telem mirror
  // adds telem.leader.*).
  if (telem.leader) {
    _ensureWorkspaceBox(scene, telem.leader);
    _ensureHudPanel(scene);
    _updateHud(telem);
    _updateIkMarkers(telem.leader);
  }
}

// -----------------------------------------------------------------------
// Kinematic preview window — render-to-texture panel that mirrors the
// URDF + markers in a contained 2D flat panel inside the VR scene.  Built
// at session start; the main loop calls renderPreviewPanel() before each
// XR frame so the texture is fresh when the main camera composites it.
// -----------------------------------------------------------------------

function _initPreviewPanel(scene) {
  if (_previewState.rt) return;
  // Modest render target — the panel is small in view, and a full re-render
  // of the high-poly URDF is the single most expensive thing we do, so keep
  // the pixel count (and thus fill cost) down.
  const W = 480, H = 360;
  _previewState.rt = new THREE.WebGLRenderTarget(W, H, {
    depthBuffer: true,
    type: THREE.UnsignedByteType,
  });
  // Camera positioned 1 m to the URDF's forward+up, looking back at it.
  // URDF root sits at (0.8, 0.167, -0.6) in scene coords after rotation.
  const cam = new THREE.PerspectiveCamera(40, W / H, 0.05, 8.0);
  cam.up.set(0, 1, 0);
  cam.position.set(1.9, 0.7, 0.1);
  cam.lookAt(0.8, 0.4, -0.6);
  cam.layers.disableAll();
  cam.layers.enable(_PREVIEW_LAYER);   // sees URDF + markers ONLY
  _previewState.cam = cam;

  // Display the texture on a flat panel in the main scene.  Positioned to
  // the operator's UPPER-RIGHT so it doesn't conflict with the camera panel
  // (which sits center-forward) or the HUD panel (upper-left).
  const aspect = W / H;
  const pw = 0.32, ph = pw / aspect;     // ~32 cm wide
  const panel = new THREE.Mesh(
    new THREE.PlaneGeometry(pw, ph),
    new THREE.MeshBasicMaterial({ map: _previewState.rt.texture, side: THREE.DoubleSide }),
  );
  panel.position.set(0.55, 1.85, -0.95);
  panel.rotation.y = -Math.PI / 7;
  scene.add(panel);

  // Subtle dark frame behind the panel so the texture's edges are visible
  // against bright backgrounds (especially in passthrough/AR mode).
  const frame = new THREE.Mesh(
    new THREE.PlaneGeometry(pw + 0.012, ph + 0.012),
    new THREE.MeshBasicMaterial({ color: 0x161b22, transparent: true, opacity: 0.7 }),
  );
  frame.position.set(0.55, 1.85, -0.952);  // 2 mm behind the panel
  frame.rotation.y = panel.rotation.y;
  scene.add(frame);

  _previewState.panel = panel;
}

/** Render the URDF + markers to the preview render target.  Called once
 *  per XR frame from main.js BEFORE the main camera render.  Throttled to
 *  ~15 Hz so the expensive 2nd full-scene render doesn't tank the 90 Hz
 *  main view; the panel texture just refreshes a bit slower. */
export function renderPreviewPanel(renderer, scene) {
  if (!_previewState.enabled) return;
  if (!_previewState.rt || !_previewState.cam) return;
  if (!_urdfState.robot) return;  // nothing to render yet
  if ((_previewState.frameCounter++ % _previewState.everyN) !== 0) return;
  // Keep the robot framed even if the operator has grabbed and moved it —
  // aim the preview camera at the robot's current world position.
  if (_urdfState.robot) {
    const c = new THREE.Vector3();
    _urdfState.robot.getWorldPosition(c);
    c.y += 0.25;  // look at roughly mid-arm height, not the base
    _previewState.cam.position.set(c.x + 1.1, c.y + 0.3, c.z + 0.7);
    _previewState.cam.lookAt(c);
  }
  const prevTarget = renderer.getRenderTarget();
  renderer.setRenderTarget(_previewState.rt);
  // Clear with a dark background so the panel reads as a "screen".
  renderer.setClearColor(0x0a0d12, 1);
  renderer.clear();
  renderer.render(scene, _previewState.cam);
  renderer.setRenderTarget(prevTarget);
  // Restore clear color so we don't disturb the main XR render.
  renderer.setClearColor(0x000000, 0);
}

/**
 * Per-render-frame eased URDF update.  Should be called from main.js's XR
 * animation loop AFTER updateReticles().  Eases displayedQ toward targetQ
 * at a velocity bound that's high enough to track normal arm motion
 * (~4 rad/s) but low enough to smooth out telem-rate hitches.
 */
const _JOINT_MAX_DISPLAY_RAD_PER_S = 8.0;   // generous so normal motion never lags

export function animateURDF() {
  if (!_urdfState.robot || !_urdfState.targetQ) return;
  const now = (typeof performance !== 'undefined' && performance.now)
    ? performance.now() : Date.now();
  const dt = Math.min((now - (_urdfState.lastFrameAt || now)) / 1000, 0.1);
  _urdfState.lastFrameAt = now;
  const maxStep = _JOINT_MAX_DISPLAY_RAD_PER_S * dt;
  _animateOne(_urdfState.targetQ, _urdfState.jointMap,
              (q) => _urdfState.displayedQ = q,
              _urdfState.displayedQ, maxStep);
  if (_urdfState.ghost && _urdfState.ghostTargetQ) {
    _animateOne(_urdfState.ghostTargetQ, _urdfState.ghostJointMap,
                (q) => _urdfState.ghostDisplayedQ = q,
                _urdfState.ghostDisplayedQ, maxStep);
    // Refresh the skeleton line from the (now-updated) ghost link world
    // positions.  Cheap: 7 getWorldPosition calls + a 21-float upload.
    _updateGhostSkeletonLine();
  }
}

const _tmpV3 = new THREE.Vector3();
function _updateGhostSkeletonLine() {
  const line = _urdfState.ghostLine;
  const chain = _urdfState.ghostChainLinks;
  if (!line || !chain || !line.visible) return;
  const posAttr = line.geometry.getAttribute('position');
  for (let i = 0; i < chain.length; i++) {
    chain[i].getWorldPosition(_tmpV3);
    posAttr.setXYZ(i, _tmpV3.x, _tmpV3.y, _tmpV3.z);
  }
  posAttr.needsUpdate = true;
}

function _animateOne(targetQ, jointMap, setDisplayed, displayedQ, maxStep) {
  if (!targetQ || !jointMap) return;
  if (!displayedQ) displayedQ = targetQ.slice();
  for (let i = 0; i < AIZEE_JOINT_NAMES.length && i < targetQ.length; i++) {
    const joint = jointMap[AIZEE_JOINT_NAMES[i]];
    if (!joint || typeof joint.setJointValue !== 'function') continue;
    const tgt = targetQ[i];
    const cur = displayedQ[i] ?? tgt;
    let next;
    const delta = tgt - cur;
    if (Math.abs(delta) <= maxStep) next = tgt;
    else next = cur + Math.sign(delta) * maxStep;
    displayedQ[i] = next;
    joint.setJointValue(next);
  }
  setDisplayed(displayedQ);
}

// ---- Workspace box + HUD ---------------------------------------------------

function _buildBoxWireframe(min, max, color, opacity) {
  const w = max[0] - min[0], d = max[1] - min[1], h = max[2] - min[2];
  const cx = (max[0] + min[0]) / 2;
  const cy = (max[1] + min[1]) / 2;
  const cz = (max[2] + min[2]) / 2;
  const box = new THREE.BoxGeometry(w, d, h);
  const edges = new THREE.EdgesGeometry(box);
  box.dispose();
  const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity });
  const lines = new THREE.LineSegments(edges, mat);
  lines.position.set(cx, cy, cz);
  lines.userData.key = min.concat(max).join(',');
  lines.layers.enable(_PREVIEW_LAYER);
  return lines;
}

function _ensureWorkspaceBox(scene, leader) {
  if (!_urdfState.robot) return;
  // Operator clutch clamp — thick green wireframe, prominent.
  const wmin = leader.workspace_min, wmax = leader.workspace_max;
  if (Array.isArray(wmin) && Array.isArray(wmax)) {
    const key = wmin.concat(wmax).join(',');
    if (!_hudState.workspaceBox || _hudState.workspaceBox.userData.key !== key) {
      if (_hudState.workspaceBox) {
        _urdfState.robot.remove(_hudState.workspaceBox);
        _hudState.workspaceBox.geometry.dispose();
      }
      _hudState.workspaceBox = _buildBoxWireframe(wmin, wmax, 0x2ea043, 0.60);
      _urdfState.robot.add(_hudState.workspaceBox);
    }
  }
  // Reachable AABB — thin faded white, draws OUTSIDE the workspace.  Shows
  // where the arm physically cannot reach even if the operator pushed.
  const rmin = leader.reachable_min, rmax = leader.reachable_max;
  if (Array.isArray(rmin) && Array.isArray(rmax)) {
    const key = rmin.concat(rmax).join(',');
    if (!_hudState.reachableBox || _hudState.reachableBox.userData.key !== key) {
      if (_hudState.reachableBox) {
        _urdfState.robot.remove(_hudState.reachableBox);
        _hudState.reachableBox.geometry.dispose();
      }
      _hudState.reachableBox = _buildBoxWireframe(rmin, rmax, 0xb1c2d8, 0.22);
      _urdfState.robot.add(_hudState.reachableBox);
    }
  }
}

function _ensureHudPanel(scene) {
  if (_hudState.panel) return;
  const canvas = document.createElement('canvas');
  canvas.width = 512; canvas.height = 220;
  const ctx = canvas.getContext('2d');
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const w = 0.6, h = w * canvas.height / canvas.width;
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(w, h),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true }),
  );
  mesh.position.set(-0.55, 1.85, -1.0);
  mesh.rotation.y = Math.PI / 14;
  scene.add(mesh);
  _hudState.panel = mesh;
  _hudState.canvas = canvas;
  _hudState.ctx = ctx;
  _hudState.texture = tex;
}

function _updateHud(telem) {
  if (!_hudState.ctx) return;
  const leader = telem.leader ?? {};
  const engaged = !!leader.engaged;
  const estop = !!leader.estop;
  // Stringify to detect "no change" and skip the redraw + GPU upload.
  const key = `${engaged}|${estop}|${telem.ts ?? 0}`;
  if (_hudState.lastDrawn === key) return;
  _hudState.lastDrawn = key;
  const ctx = _hudState.ctx;
  const w = _hudState.canvas.width, h = _hudState.canvas.height;
  ctx.clearRect(0, 0, w, h);
  // Background card
  ctx.fillStyle = 'rgba(22, 27, 34, 0.85)';
  _roundRect(ctx, 0, 0, w, h, 18);
  ctx.fill();
  ctx.fillStyle = '#e6edf3';
  ctx.font = '600 22px system-ui, sans-serif';
  ctx.fillText('AIZEE  · VR teleop', 22, 38);
  // Pills
  const pill = (x, label, on, color) => {
    ctx.fillStyle = on ? color : '#30363d';
    _roundRect(ctx, x, 60, 200, 44, 10);
    ctx.fill();
    ctx.fillStyle = on ? '#0c0f14' : '#7d8590';
    ctx.font = '600 18px system-ui, sans-serif';
    ctx.fillText(label, x + 18, 89);
  };
  pill(22,  engaged ? 'CLUTCH: ENGAGED' : 'clutch: idle', engaged, '#2ea043');
  pill(238, estop   ? 'E-STOP LATCHED'  : 'e-stop: clear', estop,   '#f85149');
  // Tiny qpos summary
  ctx.fillStyle = '#7d8590';
  ctx.font = '500 16px ui-monospace, monospace';
  const q = telem.qpos ?? [];
  const summary = q.slice(0, 7).map(v => v.toFixed(2)).join('  ');
  ctx.fillText(`qpos  ${summary}`, 22, 145);

  // Latency line: IK time, controller pose age, control RX rate, qpos age.
  // Color-code each metric: green/yellow/red against rough VR-safe budgets.
  const ikMs    = +leader.ik_ms || 0;
  const poseMs  = +leader.pose_age_ms || 0;
  const ctrlHz  = +leader.control_hz || 0;
  // qpos age = how long since the BROWSER last received a telem update.
  // Server runs at 30 Hz so steady-state should be ~33 ms.  Spikes above
  // 60 ms indicate the BROWSER hitched (GC, decode work) — not the host.
  const qposMs  = _urdfState.lastTelemTs
    ? (performance.now() - _urdfState.lastTelemTs) : 0;
  const colorFor = (v, good, warn) => v < good ? '#2ea043' : v < warn ? '#d29922' : '#f85149';
  ctx.font = '500 14px ui-monospace, monospace';
  const lat = (label, val, unit, color) => ({label, val, unit, color});
  const items = [
    lat('ik',    ikMs.toFixed(1),    'ms', colorFor(ikMs,    5,  15)),
    lat('pose',  poseMs.toFixed(0),  'ms', colorFor(poseMs,  20, 60)),
    lat('ctrl', ctrlHz.toFixed(0),   'Hz', ctrlHz < 30 ? '#f85149' : ctrlHz < 60 ? '#d29922' : '#2ea043'),
    lat('qpos', qposMs.toFixed(0),   'ms', colorFor(qposMs,  50, 120)),
  ];
  let x = 22, y = 172;
  for (const it of items) {
    ctx.fillStyle = '#7d8590'; ctx.fillText(it.label, x, y);
    x += 32;
    ctx.fillStyle = it.color;  ctx.fillText(it.val,   x, y);
    x += 50;
    ctx.fillStyle = '#7d8590'; ctx.fillText(it.unit,  x, y);
    x += 32;
  }
  _hudState.texture.needsUpdate = true;
}

function _roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

// ---- IK target / engage / clamp visualization ------------------------------
//
// Drawn in the URDF root's frame so they live in robot-base coords (which is
// exactly what QuestLeader's hud_snapshot emits).  Markers are reused across
// frames — only their position changes.
//
//   green sphere   = engage_ee    (where the clutch anchored)
//   yellow sphere  = target_ee    (where IK actually tried to reach — after
//                                   workspace clamp)
//   red line       = engage_ee -> target_ee_raw  (drawn only when the raw
//                                                  target differs from the
//                                                  clamped one; the gap
//                                                  visualizes how far past
//                                                  the workspace wall the
//                                                  operator is pushing)

function _ensureIkMarkers() {
  if (!_urdfState.robot) return;
  if (_hudState.targetMarker) return;
  const sphereMat = (color) => new THREE.MeshBasicMaterial({
    color, transparent: true, opacity: 0.85, depthTest: false,
  });
  const sphereGeom = new THREE.SphereGeometry(0.018, 12, 10);
  _hudState.engageMarker = new THREE.Mesh(sphereGeom, sphereMat(0x2ea043));
  _hudState.targetMarker = new THREE.Mesh(sphereGeom, sphereMat(0xd29922));
  // Render markers on top of the URDF (not occluded) — depthTest: false +
  // high renderOrder.  Slight ugliness in exchange for always-visible debug.
  _hudState.engageMarker.renderOrder = 999;
  _hudState.targetMarker.renderOrder = 999;
  _hudState.engageMarker.visible = false;
  _hudState.targetMarker.visible = false;
  _hudState.engageMarker.layers.enable(_PREVIEW_LAYER);
  _hudState.targetMarker.layers.enable(_PREVIEW_LAYER);
  _urdfState.robot.add(_hudState.engageMarker, _hudState.targetMarker);
  // Clamp-violation line
  const lineGeom = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0),
  ]);
  _hudState.clampLine = new THREE.Line(
    lineGeom,
    new THREE.LineBasicMaterial({ color: 0xf85149, depthTest: false }),
  );
  _hudState.clampLine.renderOrder = 999;
  _hudState.clampLine.visible = false;
  _hudState.clampLine.layers.enable(_PREVIEW_LAYER);
  _urdfState.robot.add(_hudState.clampLine);
}

function _updateIkMarkers(leader) {
  if (!_urdfState.robot) return;
  _ensureIkMarkers();
  // Python-FK validation marker.  Drawn in robot LOCAL frame so it inherits
  // the URDF root's world transform — same coordinate system as engage_ee
  // and target_ee.  If FK is correct, this blue sphere sits at the same
  // world point as the URDF mesh's EE link (1_1_02_eb571_960_stp_2).
  if (Array.isArray(leader.fk_ee_actual)) {
    if (!_hudState.fkMarker) {
      _hudState.fkMarker = new THREE.Mesh(
        new THREE.SphereGeometry(0.012, 12, 10),
        new THREE.MeshBasicMaterial({
          color: 0x5599ff, transparent: true, opacity: 0.7, depthTest: false,
        }),
      );
      _hudState.fkMarker.renderOrder = 998;
      _hudState.fkMarker.layers.enable(_PREVIEW_LAYER);
      _urdfState.robot.add(_hudState.fkMarker);
    }
    _hudState.fkMarker.position.fromArray(leader.fk_ee_actual);
    _hudState.fkMarker.visible = true;
  } else if (_hudState.fkMarker) {
    _hudState.fkMarker.visible = false;
  }
  const engage = leader.engage_ee;
  const target = leader.target_ee;
  const targetRaw = leader.target_ee_raw;
  if (Array.isArray(engage) && leader.engaged) {
    _hudState.engageMarker.position.fromArray(engage);
    _hudState.engageMarker.visible = true;
  } else {
    _hudState.engageMarker.visible = false;
  }
  if (Array.isArray(target) && leader.engaged) {
    _hudState.targetMarker.position.fromArray(target);
    _hudState.targetMarker.visible = true;
  } else {
    _hudState.targetMarker.visible = false;
  }
  // Only draw the red clamp-violation line when the operator's raw target
  // differs from the clamped target by more than 5 mm — otherwise the line
  // is a degenerate point and just adds visual noise.
  if (Array.isArray(target) && Array.isArray(targetRaw) && leader.engaged) {
    const dx = target[0] - targetRaw[0];
    const dy = target[1] - targetRaw[1];
    const dz = target[2] - targetRaw[2];
    const gap = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (gap > 0.005) {
      const pts = [
        new THREE.Vector3(target[0],    target[1],    target[2]),
        new THREE.Vector3(targetRaw[0], targetRaw[1], targetRaw[2]),
      ];
      _hudState.clampLine.geometry.setFromPoints(pts);
      _hudState.clampLine.visible = true;
    } else {
      _hudState.clampLine.visible = false;
    }
  } else {
    _hudState.clampLine.visible = false;
  }
}
