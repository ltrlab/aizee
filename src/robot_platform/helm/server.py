"""Loopback-only Helm web app: the model viewer plus robot actions.

Actions need the per-process token served with the state, so another local page
cannot post them. With `live`, the stopped reader shows the arms (motors
disabled) whenever Minerva is online over the tether (helm.live_view); without
it the robot is never contacted until Connect is pressed. The reader and a
session that holds the motors are mutually exclusive: teleop, the servo sweep
(and returns and live joint sweeps) and Minerva Check take the motors from the
reader, which starts again once they end. The 3D view shows one source: the
session's arm as its receiver reads it, else the reader. The Leaders workspace
and a teleop session both need the leader USB ports, so they exclude each other.
Teleop, the servo sweep and Minerva Check exclude each other.
"""
import asyncio
import contextvars
from pathlib import Path
import secrets
import socket
import time
from .head_only import head_only
from .session import WIRED

STATIC = Path(__file__).parent / 'static'
# Served before a login, so the login page has Helm's type: these exact paths, nothing else under /helm.
LOGIN_FONTS = frozenset({'/helm/fonts/minecraft-regular.woff2', '/helm/fonts/minecraft-bold.woff2'})
RETURN_MOVED_RAD = .02  # A confirmed return starts only if no joint moved more than this since it was shown.



def task_phase(progress):
    """Operator text from recorded scene facts; never invent a successful outcome."""
    if progress.get('training_eligible') is False or progress.get('forbidden_contacts'):
        return 'Invalid scene contact; episode is ineligible.'
    if progress.get('dropped'):
        return 'Cube dropped outside the bucket; reset before retrying.'
    picked_up = progress.get('first_gripper') in ('left', 'right')
    if progress.get('bucket_settled'):
        if progress.get('success') and picked_up and progress.get('handoff'):
            return 'Task complete: pickup, handoff and bucket placement.'
        return 'Cube settled in the bucket; the complete pickup and handoff sequence is not confirmed.'
    if progress.get('handoff'):
        return 'Handoff complete; place the cube in the bucket.'
    if picked_up:
        giver = progress['first_gripper']
        receiver = 'right' if giver == 'left' else 'left'
        return f'{giver.capitalize()} gripper picked up the cube; hand it to the {receiver} gripper.'
    if 'cube_position' in progress:
        return 'Cube on the table; pick it up with either gripper.'
    return None


def camera_preflight(cameras, roles=('left_wrist', 'right_wrist', 'head')):
    """Count observed required streams; service or device presence alone is not readiness."""
    names = tuple(roles)
    fresh = [name for name in names if (cameras.get(name) or {}).get('fresh') is True]
    missing = [name for name in names if name not in fresh]
    result = f'{len(fresh)}/{len(names)} cameras fresh'
    if missing:
        result += '; missing or stale: ' + ', '.join(missing)
    return result


def task_visual_label(capabilities):
    """Describe the worker's actual appearance provenance, not a requested CLI mode."""
    mode = capabilities.get('visual_mode')
    if mode == 'collision_proxy':
        return 'Camera appearance: collision proxy geometry, not full CAD visuals.'
    if mode == 'cad':
        return 'Camera appearance: full CAD visuals.'
    return 'Camera appearance is not reported yet.'

# The logged-in session of the request being handled (helm.access), for the state it answers with.
CURRENT_SESSION = contextvars.ContextVar('helm_session', default=None)
OPEN_POSTS = ('/api/helm/logout', '/api/helm/control/claim', '/api/helm/control/release')

# A camera viewer on a slow link (a tablet on Minerva's Wi-Fi) gets fewer, fresh frames, not a closed stream or old
# ones. About one frame waits in the connection's send buffer (VIDEO_BACKLOG) and newer ones are skipped beyond it;
# the socket's own kernel buffer is capped (VIDEO_SOCKET_BUFFER; Linux doubles it), so no second of video queues on
# the link ahead of the page's state requests. The buffer's high-water mark stays far above the backlog, so asyncio
# never pauses the stream: Python 3.10's TLS layer resumes a paused one wrongly.
VIDEO_BACKLOG = 64 * 1024
VIDEO_SOCKET_BUFFER = 32 * 1024
VIDEO_HIGH_WATER = 4 * 1024 * 1024
VIDEO_SLOW_S = 2.  # The stream reports a slow link for this long after it last skipped a frame.


def video_backlog(transport):
    """Bytes a camera viewer's connection has yet to send."""
    return transport.get_write_buffer_size() if transport is not None and not transport.is_closing() else 0


def create_app(session, link, *, teleop=None, leaders=None, check=None, tune=None, token=None, live=False,
               gripper_zero=None, simulation_only=False, cameras=None, camera_video=None, episodes=None, ups=None,
               access=None, bridge=None, mesh_store=None, operator_files=None, calibration=None, network=None,
               display=None, head_setup=None, head_alignment=None, maps=None, twin=None):
    """`live`: keep the stopped reader running whenever Minerva is online and nothing else holds the motors
    (helm.live_view); sessions then take the motors from it instead of asking for Disconnect first.
    `episodes`: the recorded-episode library behind Replay (helm.episodes), told to read nothing while a
    recording runs. `ups`: the robot's UPS state (helm.ups.UpsWatch); while it announces a low-battery shutdown,
    nothing that moves the arms starts, and a running session and sweep are stopped (the arms lowered first).
    `access` (helm.access.Access): Helm serves other devices (on Minerva, beyond loopback): a login, one device in
    control, stopping always allowed. `bridge` (helm.bridge.BridgeHub): the GELLO bridge's state for the page.
    `mesh_store`: the 3D view's meshes, converted on the build PC (viewer.server.ModelStore). `operator_files`:
    where Helm on Minerva saves the operator's files and what its start changed there (helm.operator_files).
    `calibration` (helm.calibration.CameraCalibration): wrist camera stills during a live session, and their solve.
    `network` (helm.network.NetworkView): Minerva's network for the header and Settings, and its access point switch.
    `display` (helm.display_model.DisplayModel): the model the 3D view draws (the head model), when not the active
    one; every guide is drawn on it, the neck at the head's measured angles while a session drives it.
    `head_setup` (helm.head_setup.HeadSetup): the head's setup on Helm on Minerva (Limits tab, Head): its reader,
    reference, directions and limits. `head_alignment` (helm.head_alignment.HeadAlignment): head camera stills of the
    table's tags during a live session, their solve, and the measured mount saved for the head's pose feed.
    `maps` (helm.maps.MapLibrary): head scene captures for splat maps, and the trained maps the headset draws.
    `twin` (helm.twin.TwinCorrection): the digital twin's correction of the measured joints' zeros, for drawing only:
    the 3D view and the headset's model draw through it."""
    from aiohttp import web
    from robot_platform.viewer.server import create_app as viewer_app
    from .live_view import LiveView
    from .telemetry import select_telemetry
    token = token or secrets.token_urlsafe(24)
    camera_change = asyncio.Lock()
    video_sockets = set()
    from .xr import Headset
    headset = Headset()  # The operator's headset steering the head (helm.xr).
    # Settings → Headset's preview: the 3D view's head follows the headset while no session drives the head.
    xr_preview = {'on': False}
    if simulation_only and live:
        raise ValueError('Simulation-only Helm cannot start the physical live reader')
    if simulation_only:
        grippers = None
    else:
        from .gripper_zero import GripperZero
        grippers = gripper_zero or GripperZero(lambda: getattr(session, 'address', None) or session.device['address'])

    def head_alone():
        """Whether the teleop session drives the head alone (helm.head_only), leaving the arms to the reader."""
        return teleop is not None and head_only(getattr(teleop, 'receiver', None))

    def owner(arms=False):
        """What holds the motors, other than the reader; with `arms`, the arms' motors only."""
        if teleop is not None and teleop.active and not (arms and head_alone()):
            return 'teleop'
        if tune is not None and tune.active:
            return {'verify': 'the live joint sweep', 'return': 'the return'}.get(tune.mode, 'the servo sweep')
        if check is not None and check.running:
            return 'Minerva Check'
        if grippers is not None and grippers.running:
            return 'the gripper zero'
        return None

    live_view = LiveView(session, link, lambda: owner(arms=True))
    live_view.enabled = live

    async def take_motors(arms=True):
        """The reader steps aside for a session that needs the motors (it starts again once they're free); the
        head's setup stops reading, so a session's head receiver has its serial port. A session of the head alone
        (`arms` False) leaves the arms' reader running."""
        if head_setup is not None:
            await off_loop(head_setup.stop)
        if arms and not simulation_only:
            await off_loop(live_view.yield_motors)

    def guide():
        for source in (teleop, tune):
            pose = source.guide() if source is not None else None
            if pose is not None:
                return pose
        return session.guide()

    def shown():
        """What the 3D view draws: the guide, on the display model when Helm has one: the neck at the head's
        measured angles while a session drives it or its setup reads it, else (with the headset preview on) where
        the headset looks. The digital twin's correction (helm.twin) turns the measured joints by their zeros."""
        drawn, neck = posed()
        return twin.draw(drawn, neck=neck) if twin is not None else drawn

    def posed():
        """The 3D view's pose, and whether its neck's angles are measured (by a live session or the head's setup)."""
        pose = guide()
        if display is None:
            return pose, False
        head = getattr(teleop, 'head_joints', None)
        joints = head() if head is not None else None
        if joints is None and head_setup is not None:
            read = head_setup.joints()
            if read is not None:  # Each reading a pose of its own, as each headset pose is.
                return display.preview(pose, read, head_setup.sequence, source='head-setup',
                                       note='read by its setup, the servos limp'), True
        if joints is None and xr_preview['on']:
            return display.preview(pose, headset.angles(), headset.poses), False
        live = (getattr(teleop, 'receiver', None) or {}).get('mode') == 'live'
        return display.guide(pose, joints), joints is not None and live

    drawn = display.profile if display is not None else session.model.profile
    app = viewer_app([drawn], drawn.id, guide=shown, index_html=STATIC / 'index.html', local_only=access is None,
                     mesh_store=mesh_store)
    if access is not None:
        from .access import COOKIE, stop_request

        @web.middleware
        async def gate(request, handler):
            """Logged-in devices only; changes only from the device in control, except stopping."""
            path = request.path
            if path in ('/login', '/api/helm/login') or (request.method == 'GET' and path in LOGIN_FONTS):
                return await handler(request)
            session_token = request.cookies.get(COOKIE)
            if access.session(session_token) is None:
                if request.method == 'GET' and not path.startswith('/api/'):
                    raise web.HTTPFound('/login?next=/xr' if path == '/xr' else '/login')
                return web.json_response({'error': 'Log in to Helm first', 'login': True}, status=401)
            CURRENT_SESSION.set(session_token)
            if request.method == 'POST' and path not in OPEN_POSTS and not access.in_control(session_token):
                payload = None
                try:
                    payload = await request.json()
                except Exception:  # noqa: BLE001 - not JSON: not a stop either
                    pass
                if not stop_request(path, payload):
                    holder = access.view(session_token)['holder']
                    who = f"{holder['device']} has control" if holder else 'No device has control'
                    return web.json_response({'error': f'View only: {who}. Take control first.', 'view_only': True},
                                             status=403)
            return await handler(request)
        app.middlewares.append(gate)

    def authorized(request):
        if not secrets.compare_digest(request.headers.get('X-Helm-Token', ''), token):
            raise web.HTTPForbidden(text='Missing or invalid Helm session token')

    async def body(request):
        authorized(request)
        payload = await request.json()
        if not isinstance(payload, dict):
            raise ValueError('Expected a JSON object')
        if simulation_only:
            allowed = {
                '/api/helm/teleop/start', '/api/helm/teleop/stop', '/api/helm/teleop/key', '/api/helm/teleop/pedal',
                '/api/helm/teleop/route', '/api/helm/teleop/zero', '/api/helm/teleop/target',
                '/api/helm/teleop/receiver', '/api/helm/settings/apply', '/api/helm/settings/reset',
                '/api/helm/leaders/connect', '/api/helm/leaders/disconnect', '/api/helm/leaders/range',
                '/api/helm/leaders/reference', '/api/helm/leaders/save',
                '/api/helm/teleop/tuning/apply', '/api/helm/teleop/tuning/save',
                '/api/helm/teleop/tuning/restore', '/api/helm/task', '/api/helm/teleop/grip_feedback',
                '/api/helm/xr/zero', '/api/helm/xr/preview',
            }
            if request.path not in allowed:
                raise ValueError('Simulation mode cannot operate or calibrate the physical robot')
            if request.path.endswith('/teleop/target') and payload.get('target') != 'pc':
                raise ValueError('Simulation mode runs on this PC only')
            if request.path.endswith('/teleop/receiver') and payload.get('mode') != 'virtual':
                raise ValueError('Simulation mode cannot select a physical receiver')
        return payload

    def state_response():
        state = session.snapshot()
        state['link'] = {'running': link.running}
        state['live_view'] = live_view.snapshot()
        state['teleop'] = teleop.snapshot() if teleop is not None else None
        state['leaders'] = leaders.snapshot() if leaders is not None else None
        state['check'] = check.snapshot() if check is not None else None
        state['tune'] = tune.snapshot() if tune is not None else None
        state['gripper_zero'] = grippers.snapshot() if grippers is not None else None
        state['ups'] = ups.snapshot() if ups is not None else None
        state['calibration'] = calibration.snapshot() if calibration is not None else None
        state['head_alignment'] = head_alignment.snapshot() if head_alignment is not None else None
        state['maps'] = maps.snapshot() if maps is not None else None
        state['twin'] = twin.snapshot() if twin is not None else None
        state['mode'] = 'simulation' if simulation_only else 'robot'
        task = (state['teleop'] or {}).get('task')
        if simulation_only and task is not None:
            task['display_phase'] = task_phase(task.get('task') or {})
            task['display_visual'] = task_visual_label(task.get('capabilities') or {})
        elif task is not None:
            task['camera_summary'] = camera_preflight(task.get('cameras') or {}, task.get('camera_roles')
                                                       or ('left_wrist', 'right_wrist', 'head'))
        # The model's joint telemetry from the one source the 3D view shows: a running session, else the reader.
        state['telemetry'] = select_telemetry(state)
        if display is not None:  # For the model the 3D view draws, as its guide is.
            state['telemetry'] = display.telemetry(state['telemetry'])
        state['action_token'] = token
        state['access'] = access.view(CURRENT_SESSION.get()) if access is not None else {'enabled': False}
        state['bridge'] = bridge.snapshot() if bridge is not None else None
        state['network'] = network.snapshot() if network is not None else None
        state['xr'] = dict(headset.status(), preview=xr_preview['on'], preview_available=display is not None)
        state['head_setup'] = head_setup.snapshot() if head_setup is not None else None
        state['operator_files'] = operator_files
        return web.json_response(state, headers={'Cache-Control': 'no-store'})

    async def off_loop(function):
        return await asyncio.get_running_loop().run_in_executor(None, function)

    async def state(_request):
        return state_response()

    async def login_page(_request):
        return web.FileResponse(STATIC / 'login.html', headers={'Cache-Control': 'no-store'})

    async def login(request):
        """A password and a device name for a session cookie (helm.access)."""
        if access is None:
            raise ValueError('This Helm needs no login')
        payload = await request.json()
        if not isinstance(payload, dict) or set(payload) - {'password', 'device'}:
            raise ValueError('Expected password and device')
        try:
            session_token = await off_loop(lambda: access.login(payload.get('password', ''), device=payload.get('device'),
                                                                address=request.remote))
        except PermissionError as exc:
            return web.json_response({'error': str(exc)}, status=429)
        except ValueError as exc:
            return web.json_response({'error': str(exc)}, status=401)
        from .access import COOKIE, SESSION_S
        response = web.json_response({'ok': True})
        response.set_cookie(COOKIE, session_token, httponly=True, samesite='Strict', secure=request.secure,
                            max_age=int(SESSION_S), path='/')
        return response

    async def logout(request):
        await body(request)
        from .access import COOKIE
        access.logout(CURRENT_SESSION.get())
        response = web.json_response({'ok': True})
        response.del_cookie(COOKIE, path='/')
        return response

    async def control_claim(request):
        """This device takes control of Helm (from whichever had it)."""
        await body(request)
        access.claim(CURRENT_SESSION.get())
        return state_response()

    async def control_release(request):
        await body(request)
        access.release(CURRENT_SESSION.get())
        return state_response()

    def camera_apply_reason():
        if simulation_only:
            return 'Simulation mode cannot access physical cameras'
        if cameras is None:
            return 'Camera settings are not configured'
        if getattr(cameras, 'write_fault', None):
            return cameras.write_fault
        if camera_change.locked():
            return 'A camera change is running; wait for readback'
        if teleop is not None:
            task = (teleop.snapshot().get('task') or {})
            recording = task.get('recording') or {}
            if teleop.active or recording.get('active') or recording.get('state') in ('starting', 'recording', 'stopping', 'finishing'):
                return 'Stop the teleop session and capture before changing camera settings'
        return None

    def need_cameras():
        if simulation_only:
            raise ValueError('Simulation mode cannot access physical cameras')
        if cameras is None:
            raise ValueError('Camera settings are not configured')

    replay_busy_at = [-1., None]

    def replay_busy():
        """Why Replay may not read now: a recording runs on the same robot CPUs and disk. Cached briefly, since
        every uncached frame asks."""
        now = time.monotonic()
        if now - replay_busy_at[0] > .25:
            reason = None
            if teleop is not None:
                recording = (teleop.snapshot().get('task') or {}).get('recording') or {}
                if recording.get('active') or recording.get('state') in ('starting', 'recording', 'stopping', 'finishing'):
                    reason = 'Replay waits while a recording is running'
            replay_busy_at[:] = [now, reason]
        return replay_busy_at[1]

    if episodes is not None:
        episodes.busy = replay_busy

    def need_episodes():
        if episodes is None:
            raise ValueError('Replay is not configured in this Helm')

    def episode_name(request):
        return f"{request.match_info['session']}/{request.match_info['episode']}"

    async def episodes_list(_request):
        need_episodes()
        return web.json_response(await off_loop(episodes.listing), headers={'Cache-Control': 'no-store'})

    def model_mapping():
        """Motor -> URDF joint, scale, offset and unit: how a recorded position poses the 3D model, as the stopped
        reader's telemetry maps it (helm.telemetry.stopped_packet)."""
        from contextlib import nullcontext
        mapping, transmissions = getattr(session, 'mapping', None) or {}, getattr(session, 'transmissions', None) or []
        with getattr(session, 'lock', None) or nullcontext():
            rows = {row['motor']: dict(joint=joint, scale=row['sign'], offset=row['offset_rad'], unit='rad')
                    for joint, row in mapping.items()}
            for entry in transmissions:
                scale = entry['metres_per_motor_rad']
                rows[entry['motor']] = dict(joint=entry['joint'], scale=scale, unit='m',
                                            offset=entry['joint_reference_m'] - scale * entry['motor_reference_rad'])
        return rows

    async def episode_get(request):
        from .episodes import EpisodeBusy
        need_episodes()
        name = episode_name(request)
        try:
            data = await off_loop(lambda: episodes.episode(name))
        except EpisodeBusy as exc:
            return web.json_response({'error': str(exc), 'busy': True}, status=409)
        return web.json_response(dict(data, model_mapping=model_mapping()), headers={'Cache-Control': 'no-store'})

    async def policies_list(_request):
        """Policy mode's choices: the exported policies on the robot, read by Replay's worker."""
        need_episodes()
        return web.json_response(await off_loop(episodes.policies), headers={'Cache-Control': 'no-store'})

    async def episode_review(request):
        """Keep or Discard an episode for training, with an optional note; refused while a recording runs."""
        from .episodes import EpisodeBusy
        payload = await body(request)
        need_episodes()
        if not set(payload) <= {'verdict', 'note'} or 'verdict' not in payload:
            raise ValueError('Expected verdict (keep, discard or null) and optionally a note')
        name = episode_name(request)
        try:
            record = await off_loop(lambda: episodes.review(name, payload['verdict'], payload.get('note', '')))
        except EpisodeBusy as exc:
            return web.json_response({'error': str(exc), 'busy': True}, status=409)
        return web.json_response({'review': record}, headers={'Cache-Control': 'no-store'})

    async def episode_frame(request):
        from .episodes import EpisodeBusy
        need_episodes()
        name, role = episode_name(request), request.match_info['role']
        try:
            index = int(request.match_info['index'])
        except ValueError:
            raise ValueError('A frame index is a whole number') from None
        try:
            jpeg = await off_loop(lambda: episodes.frame(name, index, role))
        except EpisodeBusy as exc:
            return web.json_response({'error': str(exc), 'busy': True}, status=409)
        # A complete episode never changes: the browser may keep its frames; a partial one might still grow.
        cache = 'private, max-age=86400, immutable' if episodes.complete(name) else 'no-store'
        return web.Response(body=jpeg, content_type='image/jpeg', headers={'Cache-Control': cache})

    async def camera_get(_request):
        need_cameras()
        result = await off_loop(cameras.snapshot)
        reason = camera_apply_reason()
        return web.json_response(dict(result, can_apply=reason is None, reason=reason),
                                 headers={'Cache-Control': 'no-store'})

    async def camera_set(request):
        payload = await body(request)
        need_cameras()
        if set(payload) != {'name', 'value'}:
            raise ValueError('Expected camera control name and value')
        reason = camera_apply_reason()
        if reason:
            raise ValueError(reason)
        # The lock is acquired before yielding. Session start shares this fence,
        # including its stopped-reader handoff, so it cannot race a remote write.
        async with camera_change:
            pending = asyncio.create_task(off_loop(lambda: cameras.set(
                request.match_info['role'], payload['name'], payload['value'])))
            try:
                result = await asyncio.shield(pending)
            except asyncio.CancelledError:
                # A disconnected HTTP client cannot release the start fence while
                # its finite camera operation might still change exposure.
                await pending
                raise
        return web.json_response(result, headers={'Cache-Control': 'no-store'})

    async def camera_frame(request):
        need_cameras()
        jpeg = await off_loop(lambda: cameras.frame(request.match_info['role']))
        return web.Response(body=jpeg, content_type='image/jpeg', headers={'Cache-Control': 'no-store'})


    async def camera_live(request):
        if simulation_only:
            raise ValueError('Simulation mode cannot access physical cameras')
        if camera_video is None:
            raise ValueError('Live camera video is not configured')
        # WebSocket handshakes are GET: the ordinary POST origin guard does not apply.
        if request.headers.get('Origin') not in (None, f'{request.scheme}://{request.host}'):
            raise web.HTTPForbidden(text='Cross-origin camera stream rejected')
        if not secrets.compare_digest(request.query.get('token', ''), token):
            raise web.HTTPForbidden(text='Missing or invalid Helm session token')
        from .camera_video import browser_packet
        ws = web.WebSocketResponse(heartbeat=5., max_msg_size=4096, compress=False)
        await ws.prepare(request)
        transport = request.transport
        if transport is not None:
            transport.set_write_buffer_limits(high=VIDEO_HIGH_WATER)
            raw = transport.get_extra_info('socket')
            if raw is not None:
                raw.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, VIDEO_SOCKET_BUFFER)
        video_sockets.add(ws)
        # A viewer may want some cameras only (?roles=head: the headset's screen), not every camera's frames; the
        # headset's 3D picture asks for the head camera's frames with their depth (?roles=head_depth).
        wanted = {role for role in request.query.get('roles', '').split(',') if role}
        from .camera_video import DEPTH_ROLE
        acquiring = asyncio.create_task(off_loop(lambda: camera_video.subscribe(depth=DEPTH_ROLE in wanted)))
        try:
            viewer = await asyncio.shield(acquiring)
        except asyncio.CancelledError:
            viewer = await acquiring
            await off_loop(lambda: camera_video.unsubscribe(viewer))
            video_sockets.discard(ws)
            raise
        viewer_heartbeat = {'sequence':None, 'at':asyncio.get_running_loop().time()}

        async def close_video(code=1000, message=b''):
            try:
                await asyncio.wait_for(ws.close(code=code, message=message), 1.)
            except asyncio.TimeoutError:
                if request.transport is not None:
                    request.transport.close()

        async def publish():
            sent, status_at, skipped_at = {}, 0., None
            try:
                while not ws.closed:
                    status, frames = camera_video.snapshot()
                    now = asyncio.get_running_loop().time()
                    if now-viewer_heartbeat['at'] > 3.:
                        await close_video(code=1008, message=b'Viewer heartbeat expired')
                        return
                    status['viewer_heartbeat'] = viewer_heartbeat['sequence']
                    status['link'] = 'slow' if skipped_at is not None and now-skipped_at < VIDEO_SLOW_S else 'ok'
                    if now-status_at >= .25:
                        await asyncio.wait_for(ws.send_json(status), .5)
                        status_at = now
                    for role, (header, jpeg) in frames.items():
                        if viewer_heartbeat['sequence'] is None or (wanted and role not in wanted):
                            continue
                        if role == DEPTH_ROLE and role not in wanted:
                            continue  # The head's depth goes only to a viewer that asks for it by name.
                        if header['sequence'] == sent.get(role):
                            continue
                        if video_backlog(transport) > VIDEO_BACKLOG:
                            skipped_at = now  # The link is behind: newer frames go next tick, or later.
                            break
                        # Recheck per role after any previous blocked socket send.
                        _current_status, current_frames = camera_video.snapshot()
                        current = current_frames.get(role)
                        if current is None:
                            continue
                        header, jpeg = current
                        header = dict(header, viewer_heartbeat=viewer_heartbeat['sequence'])
                        await asyncio.wait_for(ws.send_bytes(browser_packet(header, jpeg)), .5)
                        sent[role] = header['sequence']
                    await asyncio.sleep(.1)
            except (ConnectionError, RuntimeError, asyncio.TimeoutError):
                await close_video()
        publisher = asyncio.create_task(publish())
        try:
            async for message in ws:
                if message.type == web.WSMsgType.TEXT:
                    try:
                        value = message.json()
                        sequence = value.get('sequence')
                        if set(value) != {'type', 'sequence'} or value['type'] != 'heartbeat' or type(sequence) is not int or not 0 <= sequence <= 2**53-1:
                            raise ValueError('Invalid viewer heartbeat')
                        if viewer_heartbeat['sequence'] is not None and sequence <= viewer_heartbeat['sequence']:
                            raise ValueError('Viewer heartbeat did not advance')
                        viewer_heartbeat.update(sequence=sequence, at=asyncio.get_running_loop().time())
                    except (ValueError, TypeError, AttributeError):
                        await close_video(code=1008, message=b'Invalid read-only viewer heartbeat')
                elif message.type == web.WSMsgType.BINARY:
                    await close_video(code=1008, message=b'Camera stream is read-only')
        finally:
            async def release_viewer():
                publisher.cancel()
                await asyncio.gather(publisher, return_exceptions=True)
                video_sockets.discard(ws)
                await off_loop(lambda: camera_video.unsubscribe(viewer))
            # aiohttp can cancel a disconnected request during its first cleanup
            # await. The subscriber release must still finish in that case.
            releasing = asyncio.create_task(release_viewer())
            try:
                await asyncio.shield(releasing)
            except asyncio.CancelledError:
                await releasing
                raise
        return ws

    def power_ok():
        """Nothing that moves the arms starts while Minerva's UPS is shutting the Jetson down."""
        reason = ups.refusal() if ups is not None else None
        if reason:
            raise ValueError(reason)

    def ups_shutdown(_record):
        """Minerva's UPS is about to power the Jetson off: end what holds the arms through their own stop paths
        (the loop lowers an enabled arm before the receiver closes); a recording first, so its episode is kept."""
        if teleop is not None and teleop.active:
            recording = (teleop.snapshot().get('task') or {}).get('recording') or {}
            if recording.get('active') or recording.get('state') in ('starting', 'recording'):
                try:
                    teleop.task_action('stop_record')
                except Exception:
                    pass  # Stopping the session still ends the recording.
            teleop.stop()
        if tune is not None and tune.active:
            tune.stop()

    if ups is not None and ups.on_shutdown is None:
        ups.on_shutdown = ups_shutdown

    def not_checking():
        if grippers is not None and grippers.running:
            raise ValueError('A gripper zero is running; wait for it to finish')
        if check is not None and check.running:
            raise ValueError('Minerva Check is running; wait for it to finish')
        if tune is not None and tune.active:
            raise ValueError('The servo sweep is running; finish or stop it first')

    async def connect(request):
        await body(request)
        if teleop is not None and teleop.active and not head_alone():
            raise ValueError('Stop the teleop session before connecting to the motors')
        not_checking()
        live_view.set_enabled(True)
        if not link.running:
            link.connect()
        return state_response()

    async def disconnect(request):
        await body(request)
        live_view.set_enabled(False)  # The live view stays off until Connect.
        await off_loop(link.disconnect)
        return state_response()

    async def limits(request):
        payload = await body(request)
        if set(payload) != {'action', 'motor'}:
            raise ValueError('Expected action and motor')
        session.limits(payload['action'], payload['motor'])
        return state_response()

    async def save(request):
        await body(request)
        result = await off_loop(session.save_limits)
        if teleop is not None:
            await off_loop(teleop.reload_tuning)
        response = state_response()
        response.headers['X-Helm-Saved'] = result['path']
        return response

    async def direction(request):
        payload = await body(request)
        if set(payload) != {'action', 'joint'}:
            raise ValueError('Expected action and joint')
        session.direction(payload['action'], payload['joint'])
        return state_response()

    def need_teleop():
        if teleop is None:
            raise ValueError('Helm was started without a teleop profile')

    async def teleop_start(request):
        """Start a teleop session; with `replay` ({episode, speed}), a recorded episode drives the arms in place of
        the leaders (live replay), and Replay's library must know it as complete; with `policy` ({policy}), a policy
        does (policy mode)."""
        payload = await body(request)
        need_teleop()
        replay, policy = payload.get('replay'), payload.get('policy')
        if replay is not None and policy is not None:
            raise ValueError('A session replays an episode or runs a policy, not both')
        if replay is not None:
            from robot_platform.teleop.episode_source import validate_request
            from .episodes import EpisodeError
            need_episodes()
            name = validate_request(replay)['episode']
            try:
                summary = (await off_loop(lambda: episodes.episode(name)))['summary']
            except EpisodeError as exc:  # The robot is unreachable, or (EpisodeBusy) a recording runs.
                raise ValueError(f'The episode could not be read: {exc}') from None
            if not summary.get('complete'):
                raise ValueError('Only a complete episode can be replayed on the arms')
        power_ok()
        if leaders is not None and leaders.running:
            raise ValueError('Disconnect the leaders in the Leaders tab before starting teleop')
        not_checking()
        if simulation_only and (getattr(teleop, 'simulation', None) is None
                                or teleop.target != 'pc' or teleop.receiver['mode'] != 'virtual'):
            raise ValueError('Simulation mode needs the virtual receiver on this PC')
        if getattr(cameras, 'write_fault', None):
            raise ValueError(cameras.write_fault)
        if camera_change.locked():
            raise ValueError('A camera change is running; wait for readback before starting teleop')
        async with camera_change:
            await take_motors(arms=not head_alone())
            teleop.start(confirm=payload.get('confirm') is True, **({'replay': replay} if replay is not None else {}),
                         **({'policy': policy} if policy is not None else {}))
        return state_response()

    async def task_action(request):
        payload = await body(request)
        need_teleop()
        if set(payload) != {'action'} or payload['action'] not in ('setup', 'reset', 'start_record', 'stop_record'):
            raise ValueError('Expected task action: setup, reset, start_record or stop_record')
        if payload['action'] == 'start_record':
            power_ok()
            if getattr(cameras, 'write_fault', None):
                raise ValueError(cameras.write_fault)
        await off_loop(lambda: teleop.task_action(payload['action']))
        return state_response()

    async def teleop_stop(request):
        await body(request)
        need_teleop()
        await off_loop(teleop.stop)
        return state_response()

    async def teleop_key(request):
        payload = await body(request)
        need_teleop()
        if set(payload) != {'key'}:
            raise ValueError('Expected key')
        teleop.key(payload['key'])
        return state_response()

    async def teleop_pedal(request):
        payload = await body(request)
        need_teleop()
        if payload.get('pedal') == 'a' and getattr(cameras, 'write_fault', None):
            raise ValueError(cameras.write_fault)
        await off_loop(lambda: teleop.pedal(payload))
        return state_response()

    async def teleop_grip_feedback(request):
        payload = await body(request)
        need_teleop()
        await off_loop(lambda: teleop.grip_feedback(payload))
        return state_response()

    async def teleop_route(request):
        payload = await body(request)
        need_teleop()
        if set(payload) != {'action', 'motor'}:
            raise ValueError('Expected action and motor')
        await off_loop(lambda: teleop.route(payload['action'], payload['motor']))
        return state_response()

    async def teleop_manual_tuning(request):
        payload = await body(request)
        need_teleop()
        not_checking()
        await off_loop(lambda: teleop.manual_tuning(request.match_info['action'], payload))
        return state_response()

    async def settings_get(_request):
        """The Settings workspace's catalog: read-only, like the state."""
        need_teleop()
        return web.json_response(await off_loop(teleop.settings_snapshot), headers={'Cache-Control': 'no-store'})

    async def settings_action(request):
        """Apply edits to, or reset, the settings for the next teleop session; nothing reaches the motors."""
        payload = await body(request)
        need_teleop()
        action = request.match_info['action']
        if action not in ('apply', 'reset'):
            raise ValueError('Use apply or reset')
        result = await off_loop(lambda: teleop.settings_update(action, payload))
        return web.json_response(result, headers={'Cache-Control': 'no-store'})

    def need_leaders():
        if leaders is None:
            raise ValueError('Helm was started without a teleop profile')

    async def leaders_connect(request):
        await body(request)
        need_leaders()
        if teleop is not None and teleop.active:
            raise ValueError('Stop the teleop session before connecting the leaders here')
        not_checking()
        leaders.connect()
        return state_response()

    async def leaders_disconnect(request):
        await body(request)
        need_leaders()
        await off_loop(leaders.disconnect)
        return state_response()

    async def leaders_range(request):
        payload = await body(request)
        need_leaders()
        if set(payload) != {'action', 'key'}:
            raise ValueError('Expected action and key')
        leaders.act(payload['action'], payload['key'])
        return state_response()

    async def leaders_reference(request):
        await body(request)
        need_leaders()
        leaders.capture_reference()
        return state_response()

    async def leaders_save(request):
        await body(request)
        need_leaders()
        result = await off_loop(leaders.save)
        response = state_response()
        response.headers['X-Helm-Saved'] = result['path']
        return response

    async def teleop_target(request):
        payload = await body(request)
        need_teleop()
        if set(payload) != {'target'}:
            raise ValueError('Expected target')
        teleop.set_target(payload['target'])
        return state_response()

    async def teleop_receiver(request):
        payload = await body(request)
        need_teleop()
        if not set(payload) <= {'mode', 'arm', 'collision'} or 'mode' not in payload:
            raise ValueError('Expected mode and, for live, arm')
        teleop.set_receiver(payload['mode'], payload.get('arm'), payload.get('collision'))
        return state_response()

    async def connection(request):
        payload = await body(request)
        if set(payload) != {'connection'}:
            raise ValueError('Expected connection')
        not_checking()
        if teleop is not None:
            if teleop.active:
                raise ValueError('Stop the teleop session before changing the connection')
            if payload['connection'] not in WIRED and teleop.receiver['mode'] == 'live':
                raise ValueError('Live teleop runs over the wired tether; choose the virtual receiver first')
        await take_motors()  # The reader reconnects over the new connection (on its own over the tether).
        session.set_connection(payload['connection'])
        return state_response()

    async def teleop_zero(request):
        await body(request)
        need_teleop()
        await off_loop(teleop.set_zero)
        return state_response()

    async def gripper_zero_run(request):
        """Zero one gripper at its fully closed jaws, after the operator's confirmation (helm.gripper_zero)."""
        payload = await body(request)
        if not set(payload) <= {'motor', 'confirm'}:
            raise ValueError('Expected motor and confirm')
        if teleop is not None and teleop.active:
            raise ValueError('Stop the teleop session before zeroing a gripper')
        not_checking()
        if getattr(session, 'connection', 'tether') not in WIRED:
            raise ValueError('Gripper zero runs over the wired tether; choose Tether first')
        motor, confirm = payload.get('motor'), payload.get('confirm') is True
        if not confirm:
            raise ValueError('Close the gripper fully by hand, then confirm the zero')
        power_ok()
        await take_motors()  # The live reader steps aside; the zero takes the motors.
        await off_loop(lambda: grippers.zero(motor, confirm=confirm))
        return state_response()

    async def check_run(request):
        await body(request)
        if grippers is not None and grippers.running:
            raise ValueError('A gripper zero is running; wait for it to finish')
        if check is None:
            raise ValueError('Helm was started without Minerva Check')
        if teleop is not None and teleop.active:
            raise ValueError('Stop the teleop session before running Minerva Check')
        if tune is not None and tune.active:
            raise ValueError('Stop the servo sweep before running Minerva Check')
        power_ok()
        await take_motors()  # Minerva Check reads the motors itself.
        check.start()
        return state_response()

    def need_calibration():
        if calibration is None:
            raise ValueError('Camera calibration runs with Helm on Minerva and a teleop profile')

    async def calibration_thumbnail(_request):
        """The last still's small JPEG, its tags outlined; the page asks once per still."""
        need_calibration()
        jpeg = calibration.thumbnail()
        if jpeg is None:
            raise web.HTTPNotFound(text='No still yet')
        return web.Response(body=jpeg, content_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    async def calibration_action(request):
        """Capture a still, undo the last, start a new run, or solve (helm.calibration); none moves a motor."""
        payload = await body(request)
        need_calibration()
        action = request.match_info['action']
        if action in ('capture', 'undo'):
            if set(payload) != {'role'}:
                raise ValueError('Expected role')
            role = payload['role']
            await off_loop(lambda: calibration.capture(role) if action == 'capture' else calibration.undo(role))
        elif action in ('new', 'solve'):
            if payload:
                raise ValueError('Expected no fields')
            await off_loop(calibration.new_run if action == 'new' else calibration.solve)
        else:
            raise web.HTTPNotFound()
        return state_response()

    def need_head_alignment():
        if head_alignment is None:
            raise ValueError('Head camera alignment runs with Helm on Minerva and a teleop profile')

    async def head_alignment_thumbnail(_request):
        """The last head still's small JPEG, its tags outlined."""
        need_head_alignment()
        jpeg = head_alignment.thumbnail()
        if jpeg is None:
            raise web.HTTPNotFound(text='No still yet')
        return web.Response(body=jpeg, content_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    async def head_alignment_action(request):
        """Capture a head still, undo the last, start a new run, solve, or save the solve as the head's camera file
        (helm.head_alignment); none moves a motor."""
        payload = await body(request)
        need_head_alignment()
        actions = {'capture': head_alignment.capture, 'undo': head_alignment.undo, 'new': head_alignment.new_run,
                   'solve': head_alignment.solve, 'save': head_alignment.save}
        action = actions.get(request.match_info['action'])
        if action is None:
            raise web.HTTPNotFound()
        if payload:
            raise ValueError('Expected no fields')
        await off_loop(action)
        return state_response()

    async def twin_action(request):
        """Apply the digital twin's correction (helm.twin) or remove it; drawing only, so neither moves a motor or
        changes a zero."""
        payload = await body(request)
        if twin is None:
            raise ValueError("The digital twin's correction runs with Helm on Minerva and a teleop profile")
        action = {'apply': twin.apply, 'clear': twin.clear}.get(request.match_info['action'])
        if action is None:
            raise web.HTTPNotFound()
        if payload:
            raise ValueError('Expected no fields')
        await off_loop(action)
        return state_response()

    def need_maps():
        if maps is None:
            raise ValueError('Splat maps run with Helm on Minerva and a teleop profile')

    async def maps_listing(_request):
        """The trained maps (newest first) and Minerva's pose in each world, as the headset reads them."""
        need_maps()
        return web.json_response(await off_loop(maps.listing), headers={'Cache-Control': 'no-store'})

    async def map_splat(request):
        """A trained map's compact Gaussians (xr_splat.js draws them)."""
        need_maps()
        try:
            path = maps.splat(request.match_info['name'])
        except FileNotFoundError as exc:
            raise web.HTTPNotFound(text=str(exc))
        return web.FileResponse(path, headers={'Content-Type': 'application/octet-stream',
                                               'Cache-Control': 'no-store'})

    async def map_capture(request):
        """A head scene capture for a map, recorded on the robot as the operator looks around (helm.maps); it moves
        no motor."""
        payload = await body(request)
        need_maps()
        if set(payload) != {'seconds'}:
            raise ValueError('Expected seconds')
        await off_loop(lambda: maps.capture(payload['seconds']))
        return state_response()

    async def network_ap(request):
        """Minerva's access point on or off (helm.network, robot_platform.ops.network). Not while something holds the
        motors: the access point takes Minerva's Wi-Fi, which a session's leader link or viewer may be using."""
        payload = await body(request)
        if network is None:
            raise ValueError("Minerva's network is switched from Helm on Minerva")
        if set(payload) != {'on'} or type(payload['on']) is not bool:
            raise ValueError('Expected on: true or false')
        busy = owner()
        if busy is not None:
            raise ValueError(f"Stop {busy} first: switching the access point moves Minerva's Wi-Fi")
        await off_loop(lambda: network.set_ap(payload['on']))
        return state_response()

    async def xr_page(_request):
        return web.FileResponse(STATIC / 'xr.html', headers={'Cache-Control': 'no-store'})

    async def xr_socket(request):
        """The headset's poses (helm.xr): each, as yaw and pitch from the zero, to the running teleop session's head.
        A logged-in device's page with Helm's action token, not control: the session the device in control engages
        decides whether the head moves. One device's headset at a time (helm.xr.Headset)."""
        import json as _json
        if request.headers.get('Origin') not in (None, f'{request.scheme}://{request.host}'):
            raise web.HTTPForbidden(text='Cross-origin headset stream rejected')
        if not secrets.compare_digest(request.query.get('token', ''), token):
            raise web.HTTPForbidden(text='Missing or invalid Helm session token')
        owner = CURRENT_SESSION.get() if access is not None else None
        device = (access.view(owner)['device'] if access is not None else None) or 'The headset'
        ws = web.WebSocketResponse(heartbeat=5., max_msg_size=1024, compress=False)
        await ws.prepare(request)
        try:
            connection = headset.connect(owner, device)
        except ValueError as exc:
            await ws.send_json({'type': 'error', 'error': str(exc), 'busy': True})
            await ws.close()
            return ws
        steered = 0
        try:
            async for message in ws:
                if message.type != web.WSMsgType.TEXT:
                    continue
                try:
                    data = _json.loads(message.data)
                    kind = data.get('type') if isinstance(data, dict) else None
                    if kind in ('zero', 'start'):
                        (headset.zero if kind == 'zero' else headset.start)(connection)
                        await ws.send_json({'type': 'status', 'zeroed': kind == 'zero', 'restarted': kind == 'start',
                                            **headset.status()})
                        continue
                    pose = headset.receive(connection, data)
                except ValueError as exc:
                    await ws.send_json({'type': 'error', 'error': str(exc)})
                    continue
                if pose is None:
                    continue
                running = teleop is not None and teleop.head_pose(*pose)
                steered += 1
                if steered % 15 == 1:  # Now and then: what the head does with it.
                    head = (((teleop.snapshot() if teleop is not None else None) or {}).get('head') or {})
                    await ws.send_json({'type': 'status', 'steering': bool(running), 'head': head.get('state'),
                                        'source': head.get('source'), 'head_fault': head.get('fault'),
                                        **headset.status()})
        finally:
            headset.disconnect(connection)
        return ws

    async def xr_zero(request):
        await body(request)
        headset.zero()
        return state_response()

    def need_head_setup():
        if head_setup is None:
            raise ValueError("The head is set up in Helm on Minerva, beside its servo board")

    async def head_read(request):
        """Read the head (its servos limp), or stop. Not while a session holds the motors."""
        payload = await body(request)
        if set(payload) != {'on'} or not isinstance(payload['on'], bool):
            raise ValueError('Expected {"on": true} or {"on": false}')
        need_head_setup()
        if payload['on']:
            held = owner()
            if held:
                raise ValueError(f'The head is read once {held} has ended')
            await off_loop(head_setup.start)
        else:
            await off_loop(head_setup.stop)
        return state_response()

    async def head_reference(request):
        await body(request)
        need_head_setup()
        await off_loop(head_setup.set_reference)
        return state_response()

    async def head_direction(request):
        payload = await body(request)
        if set(payload) != {'action', 'joint'}:
            raise ValueError('Expected action and joint')
        need_head_setup()
        await off_loop(lambda: head_setup.direction(payload['action'], payload['joint']))
        return state_response()

    async def head_limits(request):
        payload = await body(request)
        if set(payload) != {'action', 'motor'}:
            raise ValueError('Expected action and motor')
        need_head_setup()
        await off_loop(lambda: head_setup.limit(payload['action'], payload['motor']))
        return state_response()

    async def head_limits_save(request):
        await body(request)
        need_head_setup()
        result = await off_loop(head_setup.save_limits)
        response = state_response()
        response.headers['X-Helm-Saved'] = result['path']
        return response

    async def head_corner(request):
        """A shoulder corner: `record` the head's pose now (turned and tilted down until just clear), or `clear` them."""
        payload = await body(request)
        if set(payload) != {'action'}:
            raise ValueError('Expected action: record or clear')
        need_head_setup()
        await off_loop(lambda: head_setup.corner(payload['action']))
        return state_response()

    async def head_clearance_save(request):
        await body(request)
        need_head_setup()
        result = await off_loop(head_setup.save_clearance)
        response = state_response()
        response.headers['X-Helm-Saved'] = result['path']
        return response

    async def xr_preview_switch(request):
        """Settings → Headset: the 3D view's head follows the headset, or not. Display only."""
        payload = await body(request)
        if set(payload) != {'on'} or not isinstance(payload['on'], bool):
            raise ValueError('Expected {"on": true} or {"on": false}')
        if payload['on'] and display is None:
            raise ValueError('The 3D view has no head model to preview')
        xr_preview['on'] = payload['on']
        return state_response()

    def need_tune():
        if tune is None:
            raise ValueError('Helm was started without a teleop profile')

    async def tune_configure(request):
        payload = await body(request)
        need_tune()
        if not set(payload) <= {'arm', 'joints', 'mode', 'target', 'absent'} or 'arm' not in payload:
            raise ValueError('Expected arm, and optionally joints, mode, target and absent (an unplugged gripper)')
        tune.configure(arm=payload['arm'], joints=payload.get('joints'), mode=payload.get('mode', 'live'),
                       target=payload.get('target'), verify=False, absent=payload.get('absent'))
        return state_response()

    async def tune_start(request):
        payload = await body(request)
        need_tune()
        if grippers is not None and grippers.running:
            raise ValueError('A gripper zero is running; wait for it to finish')
        if teleop is not None and teleop.active:
            raise ValueError('Stop the teleop session before the servo sweep')
        if check is not None and check.running:
            raise ValueError('Minerva Check is running; wait for it to finish')
        power_ok()
        await take_motors()
        tune.start(confirm=payload.get('confirm') is True)
        return state_response()

    async def tune_return(request):
        """Return an arm from where it rests to its reference. Without confirm: the offer, from the stopped link's
        readings (the 3D view shows them). With confirm: the arm must still read as offered; the link disconnects
        and the return starts from that pose."""
        payload = await body(request)
        need_tune()
        if not set(payload) <= {'arm', 'confirm'} or payload.get('arm') not in ('left', 'right'):
            raise ValueError('Expected arm (left or right), and optionally confirm')
        if teleop is not None and teleop.active:
            raise ValueError('Stop the teleop session before returning an arm')
        not_checking()
        if tune.active:
            raise ValueError('Stop the sweep before returning an arm')
        offer = tune.return_offer(payload['arm'])
        if payload.get('confirm') is not True:
            tune.offer = offer
            return state_response()
        shown, tune.offer = tune.offer, None
        if (not shown or shown['arm'] != offer['arm']
                or any(abs(a['position'] - b['position']) > RETURN_MOVED_RAD
                       for a, b in zip(shown['joints'], offer['joints']))):
            raise ValueError('The arm moved since its pose was shown: look at it again, then confirm')
        power_ok()
        await take_motors()  # The live receiver needs the CAN links; the reader must be down.
        tune.start_return(offer['arm'], {row['motor']: row['position'] for row in offer['joints']})
        return state_response()

    async def tune_key(request):
        payload = await body(request)
        need_tune()
        if set(payload) != {'key'}:
            raise ValueError('Expected key')
        tune.key(payload['key'])
        return state_response()

    async def tune_stop(request):
        await body(request)
        need_tune()
        await off_loop(tune.stop)
        return state_response()

    async def tune_save(request):
        payload = await body(request)
        need_tune()
        if set(payload) != {'motors'} or not isinstance(payload['motors'], list):
            raise ValueError('Expected motors')
        await off_loop(lambda: tune.save(payload['motors']))
        return state_response()

    async def check_live(request):
        payload = await body(request)
        need_tune()
        if check is None:
            raise ValueError('Helm was started without Minerva Check')
        if not set(payload) <= {'arm', 'confirm'} or payload.get('arm') not in ('left', 'right'):
            raise ValueError('Expected arm (left or right)')
        if teleop is not None and teleop.active:
            raise ValueError('Stop the teleop session before the live joint sweep')
        check.live_ready(payload['arm'])
        power_ok()
        await take_motors()
        tune.configure(arm=payload['arm'], mode='live', verify=True)
        tune.start(confirm=payload.get('confirm') is True)
        return state_response()

    async def startup(_app):
        if live:
            live_view.start()
        if ups is not None:
            ups.start()

    async def shutdown(_app):
        # Preserve guarded motor shutdown priority; preview clients never delay it.
        live_view.close()
        if ups is not None:
            await off_loop(ups.close)
        if tune is not None:
            await off_loop(tune.stop)
        if teleop is not None:
            await off_loop(teleop.stop)
        if leaders is not None:
            await off_loop(leaders.disconnect)
        if not simulation_only:
            await off_loop(link.disconnect)
        await asyncio.gather(*(asyncio.wait_for(
            ws.close(code=1001, message=b'Helm shutting down'), 1.)
            for ws in list(video_sockets)), return_exceptions=True)
        if camera_video is not None:
            await off_loop(camera_video.close)
        if episodes is not None:
            await off_loop(episodes.close)
        if head_setup is not None:
            await off_loop(head_setup.close)

    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    app.router.add_get('/api/helm/cameras/live', camera_live)
    app.router.add_get('/xr', xr_page)
    app.router.add_post('/api/helm/network/ap', network_ap)
    app.router.add_get('/api/helm/xr', xr_socket)
    app.router.add_post('/api/helm/xr/zero', xr_zero)
    app.router.add_post('/api/helm/xr/preview', xr_preview_switch)
    app.router.add_post('/api/helm/head/read', head_read)
    app.router.add_post('/api/helm/head/reference', head_reference)
    app.router.add_post('/api/helm/head/direction', head_direction)
    app.router.add_post('/api/helm/head/limits', head_limits)
    app.router.add_post('/api/helm/head/limits/save', head_limits_save)
    app.router.add_post('/api/helm/head/corner', head_corner)
    app.router.add_post('/api/helm/head/clearance/save', head_clearance_save)
    app.router.add_get('/api/helm/cameras', camera_get)
    app.router.add_post('/api/helm/cameras/{role}', camera_set)
    app.router.add_get('/api/helm/cameras/{role}/frame', camera_frame)
    app.router.add_get('/api/helm/state', state)
    if access is not None:
        app.router.add_get('/login', login_page)
        app.router.add_post('/api/helm/login', login)
        app.router.add_post('/api/helm/logout', logout)
        app.router.add_post('/api/helm/control/claim', control_claim)
        app.router.add_post('/api/helm/control/release', control_release)
    app.router.add_post('/api/helm/connect', connect)
    app.router.add_post('/api/helm/disconnect', disconnect)
    app.router.add_post('/api/helm/connection', connection)
    app.router.add_post('/api/helm/limits', limits)
    app.router.add_post('/api/helm/limits/save', save)
    app.router.add_post('/api/helm/direction', direction)
    app.router.add_post('/api/helm/task', task_action)
    app.router.add_post('/api/helm/teleop/start', teleop_start)
    app.router.add_post('/api/helm/teleop/stop', teleop_stop)
    app.router.add_post('/api/helm/teleop/key', teleop_key)
    app.router.add_post('/api/helm/teleop/pedal', teleop_pedal)
    app.router.add_post('/api/helm/teleop/grip_feedback', teleop_grip_feedback)
    app.router.add_post('/api/helm/teleop/route', teleop_route)
    app.router.add_post('/api/helm/teleop/tuning/{action}', teleop_manual_tuning)
    app.router.add_get('/api/helm/settings', settings_get)
    app.router.add_post('/api/helm/settings/{action}', settings_action)
    app.router.add_post('/api/helm/teleop/zero', teleop_zero)
    app.router.add_post('/api/helm/teleop/target', teleop_target)
    app.router.add_post('/api/helm/teleop/receiver', teleop_receiver)
    app.router.add_post('/api/helm/leaders/connect', leaders_connect)
    app.router.add_post('/api/helm/leaders/disconnect', leaders_disconnect)
    app.router.add_post('/api/helm/leaders/range', leaders_range)
    app.router.add_post('/api/helm/leaders/reference', leaders_reference)
    app.router.add_post('/api/helm/leaders/save', leaders_save)
    app.router.add_post('/api/helm/check/run', check_run)
    app.router.add_post('/api/helm/check/live', check_live)
    app.router.add_post('/api/helm/grippers/zero', gripper_zero_run)
    app.router.add_get('/api/helm/calibration/thumbnail', calibration_thumbnail)
    app.router.add_post('/api/helm/calibration/{action}', calibration_action)
    app.router.add_get('/api/helm/head-alignment/thumbnail', head_alignment_thumbnail)
    app.router.add_post('/api/helm/head-alignment/{action}', head_alignment_action)
    app.router.add_post('/api/helm/twin/{action}', twin_action)
    app.router.add_get('/api/helm/maps', maps_listing)
    app.router.add_get('/api/helm/maps/{name}/map.splat', map_splat)
    app.router.add_post('/api/helm/maps/capture', map_capture)
    app.router.add_post('/api/helm/tune/configure', tune_configure)
    app.router.add_post('/api/helm/tune/start', tune_start)
    app.router.add_post('/api/helm/tune/return', tune_return)
    app.router.add_post('/api/helm/tune/key', tune_key)
    app.router.add_post('/api/helm/tune/stop', tune_stop)
    app.router.add_post('/api/helm/tune/save', tune_save)
    app.router.add_get('/api/helm/episodes', episodes_list)
    app.router.add_get('/api/helm/policies', policies_list)
    app.router.add_get('/api/helm/episodes/{session}/{episode}', episode_get)
    app.router.add_get('/api/helm/episodes/{session}/{episode}/frame/{index}/{role}', episode_frame)
    app.router.add_post('/api/helm/episodes/{session}/{episode}/review', episode_review)
    app.router.add_static('/helm', STATIC, show_index=False)
    return app
