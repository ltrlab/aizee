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
from pathlib import Path
import secrets

STATIC = Path(__file__).parent / 'static'
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

def create_app(session, link, *, teleop=None, leaders=None, check=None, tune=None, token=None, live=False,
               gripper_zero=None, simulation_only=False):
    """`live`: keep the stopped reader running whenever Minerva is online and nothing else holds the motors
    (helm.live_view); sessions then take the motors from it instead of asking for Disconnect first."""
    from aiohttp import web
    from robot_platform.viewer.server import create_app as viewer_app
    from .live_view import LiveView
    from .telemetry import select_telemetry
    token = token or secrets.token_urlsafe(24)
    if simulation_only and live:
        raise ValueError('Simulation-only Helm cannot start the physical live reader')
    if simulation_only:
        grippers = None
    else:
        from .gripper_zero import GripperZero
        grippers = gripper_zero or GripperZero(lambda: getattr(session, 'address', None) or session.device['address'])

    def owner():
        """What holds the motors, other than the reader."""
        if teleop is not None and teleop.active:
            return 'teleop'
        if tune is not None and tune.active:
            return {'verify': 'the live joint sweep', 'return': 'the return'}.get(tune.mode, 'the servo sweep')
        if check is not None and check.running:
            return 'Minerva Check'
        if grippers is not None and grippers.running:
            return 'the gripper zero'
        return None

    live_view = LiveView(session, link, owner)
    live_view.enabled = live

    async def take_motors():
        """The reader steps aside for a session that needs the motors (it starts again once they're free)."""
        if not simulation_only:
            await off_loop(live_view.yield_motors)

    def guide():
        for source in (teleop, tune):
            pose = source.guide() if source is not None else None
            if pose is not None:
                return pose
        return session.guide()

    app = viewer_app([session.model.profile], session.model.profile.id,
                     guide=guide, index_html=STATIC / 'index.html')

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
                '/api/helm/teleop/start', '/api/helm/teleop/stop', '/api/helm/teleop/key',
                '/api/helm/teleop/route', '/api/helm/teleop/zero', '/api/helm/teleop/target',
                '/api/helm/teleop/receiver', '/api/helm/settings/apply', '/api/helm/settings/reset',
                '/api/helm/leaders/connect', '/api/helm/leaders/disconnect', '/api/helm/leaders/range',
                '/api/helm/leaders/reference', '/api/helm/leaders/save',
                '/api/helm/teleop/tuning/apply', '/api/helm/teleop/tuning/save',
                '/api/helm/teleop/tuning/restore', '/api/helm/task',
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
        state['action_token'] = token
        return web.json_response(state, headers={'Cache-Control': 'no-store'})

    async def off_loop(function):
        return await asyncio.get_running_loop().run_in_executor(None, function)

    async def state(_request):
        return state_response()

    def not_checking():
        if grippers is not None and grippers.running:
            raise ValueError('A gripper zero is running; wait for it to finish')
        if check is not None and check.running:
            raise ValueError('Minerva Check is running; wait for it to finish')
        if tune is not None and tune.active:
            raise ValueError('The servo sweep is running; finish or stop it first')

    async def connect(request):
        await body(request)
        if teleop is not None and teleop.active:
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
        payload = await body(request)
        need_teleop()
        if leaders is not None and leaders.running:
            raise ValueError('Disconnect the leaders in the Leaders tab before starting teleop')
        not_checking()
        if simulation_only and (getattr(teleop, 'simulation', None) is None
                                or teleop.target != 'pc' or teleop.receiver['mode'] != 'virtual'):
            raise ValueError('Simulation mode needs the virtual receiver on this PC')
        await take_motors()
        teleop.start(confirm=payload.get('confirm') is True)
        return state_response()

    async def task_action(request):
        payload = await body(request)
        need_teleop()
        if set(payload) != {'action'} or payload['action'] not in ('setup', 'reset', 'start_record', 'stop_record'):
            raise ValueError('Expected task action: setup, reset, start_record or stop_record')
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
            if payload['connection'] != 'tether' and teleop.receiver['mode'] == 'live':
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
        if getattr(session, 'connection', 'tether') != 'tether':
            raise ValueError('Gripper zero runs over the wired tether; choose Tether first')
        motor, confirm = payload.get('motor'), payload.get('confirm') is True
        if not confirm:
            raise ValueError('Close the gripper fully by hand, then confirm the zero')
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
        await take_motors()  # Minerva Check reads the motors itself.
        check.start()
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
        await take_motors()
        tune.configure(arm=payload['arm'], mode='live', verify=True)
        tune.start(confirm=payload.get('confirm') is True)
        return state_response()

    async def startup(_app):
        if live:
            live_view.start()

    async def shutdown(_app):
        live_view.close()
        if tune is not None:
            await off_loop(tune.stop)
        if teleop is not None:
            await off_loop(teleop.stop)
        if leaders is not None:
            await off_loop(leaders.disconnect)
        if not simulation_only:
            await off_loop(link.disconnect)

    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    app.router.add_get('/api/helm/state', state)
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
    app.router.add_post('/api/helm/tune/configure', tune_configure)
    app.router.add_post('/api/helm/tune/start', tune_start)
    app.router.add_post('/api/helm/tune/return', tune_return)
    app.router.add_post('/api/helm/tune/key', tune_key)
    app.router.add_post('/api/helm/tune/stop', tune_stop)
    app.router.add_post('/api/helm/tune/save', tune_save)
    app.router.add_static('/helm', STATIC, show_index=False)
    return app
