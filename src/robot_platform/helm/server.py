"""Loopback-only Helm web app: the model viewer plus robot actions.

Actions need the per-process token served with the state, so another local page
cannot post them. The robot is never contacted until Connect is pressed. The
stopped-motor link and the (virtual) teleop session are mutually exclusive, so
the 3D view always shows exactly one source. The Leaders workspace and a teleop
session both need the leader USB ports, so they exclude each other too. Minerva
Check reads the motors (stopped) and maybe the leaders itself, so it runs alone.
"""
import asyncio
from pathlib import Path
import secrets

STATIC = Path(__file__).parent / 'static'


def create_app(session, link, *, teleop=None, leaders=None, check=None, token=None):
    from aiohttp import web
    from robot_platform.viewer.server import create_app as viewer_app
    token = token or secrets.token_urlsafe(24)

    def guide():
        pose = teleop.guide() if teleop is not None else None
        return pose if pose is not None else session.guide()

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
        return payload

    def state_response():
        state = session.snapshot()
        state['link'] = {'running': link.running}
        state['teleop'] = teleop.snapshot() if teleop is not None else None
        state['leaders'] = leaders.snapshot() if leaders is not None else None
        state['check'] = check.snapshot() if check is not None else None
        state['action_token'] = token
        return web.json_response(state, headers={'Cache-Control': 'no-store'})

    async def off_loop(function):
        return await asyncio.get_running_loop().run_in_executor(None, function)

    async def state(_request):
        return state_response()

    def not_checking():
        if check is not None and check.running:
            raise ValueError('Minerva Check is running; wait for it to finish')

    async def connect(request):
        await body(request)
        if teleop is not None and teleop.active:
            raise ValueError('Stop the teleop session before connecting to the motors')
        not_checking()
        link.connect()
        return state_response()

    async def disconnect(request):
        await body(request)
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
        await body(request)
        need_teleop()
        if link.running:
            raise ValueError('Disconnect from the motors before starting teleop')
        if leaders is not None and leaders.running:
            raise ValueError('Disconnect the leaders in the Leaders tab before starting teleop')
        not_checking()
        teleop.start()
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

    async def teleop_zero(request):
        await body(request)
        need_teleop()
        await off_loop(teleop.set_zero)
        return state_response()

    async def check_run(request):
        await body(request)
        if check is None:
            raise ValueError('Helm was started without Minerva Check')
        if link.running:
            raise ValueError('Disconnect the motors first: Minerva Check reads them itself')
        if teleop is not None and teleop.active:
            raise ValueError('Stop the teleop session before running Minerva Check')
        busy = (lambda: "Helm's Leaders tab" if leaders.running else None) if leaders is not None else None
        check.start(leaders_busy=busy)
        return state_response()

    async def shutdown(_app):
        if teleop is not None:
            await off_loop(teleop.stop)
        if leaders is not None:
            await off_loop(leaders.disconnect)
        await off_loop(link.disconnect)

    app.on_shutdown.append(shutdown)
    app.router.add_get('/api/helm/state', state)
    app.router.add_post('/api/helm/connect', connect)
    app.router.add_post('/api/helm/disconnect', disconnect)
    app.router.add_post('/api/helm/limits', limits)
    app.router.add_post('/api/helm/limits/save', save)
    app.router.add_post('/api/helm/direction', direction)
    app.router.add_post('/api/helm/teleop/start', teleop_start)
    app.router.add_post('/api/helm/teleop/stop', teleop_stop)
    app.router.add_post('/api/helm/teleop/key', teleop_key)
    app.router.add_post('/api/helm/teleop/route', teleop_route)
    app.router.add_post('/api/helm/teleop/zero', teleop_zero)
    app.router.add_post('/api/helm/leaders/connect', leaders_connect)
    app.router.add_post('/api/helm/leaders/disconnect', leaders_disconnect)
    app.router.add_post('/api/helm/leaders/range', leaders_range)
    app.router.add_post('/api/helm/leaders/reference', leaders_reference)
    app.router.add_post('/api/helm/leaders/save', leaders_save)
    app.router.add_post('/api/helm/check/run', check_run)
    app.router.add_static('/helm', STATIC, show_index=False)
    return app
