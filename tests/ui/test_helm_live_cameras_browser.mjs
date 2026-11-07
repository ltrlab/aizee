// Offline real-browser contract. Supply PLAYWRIGHT_MODULE and optionally BROWSER_EXECUTABLE.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.resolve(path.dirname(new URL(import.meta.url).pathname.replace(/^\/([A-Z]:)/i, '$1')), '../..');
const evidence = path.join(root, 'build/live-camera-ui');
await fs.mkdir(evidence, { recursive: true });
const state = {
  mode: 'robot',
  action_token: 'offline-fixture',
  robot: { model: 'Minerva', asset_id: 'fixture', address: 'offline', connection: 'tether', connections: [] },
  feed: { state: 'disconnected' },
  limits: { rows: [], path: 'fixture/limits.json', pending: 0 },
  session: {},
  link: { running: false },
  teleop: { phase: 'stopped', state: 'DISABLED', joints: [], target: 'jetson', receiver_choice: { mode: 'virtual' } },
  telemetry: { rows: [] },
};
const writes = [],
  reads = [],
  errors = [];
const server = http
  .createServer(async (req, res) => {
    try {
      if (req.method !== 'GET') writes.push(req.url);
      reads.push(req.url);
      if (req.url === '/api/helm/state') {
        res.setHeader('Content-Type', 'application/json');
        res.end(JSON.stringify(state));
        return;
      }
      if (req.url === '/api/helm/settings') {
        res.setHeader('Content-Type', 'application/json');
        res.end(JSON.stringify({ sections: [], revision: 1, session_id: 'fixture', can_apply: false }));
        return;
      }
      if (req.url === '/api/helm/cameras') {
        res.setHeader('Content-Type', 'application/json');
        res.end(
          JSON.stringify({
            can_apply: true,
            cameras: { right_wrist: { available: true, controls: { gain: { value: 30, min: 0, max: 100, step: 1 } } } },
          }),
        );
        return;
      }
      if (req.url.startsWith('/viewer')) {
        res.end('<body style="background:#15191e;color:white">Offline model fixture</body>');
        return;
      }
      const name = req.url === '/' ? 'index.html' : req.url.replace(/^\/helm\//, '');
      if (!/^[a-z_.-]+$/.test(name)) {
        res.writeHead(404);
        res.end();
        return;
      }
      res.setHeader(
        'Content-Type',
        name.endsWith('.js') ? 'text/javascript' : name.endsWith('.css') ? 'text/css' : 'text/html',
      );
      res.end(await fs.readFile(path.join(root, 'src/robot_platform/helm/static', name)));
    } catch {
      res.writeHead(404);
      res.end();
    }
  })
  .listen(0, '127.0.0.1');
await new Promise((resolve) => server.once('listening', resolve));
const browser = await chromium.launch({
  headless: true,
  ...(process.env.BROWSER_EXECUTABLE ? { executablePath: process.env.BROWSER_EXECUTABLE } : {}),
});
const report = {
  fixture: 'Actual Helm browser modules; mocked read-only websocket and APIs; synthetic JPEGs; no hardware',
  checks: [],
};
try {
  const context = await browser.newContext({ viewport: { width: 800, height: 480 }, hasTouch: true });
  await context.addInitScript(() => {
    localStorage.setItem('helm-touch-layout', 'on');
    localStorage.setItem('helm-tab', 'teleop');
    window.fixtureSockets = [];
    window.fixtureUrls = new Set();
    window.fixtureCreated = 0;
    window.fixtureHidden = false;
    Object.defineProperty(document, 'hidden', { get: () => window.fixtureHidden });
    const create = URL.createObjectURL.bind(URL),
      revoke = URL.revokeObjectURL.bind(URL);
    URL.createObjectURL = (blob) => {
      const url = create(blob);
      fixtureUrls.add(url);
      fixtureCreated++;
      return url;
    };
    URL.revokeObjectURL = (url) => {
      fixtureUrls.delete(url);
      revoke(url);
    };
    window.WebSocket = class {
      static OPEN = 1;
      constructor(url) {
        this.url = String(url);
        this.readyState = 0;
        this.messages = [];
        fixtureSockets.push(this);
        setTimeout(() => {
          if (this.readyState === 0) {
            this.readyState = 1;
            this.onopen?.();
          }
        }, 0);
      }
      send(text) {
        const message = JSON.parse(text);
        if (message.type !== 'heartbeat') throw Error('Unexpected write');
        this.messages.push(message);
        this.heartbeat = message.sequence;
      }
      close() {
        this.readyState = 3;
        setTimeout(() => this.onclose?.(), 0);
      }
      emit(data) {
        if (this.readyState === 1) this.onmessage?.({ data });
      }
    };
    const decode = HTMLImageElement.prototype.decode;
    HTMLImageElement.prototype.decode = function () {
      const delay = window.fixtureDecodeDelay || 0;
      return new Promise((resolve) => setTimeout(resolve, delay)).then(() => decode.call(this));
    };
    window.fixtureFrame = (role, sequence, age_s = 0.02, heartbeat) => {
      const socket = fixtureSockets.at(-1),
        canvas = document.createElement('canvas');
      canvas.width = 640;
      canvas.height = 480;
      const ctx = canvas.getContext('2d');
      ctx.fillStyle = { left_wrist: '#168b90', right_wrist: '#cb8523', head: '#375eb4' }[role];
      ctx.fillRect(0, 0, 640, 480);
      ctx.fillStyle = 'white';
      ctx.font = '36px sans-serif';
      ctx.fillText(role, 24, 80);
      const jpeg = Uint8Array.from(atob(canvas.toDataURL('image/jpeg').split(',')[1]), (c) => c.charCodeAt(0));
      const header = new TextEncoder().encode(
        JSON.stringify({
          role,
          sequence,
          age_s,
          source_timestamp: 1234,
          viewer_heartbeat: heartbeat ?? socket.heartbeat,
        }),
      );
      const packet = new Uint8Array(4 + header.length + jpeg.length);
      new DataView(packet.buffer).setUint32(0, header.length);
      packet.set(header, 4);
      packet.set(jpeg, 4 + header.length);
      socket.emit(packet.buffer);
    };
    window.fixtureStatus = (fresh = true) => {
      const s = fixtureSockets.at(-1);
      s.emit(
        JSON.stringify({
          type: 'status',
          stale_after_s: 1,
          target_fps: 10,
          viewer_heartbeat: s.heartbeat,
          cameras: Object.fromEntries(
            ['left_wrist', 'right_wrist', 'head'].map((role) => [
              role,
              { fresh, age_s: 0.02, sequence: 1, error: fresh ? null : 'Camera disconnected' },
            ]),
          ),
        }),
      );
    };
  });
  const page = await context.newPage();
  page.on('pageerror', (e) => errors.push(String(e)));
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.waitForFunction(() => fixtureSockets.at(-1)?.heartbeat);
  assert.equal(await page.locator('#model-view').getAttribute('src'), 'about:blank');
  assert.equal(await page.locator('#teleop-session').textContent(), 'Start session');
  assert.equal(reads.filter((url) => url === '/api/helm/cameras').length, 0);
  await page.evaluate(() => {
    fixtureStatus();
    for (const role of ['left_wrist', 'right_wrist', 'head']) fixtureFrame(role, 1);
  });
  await page.waitForFunction(() => document.querySelectorAll('[data-camera-role][data-fresh=true]').length === 3);
  report.checks.push(
    'All three feeds render before session start; model iframe unloaded; no camera settings/snapshot reads',
  );
  for (const [width, height] of [
    [800, 480],
    [1024, 600],
  ]) {
    await page.setViewportSize({ width, height });
    state.teleop.phase = 'running';
    state.teleop.state = 'HOLD';
    await page.waitForFunction(() => !document.getElementById('global-controls').hidden);
    await page.evaluate(() => {
      for (const role of ['left_wrist', 'right_wrist', 'head']) fixtureFrame(role, Math.floor(performance.now()));
    });
    for (const id of ['global-hold', 'global-shutdown', 'global-disable']) {
      const box = await page.locator('#' + id).boundingBox();
      assert(box && box.height >= 48 && box.x >= 0 && box.x + box.width <= width && box.y + box.height <= height);
    }
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
    await page.screenshot({ path: path.join(evidence, `teleop-${width}x${height}.png`) });
    const connections = await page.evaluate(() => fixtureSockets.length);
    await page.locator('.tab[data-tab=settings]').tap();
    await page.locator('[data-settings-section=cameras]').tap();
    assert(await page.locator('#camera-live-view').isVisible());
    assert.equal(reads.filter((url) => url === '/api/helm/cameras').length, 0);
    await page.screenshot({ path: path.join(evidence, `settings-${width}x${height}.png`) });
    await page.locator('.tab[data-tab=teleop]').tap();
    // Navigating through another Settings category closes; direct visible Cameras↔Teleop shares the socket.
    const returning = await page.evaluate(() => fixtureSockets.length);
    await page.locator('.tab[data-tab=settings]').tap();
    await page.locator('.tab[data-tab=teleop]').tap();
    assert.equal(await page.evaluate(() => fixtureSockets.length), returning);
  }
  report.checks.push(
    '800x480 and1024x600: camera/settings views, no horizontal overflow, existing Hold/Return/Disable visible',
  );
  await page.locator('[data-camera-role=head]').tap();
  assert.equal(await page.locator('#camera-focus-title').textContent(), 'Head');
  await page.waitForFunction(() => document.querySelectorAll('[data-camera-role][data-fresh=true]').length === 0);
  assert(await page.locator('#camera-focus-image').isHidden());
  assert.equal(await page.evaluate(() => fixtureUrls.size), 0);
  report.checks.push('Stalled frames expire, pixels clear, URLs released');
  const oldHeartbeat = await page.evaluate(() => fixtureSockets.at(-1).heartbeat);
  await page.waitForTimeout(1100);
  await page.evaluate((token) => fixtureFrame('head', 100000, 0.01, token), oldHeartbeat);
  assert(await page.locator('#camera-focus-image').isHidden());
  report.checks.push('Buffered frame with old viewer heartbeat cannot become live');
  await page.evaluate(() => {
    fixtureDecodeDelay = 250;
    fixtureCreatedBefore = fixtureCreated;
    for (let i = 200000; i < 200020; i++) fixtureFrame('head', i);
  });
  assert.equal(await page.evaluate(() => fixtureCreated - fixtureCreatedBefore), 1);
  await page.waitForFunction(() => document.querySelector('[data-camera-role=head]').dataset.fresh === 'true');
  await page.evaluate(() => {
    fixtureDecodeDelay = 0;
    fixtureStatus(false);
  });
  assert(await page.locator('#camera-focus-image').isHidden());
  report.checks.push('One decode per role, flooding dropped without backlog; stale status clears pixels');
  await page.locator('[data-preview-view=model]').tap();
  assert.equal(await page.locator('#model-view').getAttribute('src'), 'viewer?embed=1');
  assert.equal(await page.evaluate(() => fixtureSockets.at(-1).readyState), 3);
  assert.equal(await page.evaluate(() => fixtureUrls.size), 0);
  await page.locator('[data-preview-view=cameras]').tap();
  await page.waitForFunction(() => fixtureSockets.at(-1)?.heartbeat && fixtureSockets.at(-1).readyState === 1);
  await page.locator('#camera-pause').tap();
  assert.equal(await page.evaluate(() => fixtureSockets.at(-1).readyState), 3);
  await page.locator('#camera-pause').tap();
  await page.waitForFunction(() => fixtureSockets.at(-1).readyState === 1);
  await page.evaluate(() => {
    fixtureHidden = true;
    document.dispatchEvent(new Event('visibilitychange'));
  });
  assert.equal(await page.evaluate(() => fixtureSockets.at(-1).readyState), 3);
  await page.evaluate(() => {
    fixtureHidden = false;
    document.dispatchEvent(new Event('visibilitychange'));
  });
  await page.waitForFunction(() => fixtureSockets.at(-1).readyState === 1);
  report.checks.push('Model restore, pause/resume, document hide/show close and reopen only visible subscriptions');
  let count = await page.evaluate(() => fixtureSockets.length);
  await page.evaluate(() => fixtureSockets.at(-1).close());
  await page.waitForFunction((n) => fixtureSockets.length === n + 1 && fixtureSockets.at(-1).readyState === 1, count);
  await page.locator('.tab[data-tab=limits]').tap();
  assert.equal(await page.evaluate(() => fixtureSockets.at(-1).readyState), 3);
  await page.locator('.tab[data-tab=teleop]').tap();
  await page.waitForFunction(() => fixtureSockets.at(-1).readyState === 1);
  report.checks.push('Disconnect clears/reconnects; leaving preview workspaces closes stream');
  for (let i = 0; i < 4; i++) {
    count = await page.evaluate(() => fixtureSockets.length);
    await page.evaluate(() => fixtureSockets.at(-1).close());
    if (i < 3)
      await page.waitForFunction(
        (n) => fixtureSockets.length === n + 1 && fixtureSockets.at(-1).readyState === 1,
        count,
      );
  }
  await page.waitForFunction(() => !document.getElementById('camera-retry').hidden);
  count = await page.evaluate(() => fixtureSockets.length);
  await page.waitForTimeout(1200);
  assert.equal(await page.evaluate(() => fixtureSockets.length), count);
  await page.locator('#camera-retry').tap();
  await page.waitForFunction((n) => fixtureSockets.length === n + 1 && fixtureSockets.at(-1).readyState === 1, count);
  report.checks.push('Exactly three automatic retries then explicit Retry; no unbounded reconnect loop');
  state.teleop.phase = 'stopped';
  await page.locator('.tab[data-tab=settings]').tap();
  await page.locator('[data-settings-section=cameras]').tap();
  await page.locator('#camera-refresh').tap();
  await page.locator('#camera-right_wrist-gain').waitFor();
  await page.waitForFunction(() => !document.getElementById('camera-right_wrist-gain').disabled);
  await page.locator('#camera-right_wrist-gain').fill('37');
  await page.evaluate(() => {
    fixtureStatus();
    fixtureFrame('right_wrist', 900000);
  });
  await page.waitForTimeout(300);
  assert.equal(await page.locator('#camera-right_wrist-gain').inputValue(), '37');
  assert.equal(reads.filter((url) => url === '/api/helm/cameras').length, 1);
  report.checks.push('Exposure/gain draft survives live frames; controls fetched only by explicit Read settings');
  state.mode = 'simulation';
  state.teleop.phase = 'stopped';
  await page.waitForFunction(() => document.body.dataset.mode === 'simulation');
  assert(await page.locator('#camera-live-view').isHidden());
  assert(await page.locator('[data-preview-view=cameras]').isDisabled());
  assert.equal(await page.evaluate(() => fixtureSockets.at(-1).readyState), 3);
  assert.deepEqual(writes, []);
  assert(!reads.some((url) => url.endsWith('/frame')));
  assert.deepEqual(errors, []);
  report.checks.push('Simulation rejects physical preview; zero mutation or snapshot requests; no browser errors');
  report.passed = true;
} finally {
  await browser.close();
  await new Promise((resolve) => server.close(resolve));
  await fs.writeFile(path.join(evidence, 'report.json'), JSON.stringify(report, null, 2));
}
console.log(JSON.stringify(report));
