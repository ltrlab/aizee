// Offline real-browser contract for the day / night theme. Supply PLAYWRIGHT_MODULE and optionally BROWSER_EXECUTABLE.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import http from 'node:http';
import path from 'node:path';
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.resolve(path.dirname(new URL(import.meta.url).pathname.replace(/^\/([A-Z]:)/i, '$1')), '../..');
const evidence = path.join(root, 'build/theme-ui');
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
  errors = [];
const server = http
  .createServer(async (req, res) => {
    try {
      if (req.method !== 'GET') writes.push(req.url);
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
      if (req.url.startsWith('/viewer')) {
        res.end('<body>Offline model fixture</body>');
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
const base = `http://127.0.0.1:${server.address().port}/`;
const browser = await chromium.launch({
  headless: true,
  ...(process.env.BROWSER_EXECUTABLE ? { executablePath: process.env.BROWSER_EXECUTABLE } : {}),
});
const report = { fixture: 'Actual Helm browser modules; mocked read-only APIs and 3D view; no hardware', checks: [] };
try {
  const context = await browser.newContext({ viewport: { width: 1280, height: 800 } });
  const page = await context.newPage();
  page.on('pageerror', (error) => errors.push(String(error)));
  const look = () =>
    page.evaluate(() => ({
      theme: document.documentElement.dataset.theme,
      frame: document.getElementById('model-view').contentDocument?.documentElement.dataset.theme || 'light',
      pressed: [...document.querySelectorAll('[data-theme-toggle]')].map((button) =>
        button.getAttribute('aria-pressed'),
      ),
      label: document.querySelector('[data-theme-toggle=label]').textContent,
      saved: localStorage.getItem('helm-theme'),
      background: getComputedStyle(document.body).backgroundColor,
      stage: getComputedStyle(document.querySelector('.view')).backgroundImage,
      overflow: document.querySelector('header').scrollWidth - document.querySelector('header').clientWidth,
    }));
  await page.goto(base);
  await page.waitForFunction(() => document.querySelector('#model-view').contentDocument?.body?.textContent);
  let now = await look();
  assert.equal(now.theme, 'light');
  assert.equal(now.background, 'rgb(255, 255, 255)');
  assert.deepEqual(now.pressed, ['false', 'false']);
  assert.equal(now.saved, null);
  assert.equal(now.overflow, 0);
  assert.match(now.stage, /rgb\(255, 255, 255\)/);
  report.checks.push('Day (white, light 3D stage) by default with nothing saved; header fits 1280 px');
  await page.locator('header [data-theme-toggle]').click();
  now = await look();
  assert.equal(now.theme, 'dark');
  assert.equal(now.frame, 'dark');
  assert.equal(now.saved, 'dark');
  assert.equal(now.background, 'rgb(11, 12, 15)');
  assert.match(now.stage, /rgb\(31, 36, 44\)/);
  assert.deepEqual(now.pressed, ['true', 'true']);
  assert.equal(now.label, 'Night mode: On');
  assert.equal(now.overflow, 0);
  report.checks.push(
    'Header moon switches page, 3D stage and embedded 3D view to night (black), saved in this browser; header still fits',
  );
  await page.reload();
  await page.waitForFunction(() => document.querySelector('#model-view').contentDocument?.body?.textContent);
  now = await look();
  assert.equal(now.theme, 'dark');
  assert.equal(now.frame, 'dark');
  report.checks.push('Night survives a reload, page and 3D view');
  await page.evaluate(() => document.querySelector('[data-theme-toggle=label]').click());
  now = await look();
  assert.equal(now.theme, 'light');
  assert.equal(now.frame, 'light');
  assert.equal(now.saved, 'light');
  assert.equal(now.label, 'Night mode: Off');
  report.checks.push('Settings switch returns to day');
  const style = () =>
    page.evaluate(() => ({
      look: document.documentElement.dataset.look,
      saved: localStorage.getItem('helm-look'),
      frame: document.getElementById('model-view').contentDocument?.documentElement.dataset.look,
      sheets: [...document.querySelectorAll('link[data-look-sheet]')].map((link) => link.getAttribute('href')),
      background: getComputedStyle(document.body).backgroundColor,
      moon: getComputedStyle(document.querySelector('header [data-theme-toggle]')).display,
      night: getComputedStyle(document.getElementById('theme-preference')).display,
      pressed: [...document.querySelectorAll('[data-look-choice]')].map((button) =>
        button.getAttribute('aria-pressed'),
      ),
    }));
  let shown = await style();
  assert.equal(shown.look, 'retro');
  assert.deepEqual(shown.pressed, ['true', 'false']);
  await page.evaluate(() => document.querySelector('[data-look-choice=classic]').click());
  await page.waitForFunction(() => getComputedStyle(document.body).backgroundColor === 'rgb(11, 12, 14)');
  shown = await style();
  assert.equal(shown.look, 'classic');
  assert.equal(shown.frame, 'classic');
  assert.equal(shown.saved, 'classic');
  assert.deepEqual(shown.sheets, ['helm/classic-helm.css', 'helm/classic-cameras.css', 'helm/classic-replay.css']);
  assert.equal(shown.moon, 'none');
  assert.equal(shown.night, 'none');
  assert.deepEqual(shown.pressed, ['false', 'true']);
  report.checks.push(
    'Settings → Look switches live to the original look (its stylesheets, day / night hidden), saved in this browser',
  );
  await page.reload();
  await page.waitForFunction(() => document.querySelector('#model-view').contentDocument?.body?.textContent);
  shown = await style();
  assert.equal(shown.look, 'classic');
  assert.equal(shown.background, 'rgb(11, 12, 14)');
  assert.equal(shown.frame, 'classic');
  await page.evaluate(() => document.querySelector('[data-look-choice=retro]').click());
  await page.waitForFunction(() => getComputedStyle(document.body).backgroundColor === 'rgb(255, 255, 255)');
  shown = await style();
  assert.equal(shown.look, 'retro');
  assert.equal(shown.saved, 'retro');
  assert.equal(shown.frame, 'retro');
  report.checks.push('The original look survives a reload; RetroUI comes back the same way');
  assert.deepEqual(writes, []);
  assert.deepEqual(errors, []);
  report.checks.push('A display preference never reaches the server; no browser errors');
  report.passed = true;
} finally {
  await browser.close();
  await new Promise((resolve) => server.close(resolve));
  await fs.writeFile(path.join(evidence, 'report.json'), JSON.stringify(report, null, 2));
}
console.log(JSON.stringify(report));
