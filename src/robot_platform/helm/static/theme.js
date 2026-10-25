// Browser-only display preferences, like touch mode. Never sends requests or changes control availability.
// The look: RetroUI (helm.css, cameras.css, replay.css) or Helm's original (their classic- copies), and, in RetroUI,
// day or night. index.html applies both before the first paint; the embedded 3D view reads the same keys itself and
// is told of changes here (same origin).
const themeKey = 'helm-theme', lookKey = 'helm-look';
const root = document.documentElement;
const toggles = [...document.querySelectorAll('[data-theme-toggle]')];
const choices = [...document.querySelectorAll('[data-look-choice]')];
const themeStatus = document.getElementById('theme-status'), lookStatus = document.getElementById('look-status');
const frame = document.getElementById('model-view');

function frameRoot() {
  try { return frame?.contentDocument?.documentElement || null; } catch { return null; /* Not loaded yet. */ }
}

function apply(night) {
  const theme = night ? 'dark' : 'light';
  root.dataset.theme = theme;
  frameRoot()?.setAttribute('data-theme', theme);
  window.dispatchEvent(new CustomEvent('helm-theme', {detail: theme}));
  for (const button of toggles) {
    button.setAttribute('aria-pressed', String(night));
    if (button.dataset.themeToggle === 'label') button.textContent = `Night mode: ${night ? 'On' : 'Off'}`;
    else button.title = night ? 'Night mode is on: switch to day' : 'Switch to night mode';
  }
}

function applyLook(look) {
  const classic = look === 'classic';
  root.dataset.look = classic ? 'classic' : 'retro';
  for (const link of document.querySelectorAll('link[data-look-sheet]')) {
    const href = `helm/${classic ? 'classic-' : ''}${link.dataset.lookSheet}.css`;
    if (link.getAttribute('href') !== href) link.setAttribute('href', href);
  }
  const viewer = frameRoot();
  if (viewer) {
    viewer.setAttribute('data-look', root.dataset.look);
    const sheet = viewer.ownerDocument.querySelector('link[href$="viewer-theme.css"]');
    if (sheet) sheet.disabled = classic;  // The original 3D view had no Helm sheet.
  }
  for (const button of choices) button.setAttribute('aria-pressed', String(button.dataset.lookChoice === root.dataset.look));
  window.dispatchEvent(new CustomEvent('helm-look', {detail: root.dataset.look}));
}

const night = () => root.dataset.theme === 'dark';
apply(night());
applyLook(root.dataset.look);
frame?.addEventListener('load', () => { apply(night()); applyLook(root.dataset.look); });

function save(key, value, status) {
  try {
    localStorage.setItem(key, value);
    status.textContent = 'Display preference saved in this browser.';
  } catch {
    status.textContent = 'Applied for this page. Browser storage is unavailable; this preference may reset on reload.';
  }
}

for (const button of toggles) button.addEventListener('click', () => {
  const enabled = !night();
  apply(enabled);
  save(themeKey, enabled ? 'dark' : 'light', themeStatus);
});
for (const button of choices) button.addEventListener('click', () => {
  applyLook(button.dataset.lookChoice);
  save(lookKey, root.dataset.look, lookStatus);
});

window.addEventListener('storage', event => {
  if (event.key === themeKey || event.key === null) {
    apply(event.newValue === 'dark');
    themeStatus.textContent = 'Display preference updated in another tab.';
  }
  if (event.key === lookKey || event.key === null) {
    applyLook(event.newValue === 'classic' ? 'classic' : 'retro');
    lookStatus.textContent = 'Display preference updated in another tab.';
  }
});
