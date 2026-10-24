// Browser-only day/night preference, like touch mode. Never sends requests or changes control availability.
// index.html applies the saved theme before the first paint; the embedded 3D view reads the same key itself and is
// told of changes here (same origin).
const preferenceKey = 'helm-theme';
const toggles = [...document.querySelectorAll('[data-theme-toggle]')];
const status = document.getElementById('theme-status');
const frame = document.getElementById('model-view');

function apply(night) {
  const theme = night ? 'dark' : 'light';
  document.documentElement.dataset.theme = theme;
  try { frame?.contentDocument?.documentElement.setAttribute('data-theme', theme); } catch { /* Not loaded yet. */ }
  for (const button of toggles) {
    button.setAttribute('aria-pressed', String(night));
    if (button.dataset.themeToggle === 'label') button.textContent = `Night mode: ${night ? 'On' : 'Off'}`;
    else button.title = night ? 'Night mode is on: switch to day' : 'Switch to night mode';
  }
}

const night = () => document.documentElement.dataset.theme === 'dark';
apply(night());
frame?.addEventListener('load', () => apply(night()));

for (const button of toggles) button.addEventListener('click', () => {
  const enabled = !night();
  apply(enabled);
  try {
    localStorage.setItem(preferenceKey, enabled ? 'dark' : 'light');
    status.textContent = 'Display preference saved in this browser.';
  } catch {
    status.textContent = 'Applied for this page. Browser storage is unavailable; this preference may reset on reload.';
  }
});

window.addEventListener('storage', event => {
  if (event.key === preferenceKey || event.key === null) {
    apply(event.newValue === 'dark');
    status.textContent = 'Display preference updated in another tab.';
  }
});
