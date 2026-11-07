// Browser-only presentation preference. Never sends requests or changes control availability.
const preferenceKey = 'helm-touch-layout';
const toggles = [...document.querySelectorAll('[data-touch-toggle]')];
const status = document.getElementById('touch-status');

function apply(enabled) {
  document.body.dataset.touch = enabled ? 'on' : 'off';
  for (const button of toggles) {
    button.setAttribute('aria-pressed', String(enabled));
    button.textContent = `Touch mode: ${enabled ? 'On' : 'Off'}`;
  }
}

let saved = false;
try {
  saved = localStorage.getItem(preferenceKey) === 'on';
} catch {
  /* Storage may be unavailable. */
}
apply(saved);

for (const button of toggles)
  button.addEventListener('click', () => {
    const enabled = document.body.dataset.touch !== 'on';
    apply(enabled);
    try {
      localStorage.setItem(preferenceKey, enabled ? 'on' : 'off');
      status.textContent = 'Display preference saved in this browser.';
    } catch {
      status.textContent =
        'Applied for this page. Browser storage is unavailable; this preference may reset on reload.';
    }
  });

window.addEventListener('storage', (event) => {
  if (event.key === preferenceKey || event.key === null) {
    apply(event.newValue === 'on');
    status.textContent = 'Display preference updated in another tab.';
  }
});
