// Minerva's UPS in Helm without the DOM: the header's battery pill and the banner (helm.ups, ops.ups).

const INSTALL = 'cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.ups install';

function percentOf(value) {
  return Number.isFinite(value) ? Math.max(0, Math.min(100, Math.round(value))) : null;
}

export function duration(minutes) {
  if (!Number.isFinite(minutes) || minutes < 0) return null;
  if (minutes < 60) return `${Math.round(minutes)} min`;
  const hours = Math.floor(minutes / 60),
    rest = Math.round(minutes % 60);
  return rest ? `${hours} h ${rest} min` : `${hours} h`;
}

function details(state) {
  const parts = [];
  if (Number.isFinite(state.voltage_v)) parts.push(`${state.voltage_v.toFixed(2)} V`);
  if (Number.isFinite(state.current_a)) parts.push(`${state.current_a > 0 ? '+' : ''}${state.current_a.toFixed(2)} A`);
  if (Number.isFinite(state.power_w)) parts.push(`${state.power_w.toFixed(1)} W`);
  if (Number.isFinite(state.cell_v)) parts.push(`${state.cell_v.toFixed(2)} V a cell`);
  return parts.join(' · ');
}

// The pill is a phone-style battery: a glyph and a few characters (the header has no room for more); the words
// go in its title and in the banner, which shows whenever the battery needs the operator.
// {hidden, tone (good, warn, bad, muted), text, title, level (0-100 or null), charging, banner: null or {text, tone}}
export function upsView(ups) {
  const view = {
    hidden: false,
    tone: 'muted',
    text: '…',
    title: "Reading Minerva's UPS",
    level: null,
    charging: false,
    banner: null,
  };
  if (!ups) return { ...view, hidden: true }; // Simulation: no robot, no UPS.
  if (ups.status === 'unreachable') {
    const last = ups.last;
    const lastPercent = percentOf(last?.percent);
    const wasOnBattery = last?.source === 'battery';
    view.text = wasOnBattery && lastPercent !== null ? `${lastPercent}%?` : '—';
    view.title =
      `No UPS reading: Minerva is offline or unreachable${ups.error ? ` (${ups.error})` : ''}.` +
      (last ? ` Last reading ${duration((last.seen_s_ago || 0) / 60) || 'moments'} ago: ${details(last)}.` : '');
    if (wasOnBattery) {
      view.tone = 'bad';
      view.level = lastPercent;
      view.banner = {
        tone: 'bad',
        text:
          `Minerva went offline while running on its UPS battery` +
          `${lastPercent !== null ? ` (last about ${lastPercent}%)` : ''}. It may have shut down or run out: ` +
          'plug in the UPS charger, then power the Jetson on.',
      };
    }
    return view;
  }
  if (ups.status === 'missing') {
    return { ...view, tone: 'warn', text: '—', title: `Minerva has no UPS monitor running. On the Jetson: ${INSTALL}` };
  }
  if (ups.status === 'stale') {
    return {
      ...view,
      tone: 'warn',
      text: '—',
      title:
        `Minerva's UPS service stopped writing${Number.isFinite(ups.age_s) ? ` ${Math.round(ups.age_s)} s ago` : ''}. ` +
        'On the Jetson: systemctl status minerva-ups',
    };
  }
  if (ups.status !== 'ok' || !ups.state) return view;
  const state = ups.state;
  if (state.sensor === 'starting') return view;
  if (state.sensor === 'error') {
    return {
      ...view,
      tone: 'warn',
      text: '!',
      title: `Minerva's UPS monitor cannot read the battery: ${state.error || 'no reading'}. It cannot warn or shut down until it reads again.`,
    };
  }
  const percent = percentOf(state.percent);
  const limits = state.thresholds || {};
  const cutoff = Number.isFinite(limits.shutdown_percent)
    ? `${Math.round(limits.shutdown_percent)}%`
    : 'its shutdown level';
  const left = duration(state.minutes_left);
  view.level = percent;
  view.title = `${details(state)} · about ${percent}% charge`;
  const shutdown = state.shutdown;
  if (shutdown) {
    const seconds = Number.isFinite(ups.poweroff_in_s) ? Math.max(0, Math.round(ups.poweroff_in_s)) : null;
    const powering = shutdown.stage === 'powering_off';
    view.tone = 'bad';
    view.text = powering || seconds === null ? 'Off' : `${seconds} s`;
    view.title = `Shutting down: ${view.title}`;
    view.banner = {
      tone: 'bad',
      text: powering
        ? `Minerva's UPS battery is critical (${shutdown.reason || 'low'}): the Jetson is powering off now.`
        : `Minerva's UPS battery is critical (${shutdown.reason || 'low'}). Helm ends any session, lowering the arms first, ` +
          `and the Jetson powers off${seconds !== null ? ` within ${seconds} s` : ' shortly'}. ` +
          'Plug in the UPS charger now to cancel.',
    };
    return view;
  }
  if (state.source === 'battery') {
    const low = state.level === 'low' || state.level === 'critical';
    view.tone = low ? 'bad' : 'warn';
    view.text = `${percent}%`;
    view.title = `On battery${left ? `, about ${left} to shutdown` : ''}: ${view.title}`;
    view.banner = {
      tone: low ? 'bad' : 'warn',
      text:
        (low
          ? `Minerva's UPS battery is low: about ${percent}%`
          : `Minerva is running on its UPS battery: about ${percent}%`) +
        `${left ? `, roughly ${left} before it shuts itself down at ${cutoff}` : `; it shuts itself down at ${cutoff}`}. ` +
        (low ? 'Plug in the UPS charger now.' : 'Plug in the UPS charger.'),
    };
    return view;
  }
  view.tone = 'good';
  view.charging = state.charge === 'charging';
  view.text = state.charge === 'full' ? 'Full' : `${percent}%`;
  view.title = `On the charger${view.charging ? ', charging' : ''}: ${view.title}`;
  return view;
}
