// Minerva's network in Helm without the DOM: the header's network pill and Settings → Network (helm/network.py, the
// state robot_platform.ops.network writes). The modes: on a network by its cable (wired), a tether to a PC's USB
// Ethernet adapter at 10.42.0.1, Wi-Fi, and the access point Minerva's Wi-Fi becomes, also reached at 10.42.0.1.

const MODES = {
  lan: 'Wired',
  tether: 'Tether',
  probing: 'Checking',
  unplugged: 'Wi-Fi',
  no_address: 'No address',
  starting: 'Starting',
};

const plain = (address) => String(address).split('/')[0];

// {hidden, tone, text, title, mode, lines, apLabel, apOn, apEnabled, apConfirm}
export function networkView(network, { host = 'aizee-rover', busy = false } = {}) {
  if (!network) return { hidden: true }; // Helm on a PC: the service runs on Minerva.
  if (!network.installed) {
    return {
      hidden: true,
      mode: 'Not installed',
      apEnabled: false,
      lines: [
        'The network service is not installed on Minerva: on the Jetson, run ' +
          'cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.network install',
      ],
    };
  }
  const wired = (network.ethernet?.addresses || []).map(plain);
  const wifi = (network.wifi?.addresses || []).map(plain);
  const ap = network.ap || {};
  const lines = [];
  let text = MODES[network.mode] || network.mode,
    tone = 'good';
  if (network.mode === 'lan') {
    lines.push(
      `On a network by its cable: ${wired.join(', ') || 'no address yet'}. Minerva's Wi-Fi steps aside meanwhile.`,
    );
  } else if (network.mode === 'tether') {
    lines.push("Cable to a PC's USB Ethernet adapter: Minerva is 10.42.0.1 there, and hands the PC an address.");
  } else if (network.mode === 'unplugged') {
    text = wifi.length ? 'Wi-Fi' : ap.on ? 'AP only' : 'No network';
    tone = wifi.length || ap.on ? 'good' : 'warn';
    lines.push(wifi.length ? `No cable: on Wi-Fi at ${wifi.join(', ')}.` : 'No cable, and no Wi-Fi network joined.');
  } else if (network.mode === 'no_address') {
    tone = 'bad';
    lines.push(
      'The cable is on a network whose DHCP server did not answer: Minerva serves no addresses there. ' +
        'Replug the cable to try again.',
    );
  } else {
    tone = 'muted';
    lines.push('Finding out what the Ethernet cable is plugged into…');
  }
  if (ap.on) {
    text = network.mode === 'lan' ? 'Wired + AP' : network.mode === 'tether' ? 'Tether + AP' : 'AP';
    lines.push(`Access point "${ap.ssid}" is on: its devices reach Minerva at ${network.local}.`);
  } else if (ap.wanted) {
    tone = tone === 'bad' ? 'bad' : 'warn';
    lines.push(
      ap.failed
        ? `The access point "${ap.ssid}" did not start: Minerva tries again every 30 s ` +
            '(journalctl -u minerva-net on the Jetson says why).'
        : `Switching the access point "${ap.ssid}" on…`,
    );
  }
  if (!network.running) {
    tone = 'bad';
    lines.unshift(`The network service has not reported for ${network.age_s ?? '?'} s: is it running?`);
  }
  // Where Helm opens: the wired address on a network, 10.42.0.1 from the tether or the access point (never the
  // access point's own 10.42.1.1), and the Wi-Fi address when Minerva is a client on a network.
  const reach = [
    ...new Set([
      ...(network.mode === 'lan' ? wired : []),
      ...(network.mode === 'tether' || ap.on ? [network.local] : []),
      ...(network.wifi?.mode === 'client' ? wifi : []),
    ]),
  ];
  if (reach.length)
    lines.push(
      `Open Helm at ${reach.map((address) => `https://${address}:8760`).join(' or ')}` +
        ` (or https://${host}.local:8760).`,
    );
  const on = Boolean(ap.wanted);
  return {
    hidden: false,
    tone,
    text,
    mode: MODES[network.mode] || network.mode,
    lines,
    title: lines.join(' '),
    apOn: on,
    apLabel: on ? 'Turn the access point off' : 'Turn the access point on',
    apEnabled: !busy && network.running,
    apConfirm: on
      ? 'Turn the access point off? Devices on it lose Minerva; its Wi-Fi rejoins its network unless the cable is on one.'
      : `Turn the access point "${ap.ssid}" on? Minerva's Wi-Fi leaves its network while the access point is on: ` +
        `devices reaching Minerva over that Wi-Fi lose it. Join "${ap.ssid}" and open https://${network.local}:8760.`,
  };
}

// The headset's /xr links (helm/network.py): on the network Minerva is on (its wired address, or its Wi-Fi client's),
// and on its access point (its local address, while the access point is on); a headset's browser refuses Helm at an
// address its certificate does not name. {links: [{mode, url, live, text, tone}], hint}
const CERTIFICATE = 'cd ~/minerva/current && PYTHONPATH=src python3 -m robot_platform.ops.helm_service certificate';
export function headsetLinks(network) {
  if (!network) return { links: [], hint: "Helm runs on this PC: open its /xr page at this PC's address." };
  if (!network.installed) return { links: [], hint: 'The network service is not installed on Minerva.' };
  const ssid = network.ap?.ssid || 'minerva';
  const links = (network.links || []).map((link) => {
    const where = link.mode === 'network' ? 'On the network' : `On the access point "${ssid}"`;
    const notes = [];
    if (!link.live) notes.push('off: turn it on in Settings → Network');
    if (link.certified === false) notes.push('the certificate does not name it');
    return {
      mode: link.mode,
      url: link.url,
      live: link.live,
      text: `${where}: ${link.url}${notes.length ? ` (${notes.join('; ')})` : ''}`,
      tone: link.certified === false ? 'bad' : link.live ? 'good' : 'muted',
    };
  });
  const given = network.links || [];
  const uncovered = given.filter((link) => link.certified === false).map((link) => link.address);
  let hint = '';
  if (!links.length) hint = 'No link yet: Minerva has no network address, and Helm serves no HTTPS here.';
  else if (uncovered.length) {
    hint =
      `Helm's certificate does not name ${uncovered.join(' or ')}: the headset's browser refuses it there. On ` +
      `the Jetson, run ${CERTIFICATE}, then helm_service restart, and accept the new certificate once on each device.`;
  } else if (given.some((link) => link.certified == null)) hint = "Helm's certificate could not be read.";
  return { links, hint };
}

// The GELLO pill: whether the bridge is connected, where, and whether over Wi-Fi.
export function bridgeView(bridge) {
  if (!bridge) return { hidden: true };
  if (!bridge.connected) {
    return {
      hidden: false,
      text: 'No GELLO',
      tone: 'warn',
      title:
        'The GELLO bridge is not connected: start it on the PC the leader arms are plugged into. Replay, ' +
        'policies and Check work without it.',
    };
  }
  const where = `${bridge.host}${bridge.peer ? ` (${bridge.peer})` : ''}`;
  const at = bridge.robot_address ? ` at ${bridge.robot_address}` : '';
  if (bridge.path === 'wifi') {
    return {
      hidden: false,
      text: 'GELLO · Wi-Fi',
      tone: 'warn',
      title:
        `The GELLO bridge on ${where} reaches Minerva${at} over Wi-Fi: leader teleop runs, but expect holds ` +
        'when samples arrive late. A cable (network or tether) is steadier.',
    };
  }
  return {
    hidden: false,
    text: 'GELLO',
    tone: 'good',
    title: `The GELLO bridge on ${where} serves the leader arms${at}${bridge.path === 'wired' ? ', wired' : ''}`,
  };
}
