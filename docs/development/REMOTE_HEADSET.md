# The headset on a slow or lossy link

Started 2026-11-03, after the first test from another network: a Quest Pro on a phone hotspot, over Tailscale, steered the
head well but showed about 2 video frames in 2 minutes. `ss -ti` on the Jetson showed 15-35 % of bytes retransmitted and
25-330 kbit/s delivered. One head view as JPEGs at 10 frames a second needs about 4 Mbit/s, over TCP, which resends what
is already out of date.

## What it does

Open the headset's page as `/xr?remote`. Everything else about the headset is unchanged, and the ordinary `/xr` still uses
its WebSockets.

| | `/xr` | `/xr?remote` |
|---|---|---|
| Head camera | JPEG over WebSocket | H.264 over WebRTC, rate-controlled |
| Poses | WebSocket | Unordered WebRTC channel, no resends; the socket until it opens |
| Status replies | WebSocket | The channel, while poses use it |
| 3D (depth) | Yes | No: the picture stays flat, since video carries no per-frame depth or head pose |
| Wrist pictures | Yes | No |
| Helm's state, the model's pose | Every 0.5 s, 0.1 s | Every 2 s, 0.5 s |

Zero and start messages stay on the socket. A pose reaches `helm.xr.Headset.receive` by either path, with the same checks:
its sequence drops anything older than the last one. The channel steers only for the login whose socket is connected, and
offers need a login, not control, as the socket does.

## The parts

- `helm/webrtc_worker.py`: its own process, on the background CPUs and nicer than Helm, because libx264 and aiortc's Python
  must not share Helm's interpreter: a head thread holding the arms' process's GIL once stretched their cycles to 40 ms.
- `helm/webrtc_link.py`: Helm's side, on its loop. Sends only the head camera's newest frame, and only while a peer lives.
- `helm/webrtc_video.py`: libx264 at the rate control's target. No STUN or TURN: Tailscale or the LAN reaches Minerva.
- `helm/webrtc_rate.py`: Google Congestion Control's loss rule, with slow growth near the last rate that lost too much.
- `static/xr_remote.js`: the page's side, with the same face as `xr_camera.HeadCamera`.
- `requirements/helm-webrtc.lock`: installed by `ops.helm_robot deps` (and a deploy) into its own folder, on the worker's path only,
  so its cryptography never reaches Helm's paramiko. If it is missing, Helm runs and refuses `?remote` with a message.

The Orin Nano has no hardware encoder, and no GPU encoder exists for it, so this is CPU work. The worker's load on the
Jetson has not been measured.

## Limits on x264

All measured on 640x480 at 10 frames a second. Without VBV the first picture of a busy scene was 148 kB, over 2 s at
600 kbit/s, so `bufsize` limits a picture to 20 kB. x264 follows a rate changed mid-stream only below the rate it was opened
at: opened at 300 kbit/s it stays near 290 whether asked for 150 or 1000, while opened at 1.5 Mbit/s it gave 152 and 706
with no new whole picture. So it opens at the top and is steered down.

aiortc answers every resend request. At a 300 kbit/s cap they reached 600 packets a second, so nothing new got through, and
a budget of 3 a second showed nothing at 30 % random loss, where resends are what make a picture at all. Hence at least 15
a second, or half the rate.

## Finding what a link carries

`scripts/webrtc_link_probe.py` measures from the headset's own browser and needs nothing of Helm's. It sends fixed rates of
real H.264 padded with filler the decoder skips, so its CPU use is negligible, and compares poses over the socket and the
channel. Results go to a JSON file. Running it from `ltr-portal` covers all of the path but the last 0.3 ms to the Jetson.

First result, 2026-11-04, a Quest Pro on a phone hotspot over Tailscale: the link carried 0.25 Mbit/s whole at 10 frames a
second, lost frames at 0.5, and showed none from 1 Mbit/s up, with what got through falling as more was sent. So a cap of
about 0.3-0.4 Mbit/s, where one JPEG view needs 4. Poses over the socket on the idle link had gaps up to 3.1 s, with 429 of
938 in time. Once settled, on the channel with video saturating the link, their worst gap was about 200 ms.

## Rehearsal, and what it does not show

All offline, on one PC: a real Helm, a fake camera relay making 68 kB JPEGs, a headless browser, and a UDP relay that drops
what exceeds a token bucket, 45 s each. The rehearsal scripts are not in the repository.

| Link | Frames a second | Video Mbit/s |
|---|---|---|
| Clean | 9-9.5 | climbs to 0.8 |
| 600 kbit/s cap | 5-9.5 | 0.5-0.6 |
| 400 kbit/s cap | 9 | 0.25-0.4 |
| 300 kbit/s cap | 7-9.5 | 0.22-0.27 |
| 400 kbit/s cap and 10 % loss | 7.5-9 after 15 s | 0.12-0.18 |
| 30 % loss | 0.5-5.5, gaps to about 2 s | 0.08-0.14 |

Decoding and encoding took 15 % of one Raspberry Pi 4 core at 10 frames a second, on those JPEGs. aiortc's own Python comes
on top and is not in that figure.

This has not run on a Quest, on the Jetson, on a real carrier, or through Tailscale. `xr_remote.js` reads frames with `createImageBitmap` on a timer because a video element's frame callback may pause in
an immersive session, which also has not been tried on a Quest.
