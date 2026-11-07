# The headset on a slow or lossy link

A Quest Pro on a phone hotspot, over Tailscale, steered the head well but showed about 2 video frames in 2 minutes. On the
Jetson `ss -ti` showed 15-35 % of bytes retransmitted and 25-330 kbit/s delivered. One head view as JPEGs at 10 frames a second
needs about 4 Mbit/s, over TCP, which resends what is already out of date. `/xr?remote` sends the head camera as rate-controlled
H.264 over WebRTC instead.

**Status.** Built and measured in pieces: the worker and a real peer on the LAN, offline link rehearsals, and a probe from the
headset's own browser. **WebRTC with real video on a Quest has never been seen working**: both Quest tests on a hotspot ran on
the ordinary link, not the remote one. Head steering from another network (over Tailscale, on the ordinary link) worked.

## Using it

Open the headset's page as `/xr?remote`. Everything else about the headset is unchanged, and the ordinary `/xr` still uses its
WebSockets. The page's Video row names its link and a button leads to the other. Inside a session they look alike, except that the
ordinary link has wrist pictures and the remote one has a `Remote link` line in the ring. **A button reading "Use the remote
link" means the ordinary link is open.**

| | `/xr` | `/xr?remote` |
|---|---|---|
| Head camera | JPEG over WebSocket | H.264 over WebRTC, rate-controlled |
| Poses | WebSocket | Unordered WebRTC channel, no resends; the socket until it opens |
| Status replies | WebSocket | The channel, while poses use it |
| 3D (depth) | Yes | No: the picture stays flat, since video carries no per-frame depth or head pose |
| Wrist pictures | Yes | No |
| Helm's state, the model's pose | Every 0.5 s, 0.1 s | Every 2 s, 0.5 s |

Zero and start messages stay on the socket. A pose reaches `helm.xr.Headset.receive` by either path with the same checks (its
sequence drops anything older than the last one). The channel steers only for the login whose socket is connected, and offers
need a login, not control, as the socket does. A Helm restart ends logins; a page left open then goes to `/login?next=/xr`.

## The parts

- `helm/webrtc_worker.py`: its own process, on the background CPUs and nicer than Helm, because libx264 and aiortc's Python must
  not share Helm's interpreter (a head thread holding the arms' process's GIL once stretched their cycles to 40 ms).
- `helm/webrtc_link.py`: Helm's side. Sends only the head camera's newest frame, and only while a peer lives.
- `helm/webrtc_video.py`: libx264 at the rate control's target. No STUN or TURN: Tailscale or the LAN reaches Minerva.
- `helm/webrtc_rate.py`: Google Congestion Control's loss rule, with slow growth near the last rate that lost too much.
- `static/xr_remote.js`: the page's side, with the same face as `xr_camera.HeadCamera`.
- `requirements/helm-webrtc.lock`: installed by `ops.helm_robot deps` (and a deploy) into its own folder, on the worker's path only,
  so its cryptography never reaches Helm's paramiko. If it is missing, Helm runs and refuses `?remote` with a message.

The Orin Nano has no hardware encoder, so this is CPU work: 17 % of one core measured on the Jetson, below.

## Limits on x264

All measured on 640 x 480 at 10 frames a second. Without VBV the first picture of a busy scene was 148 kB, over 2 s at 600 kbit/s,
so `bufsize` limits a picture to 20 kB. x264 follows a rate changed mid-stream only below the rate it was opened at, so it opens
at the top (1.5 Mbit/s) and is steered down. aiortc answers every resend request; at a 300 kbit/s cap they reached 600 packets a
second and nothing new got through, hence at least 15 resends a second, or half the rate.

## Finding what a link carries

`scripts/webrtc_link_probe.py` measures from the headset's own browser and needs nothing of Helm's. It sends fixed rates of real
H.264 padded with filler the decoder skips, compares poses over the socket and the channel, and writes a JSON file. On a Quest
Pro on a phone hotspot over Tailscale the link carried 0.25 Mbit/s whole at 10 frames a second, lost frames at 0.5 and showed
none from 1 Mbit/s up, so a cap of about 0.3-0.4 Mbit/s against the 4 Mbit/s one JPEG view needs. Poses over the socket on the
idle link had gaps up to 3.1 s; once settled on the channel with video saturating the link, their worst gap was about 200 ms.

## Offline rehearsal and the Jetson

A real Helm, a fake camera relay making 68 kB JPEGs, a headless browser and a UDP relay that drops what exceeds a token bucket,
45 s each (not a Quest, the Jetson, a real carrier or Tailscale):

| Link | Frames a second | Video Mbit/s |
|---|---|---|
| Clean | 9-9.5 | climbs to 0.8 |
| 600 kbit/s cap | 5-9.5 | 0.5-0.6 |
| 400 kbit/s cap | 9 | 0.25-0.4 |
| 300 kbit/s cap | 7-9.5 | 0.22-0.27 |
| 400 kbit/s cap and 10 % loss | 7.5-9 after 15 s | 0.12-0.18 |
| 30 % loss | 0.5-5.5, gaps to about 2 s | 0.08-0.14 |

On the Jetson, with nothing running on the arms, the real worker and a real peer on a PC on the LAN (without Helm between them),
fed the same JPEGs at 10 frames a second: 599 of 600 frames arrived at 640 x 480, the first after 0.3 s, the worst gap 0.19 s, no
resends. The worker took about 17 % of one core and 71-77 MB at nice 5 on the background CPUs, which were 43-50 % busy against
35-39 % without it (most of the baseline is the wrist cameras). Nothing was measured at a hotspot's 0.3 Mbit/s or beside a
teleop session.

## Reading a test afterwards

Each run of the worker writes `~/minerva/evidence/remote-link-<UTC>.jsonl`: `offer` and `answer` with their candidates and each
connection `state`; `stats` each second (rate and target, loss, round trip, resends, JPEGs in from Helm, poses in); and `page`
every 2 s (the headset's own bytes and frames received, decoded and handed to the screen, the video element's state, visibility,
and the path used). `received` rising with `shown` flat means the picture is lost inside the headset; `frames_in` flat means the
camera stopped, not the link. No file and no worker means the page was on the ordinary link.

## Open

- Real camera video in a Quest's immersive session. `xr_remote.js` reads frames with `createImageBitmap` on a timer because a
  video element's frame callback may pause there.
- Helm's own relay feeding the worker on Minerva, which needs a login.
- A path through a subnet router to Minerva, where the Quest's address is not reachable from Minerva and the connection
  depends on the Quest's checks arriving first.
- The worker beside the arms, and arm control over the channel.
- **The head faulted with `command_expired` in both hotspot sessions** (3 to 4 minutes into following) while the arms kept every
  cycle on time. A command's lifetime counts from the reply before it, so the head's 100 ms `command_ms` spans two 40 ms gaps and
  leaves about 20 ms; the receiver checks expiry at each 10 ms tick, so the next command must be read within one tick of its due
  time. Nothing records where the milliseconds went, and 29 minutes on the LAN on the same CPUs had no fault. Feedback older than
  60 ms or more than 80 ms between steps also fault the head. The margin is thin in any session.
