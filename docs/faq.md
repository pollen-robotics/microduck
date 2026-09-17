# FAQ

Questions that come up when you want a duck to do something it does not do out of the box. The
design docs say how the machinery works; this says which piece to reach for.

## I want to run a model that is too heavy for the board

Run it somewhere else and send it the camera. The board is a Radxa Zero 3 — the NPU takes a small
detector and little more — so anything larger belongs off the robot, and a Hugging Face Space on
paid hardware is the path with the least to build: it already has a GPU option, an account system
the robot shares, and a URL.

**Use WebRTC.** The robot publishes H.264 over a WebRTC session, your Space consumes it, and the
rendezvous introduces the two so neither needs to know the other's address. `spaces/vision-demo/`
is the worked example.

```
your Space ──sign in, list robots──►  rendezvous  ◄──registers──  the duck
your Space ◄═══════════ H.264 over WebRTC, media and control ════════════►  the duck
```

Three things you get for free by staying on WebRTC, and they are the reason it is the default:
the media is encrypted end to end by DTLS-SRTP, the control channel rides the same session so you
can *drive* as well as watch, and there is a return path — audio, or a second video track — the
day you want one.

## Does it work from a data centre? My Space is behind a NAT I do not control

Yes. A robot behind a home router and a container behind a data centre's NAT usually cannot
hole-punch to each other, so the robot offers a **relay candidate** — short-lived Cloudflare
credentials it mints with its own account token — and your consumer uses it without holding any
credentials of its own. `remote-access-design.md` §6 is the mechanism.

Two caveats worth knowing before you are surprised by them:

- **A relay costs the robot owner's bandwidth**, metered per Hugging Face account at 10 GB a
  month on the free tier. A continuous 720p stream is roughly a gigabyte an hour, so a Space that
  watches all day will spend it. ICE only relays when it must — a direct pair is used whenever
  one can be found — but "must" is common between a home and a data centre.
- **A browser consumer needs relay credentials of its own** when it has no IPv4 of its own; a
  Python one does not. §6 has the case.

## My consumer is a program and the stream runs all day. Is there something cheaper?

There is a fallback, and it is a fallback rather than a second design: **`media.stream`**, where
the robot dials *your* WebSocket and pushes frames outbound.

```
your Space ──media.stream {url: "wss://…/frames"}──►  rendezvous  ──►  the duck
the duck   ══════════ frames, outbound wss, direct ══════════════►  your Space
```

It costs nobody's relay, because an outbound connection from the robot needs no hole punched. Use
it when all three are true: the consumer is a **program** and not a person, the stream is
**long-running** enough for relay metering to matter, and you need **frames only**.

What you give up by leaving WebRTC, which is why it is not the default:

- **No return path at all.** Nothing reaches the robot on this transport — no audio, no second
  track, no teleop loop.
- **No control channel.** Driving the robot means a JSON-RPC call over the rendezvous
  (`spaces/shared/wire.py`), separately.
- **Encryption is your TLS, not DTLS-SRTP**, and it terminates at your server rather than at the
  peer.

The robot sends one text **hello** describing what is coming — `frames.encoding` is `h264` or
`jpeg` — then one binary message per frame. Branch on the hello rather than sniffing the bytes;
`spaces/vision-demo/receiver.py` is the reference receiver and `decoder_for` is the branch.

## How does my Space find the robot, and what stops somebody else's reaching it?

The account. The robot registers with the rendezvous holding its own Hugging Face token, your
Space signs the visitor in with `hf_oauth`, and the service only ever shows an account the robots
it owns. Nothing is configured on the robot and no robot-side gate is involved —
`remote-access-design.md` §7 is the argument for why that is enough.

`spaces/shared/rendezvous.py` is the listing call: `ducks(token)` gives you what that account can
reach.

## Can I test without a robot?

`scripts/duck-sim` runs the real daemons against a MuJoCo body
([`robot/simulation.md`](robot/simulation.md)). It registers with the rendezvous the way a board
does, flagged simulated, so a Space reaches it exactly as it reaches a robot.
