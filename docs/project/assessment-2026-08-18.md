# Where the project stands

Status: snapshot · Date: 2026-08-18 · Owner: pierre

A read of the tree at `59cb88b`, against [`roadmap.md`](roadmap.md). The roadmap owns what each
milestone is and what "done" means for it; this is a dated judgement about what to do next, what is
missing, and what has accumulated. It goes stale on purpose.

## The tree is healthy

`cargo test --workspace` is green — 636 tests, one ignored, on a cold clone with no hardware, no
network and no Docker. CI runs fmt, clippy over all targets, the suite, `shellcheck` on all eleven
shipped scripts, an `xtask package` smoke test, the 13-check board suite under QEMU against Debian
13, and a coverage ratchet at 72% against ~77% actual. There is not one `TODO`, `FIXME` or `todo!()`
in the workspace.

That is the state to keep in mind while reading the rest: nothing below is a fire. The problems are
of the "shipping is blocked on a decision nobody has made" and "effort is going somewhere other than
the risk" kind.

## Three things block a robot leaving the lab

Each is known and written down. None is being worked on.

**1. A private repo cannot serve the fleet** ([`updater-design.md`](../design/updater-design.md)
§6.1). A field robot has no GitHub token and should not have one, and a private repo's asset URLs
404 without one. So every board that updates itself today does so because a developer's token is in
its environment. The conventional fix — a second, public repo holding only signed artifacts — costs
one repository and two lines of config, changes no engine code, and closes M4's last hard blocker.
It has been open since 2026-08-05 and is the cheapest item on this page.

**2. BLE runs unencrypted, and the PIN crosses that link**
([`app-path-design.md`](../design/app-path-design.md) §5.5). `--require-pairing` defaults off
because requiring encryption on the version read hangs CoreBluetooth, which made a fresh install
unusable. The right call for a dev tool, and the cost is stated honestly in the doc: every robot
running this has wifi credentials and a pairing PIN readable by a bystander. What has not happened
is the next step §5.5 names — establishing whether a bond exists at all, which decides whether
moving the requirement to the write is a fix or a guess.

**3. The numbers M4 exists for are still unmeasured.** Thermals, eMMC write timing, battery under
load, whether the 30 s health-gate timeout has margin on a cold boot, and whether
`journalctl -u robotd -b -1` survives a power cut. These need a bench, a board and an afternoon —
not a design. They have been the open half of M4 since 2026-08-05, and two of them have downstream
consequences that are waiting on them:

- the safety layer has **no thermal gate**. `robotd` reads per-servo case temperature and the SoC
  zone, reports both in `health`, and acts on neither; `SafetyConfig` carries a deadman, a fall gate
  and a range clamp and nothing thermal. The roadmap says thermal "waits for a measured threshold
  rather than a guessed one", which is right — and it means the measurement is the only thing
  between here and a real protection.
- the 30 s gate is described in the roadmap as a guess. It is the number that decides whether a slow
  cold boot reverts a good release.

## Effort is going into a pad, not into the risk

Of the 50 commits since 2026-08-11, roughly 30 are gamepad radio and BLE pairing work, on top of a
hardware fault that is already diagnosed: half the Radxa units cannot keep an Xbox pad bonded, the
link never gets encrypted (`PIN or Key Missing`), and a *different* BLE pad bonds and re-encrypts
fine on the same board. That diagnosis is what the instrumentation was for, and it has arrived.

What has accumulated around it: two diagnostic scripts (`pad-link-test.sh`, `pad-stack-report.sh`),
a raw-input socket in `padd` with its own protocol method, a monitor view, a `pad fallback` command
writing kernel and BlueZ settings, a bond-hold watch with its own exit code, and three docs pages.
Most of it is genuinely good — `pad pair` no longer declares success on the pairing exchange alone,
which was a real lie — but the open PRs (#103, #104, #105) are now investment in a workaround for
one pad model on some units.

The decision that ends this is a pad choice, not more measurement. Either a pad model that bonds on
every unit becomes the supported one, or the Xbox interop is bisected against the old runtime's
three differences (`Privacy = device`, `disable_ertm=1`, the no-`Pair()` sequence) — #104 already
makes that a one-line comparison. Picking neither is what keeps generating this work.

`pad fallback` in particular needs an explicit exit condition written next to it: it is a crutch for
specific units, every board is meant to end up with neither setting, and a temporary mechanism with
no stated end becomes permanent.

## Missing features, in the order they will be wanted

| | |
|---|---|
| `mediad` | Nothing exists. Camera, encode, perception, WebRTC gateway, and the privacy work (per-session consent, streaming indicator) that M5 deliberately does not defer. The whole of M5. |
| SDK | The WebSocket + snapshot path that makes "a script drives the robot" a few dozen lines. Nothing exists. |
| Phone app | Nothing exists, and `configd` + `btd` already serve the API it will use — which was the right order. |
| Authority arbitration | [`architecture.md`](../design/architecture.md) §6 designs it; safety landed in M3, arbitration between clients did not. |
| Provisioning | Calibration, and a per-robot PIN that is generated, printed and recorded. Identity no longer waits on it. The factory default `000000` is public in this repository. |
| Bond revocation | Nothing un-pairs a phone; `bluetoothctl untrust` is the manual escape (§5.6). |
| PIN retry limiting across sessions | Three wrong guesses close a session; nothing counts across reconnects (§5.6). |
| Recovery mode | The boot recovery net exists and is good. What M6 still names is the last link: a board that needs recovering with no laptop. |
| Manifest staleness reporting | M6 cites `architecture.md` §8.4.2, which does not exist — see the drift note below. |
| Per-joint limits | `safety.rs` clamps to actuator travel (±π), not anatomy. Needs the alpha MJCF vendored. |
| Golden observation vectors | The 61-D encoding is tested for shape, not for agreement with `microduck_brain`. This is the one gap where a silent numerical divergence from the prototype would look like a policy that walks badly. |
| Six remaining skills, MuJoCo backend | Walk and stand are the two policies that exist. |

## Cleaning and simplification

Ordered by what gets more expensive if it waits.

**The shared serve loop, before `mediad` exists.** Four daemons carry their own accept-and-dispatch
loop (`updater/src/ipc.rs`, `configd/src/main.rs`, `robotd/src/main.rs`, plus `btd`'s session
routing), differing in four ways that are requirements rather than drift — peer policy, line cap,
push side, connection shape. PR #52's `monitor-design.md` (design only, unmerged)
makes the case and it holds: three implementations is the cheapest this
refactor ever gets, and `mediad` makes it four. It is also a prerequisite for any IPC observability,
because a tap each service must remember to call is the `_ => None` wildcard `route.rs` already
refuses on principle.

That PR also surfaced two things worth settling deliberately rather than by accident: `robotd`
passes no peer policy at all, and `Call::is_mutating()` does not cover `robot.enable` — the call
that starts a policy running on a walking robot is classified alongside `hello`.

**Files that have outgrown one file.** `robotctl/src/main.rs` holds every subcommand definition and
every renderer; `duck-ipc-proto/src/lib.rs` holds the whole wire contract; `updater/src/engine.rs`
holds the engine. All three are coherent and well commented, and all three are now the file where
two people editing different features collide. The proto crate is the easy one and the most useful:
splitting it along the namespaces it already has (`update.*`, `robot.*`, `net.*`, `pad.*`,
`system.*`) costs nothing and makes an API bump a diff in one module.

**Two CLIs over one API.** `robotctl` and `duck-btctl` speak the same JSON-RPC to the same services
over different transports, and the second one re-implements a subset of the first's surface with its
own rendering. `duck-btctl` is deliberately a test tool, so this is not urgent — but the moment it
grows a third command that `robotctl` already renders, the cheaper shape is one CLI with a transport
flag.

**7 400 lines of shell.** `install.sh`, `provision.sh`, `setup-board.sh`, `provision-board.sh` and
`migrate-network.sh` overlap in what they set up on a board, and the boundary between them is
learned rather than stated. They are linted and they work; what is missing is one page naming which
script owns which step, in the style the docs index already uses for design docs.

**Two D-Bus stacks in the shipped artifact.** `btd` links libdbus through `bluer`, `configd` uses
`zbus`. Already documented as accepted in `configd/Cargo.toml` with the condition that would revisit
it. Noted here only so it is not rediscovered as a surprise.

## Checks worth adding

**Supply-chain advisories.** 459 crates in the lock file, a signed-artifact update path, and no
`cargo audit`, no `cargo deny`, no dependabot. The concern is not the dependency count — it is that
an advisory in the tar, http or crypto tree that the engine depends on would reach a robot with
nothing in CI saying so. A `cargo deny check advisories` step is minutes of work and belongs in the
`check` job.

**`padd`'s intent mapping is untested.** All six tests in the crate are in `tap.rs`. `main.rs` — the
button semantics, body/head mode switching, the enable and stop path, and the deliberate decision
*not* to send a zero on disconnect — has none. That last one is a documented safety-relevant
behaviour with nothing pinning it.

**Nothing checks NetworkManager against the fake.** `app-path-design.md` §2.3 states it plainly:
every wifi bug found on a board was one where `FakeNet` had the correct behaviour and the NM path
had drifted. The fake is the written form of the contract and the check runs in the wrong direction.
Both implement `Net`, so the same assertions could run against both — it needs a board, which means
`board-test.sh` with a real NetworkManager rather than CI.

## Documentation drift

Small, and worth a single pass:

- `CONTRIBUTING.md` promises 508 tests, `roadmap.md` says 458, the suite runs 636. A number in prose
  that no test asserts will always drift; either assert it or describe it as "no hardware, no
  network, no Docker" and drop the count.
- `roadmap.md` M6 cites `architecture.md` §8.4.2 for manifest staleness. Section 8 stops at 8.4, and
  the word does not appear in the file. Either the section was removed or it was never written.
- `roadmap.md`'s "Where we are" table predates the pad work entirely, and M4 is still "in progress"
  with the same five open measurements it had on 2026-08-05.

## What I would do next

1. Create the public artifact repo and point `release.yml` and `updater.toml` at it. It is the
   smallest change on this page and it unblocks every robot that does not have a developer attached.
2. Spend one bench session on M4's five numbers, and land the thermal threshold that comes out of
   it as a real gate in `safety.rs`.
3. Decide the pad question — a supported pad model, or a bisect against the runtime's three
   differences — and give `pad fallback` a stated end.
4. Land the shared serve loop while there are three implementations rather than four.
5. Then start `mediad`, with the privacy surface in the first cut rather than after it.
