# Virtual Duck

A Microduck that lives on an Android phone's screen, on top of whatever app is open. It walks along
the bottom edge, looks at you, asks for food, and plays. You can tap it, stroke its head, pick it up
and throw it. It also has a home screen with its needs, its name, its voice and its size.

Everything it does is the real duck's. The **body** is the MJCF and meshes from
[microduck_rl](https://github.com/pollen-robotics/microduck_rl), and the **motions** are recorded from
the official policy set that `Cargo.toml` pins. The **voice** is made for a phone's speaker, by the
synths in `tools/voices`, and the song it sings is the robots' own chorale from [`sounds`](../sounds).

Android only: iOS does not let an app draw over other apps.

```
microduck_rl (MJCF, meshes, infer_policy.py) ─┐
official policies (HF Hub, Cargo.toml pin) ───┼─ tools/build_assets.sh ─► android/app/src/main/assets/duck
tools/voices + sounds/scores/wistful ─────────┘                             duck.glb · duck_rig.json
                                                                            motions.bin · motions.json
                                                                            voices/ · voices.json
```

## Why the motions are recorded, not simulated

The policies were trained against MuJoCo and the BAM model of the XL330. Under any other physics
engine they would not hold the duck up, and a phone running MuJoCo plus ONNX Runtime all day over
other apps would be a battery problem. So `tools/record_motions.py` runs them here, under exactly
that physics, using microduck_rl's own `scripts/infer_policy.py` harness (imported, not copied). It
records what they did at 50 Hz. The app replays the frames: every step, sit, peck, kick, fall and
get-up on screen is a copy of what the real policies drove the simulated body to do.

Each take is cut into clips, all stored relative to where the duck stood when the clip began:

| clips | from |
|---|---|
| `stand`, `walk_start` · `walk` · `walk_stop` | `velstand`, at 0.4 m/s — a full stick, robotd's `pad_drive` default; it stands still below ~0.25 |
| `turn_{left,right}_start` · loop · `_stop` | `velstand` at ±1.0 rad/s |
| `sit_down` · `sit_idle` · `sit_up` | `alpha_sitstand` |
| `peck` | `alpha_ground_pick` — with the bite time and beak reach measured on the jaw mesh |
| `kick_left`, `kick_right` | the kick policies, with the ball's track |
| `roulade` | the forward roll |
| `fall_{forward,backward,left,right}` | `velstand` pushed over at 1.2 m/s, getting itself up |
| `dance` | `velstand` standing, with its head and body-pose commands swung to a beat |

Locomotion is cut into a start, a seamless loop and a stop. The stop comes from a deterministic
re-run that stops on the loop's own first pose, so the app can leave the loop on any wrap.

## What the app adds

Only what no policy commanded:

- the head turning toward a point, using signs measured in MuJoCo;
- the beak opening in time with the voice (its hinge is worked out from the jaw's geometry, since
  the MJCF has no mouth joint);
- steering of a straight walk, a few centimetres of slide when lining up a peck or a kick, and the
  swing of a carried or thrown duck.

The brain (`pet/Brain.kt`) chooses among the recorded motions from needs (food, energy, fun, love),
mood, and what is put in front of it. Anything the owner gives it — a ball, seeds, a dance — it
does whatever its mood, waiting for the end of a stride, standing up or waking first. Seeds and
a ball come before everything else it might be minding: listening, a dance, a nap on the charger.

What it reacts to:

- **Being talked to.** With the microphone switched on, the duck stops, faces you and tilts its
  head. When you finish, it answers in duck: as many sounds as you said syllables, in your rhythm.
  It works from loudness alone (`pet/Conversation.kt`), so nothing is recorded, nothing leaves the
  phone, and it never answers its own voice. It does not listen while the phone plays anything
  else, such as a video in the app underneath. A long, steady sound like a television becomes the
  room's background, and a sound that turns out not to be speech is let go.
- **Touch.** Stroking it makes hearts float up, and a few strokes in a row make it wiggle with
  pleasure. Tapping it gets a hello. Shaking the phone makes it tumble and get back up.
- **The phone.** Unlocking gets a hello. On the charger, a tired duck naps.
- **Singing.** From friendship level 3 it sings the robots' chorale in its own voice, and dances to it.
- **Habit.** Every kindness adds friendship XP, which unlocks the song. Every day it is looked
  after extends a streak. A gentle reminder, at most twice a day and only for a reason, comes while
  it is put away.

How it's handled on the screen:

- **Hold it** for its menu, with a ✕ underneath, as with Messenger's chat heads. Drag it onto the ✕
  to put it away.
- **📍, then tap anywhere:** it walks there, or jumps there and lives at that height. Its floor is
  the bottom of the screen by default, or wherever its owner chooses.

## Its voice

Three voices, each low, middle or high, chosen on the home screen:

- **Babble** (the default): duck-speak. Quick vowel syllables on a pentatonic scale, in the manner
  of a game's animalese, with a nasal beak sound under every vowel.
- **Duckling**: a duckling's soft peeps.
- **Squeaky**: a bath-toy squeak, with the shrill part taken out.

Each is a numpy synth in `tools/voices/` that renders every call the duck makes:
- a hello that ends falling;
- a question that ends rising;
- a happy chirp and a content coo;
- a nibble;
- a startled eek, which jumps up and tumbles;
- a whee.

Each call comes in several variants, so the duck never sounds like a recording. Talk-back strings
them together, so they are pitched in one key and levelled by how loud they sound through a phone
speaker, not by their peak. Nobody can listen from a script, so `tools/voices/check.py` measures each
voice instead (`comfort.py` says what each measure stands for): shrillness, buzz, roughness, clicks
and attack, and loudness against the other calls. All three pass every target. The three come out
within about two decibels of each other, so changing voice hardly changes volume.
`render_voices.py` levels each song to sit just under the talking.

Buttons make no sound of their own.

## Installing on a phone

Install the APK (`android/app/build/outputs/apk/debug/app-debug.apk`), open it, and tap
**Allow the duck over other apps**.

On Android 15 and later, an app installed from a file rather than a store cannot be given "Display
over other apps" straight away: the switch answers *App was denied access*. This is Android's
[restricted settings](https://androidauthority.com/android-15-restricted-settings-sideloading-3481098),
and the app walks you through it when it happens. The steps:

1. Open the app's info page.
2. Tap ⋮ → **Allow restricted settings**.
3. Come back and turn the switch on.

Installing over `adb install` usually avoids the lock.

## Building

```sh
virtual-duck/tools/build_assets.sh        # needs ../microduck_rl (or DUCK_SIM_RL), uv, ffmpeg
cd virtual-duck/android && ./gradlew assembleDebug
```

The assets take about half a minute on one core. The app needs the Android SDK (compileSdk 36) and
JDK 17+. Gradle is configured small (1.5 GB heap, two workers) on purpose.

Tests: `./gradlew testDebugUnitTest`. They cover the geometry, the clip sampler and the animator
with synthetic clips, and the voice-activity detector with synthetic sound. They also drive the
brain through whole behaviours with the recorded motions: eating a seed, being thrown, chasing the
ball, sleeping, and every way it has been found to ignore its owner. They skip those if the assets
have not been built.

End to end, on a device or an emulator, `tools/e2e.sh [app-debug.apk]` drives the real app. A
debuggable build carries a debug bridge (`DebugBridge.kt`) that takes commands over adb (`feed`,
`ball`, `listen`, `voice:squeaky` and so on) and writes the duck's state where `run-as` can read it.
The script lets the duck out over another app and checks that it eats and plays in every situation
it used to fail in, then cycles through every voice. `SPEED=8` runs the duck's logic eight times
faster than it is drawn, for an emulator that draws a frame or two a second.

## Performance

It runs on top of other apps all day, so:

- **Rendering:** 53k triangles, decimated from the CAD's 740k with meshoptimizer. Rendered by Filament
  into a window little bigger than the duck, with MSAA and no shadow maps.
- **Frame rate:** about 30 fps moving, 20 idle, 6 asleep, and none with the screen off or the app open.
  Between frames the loop is not woken at all.
- **Per-frame cost:** no allocation in the frame path. One transform transaction per frame. Motions
  are one flat float array read in a single copy.
- **Package size:** Ogg voices at 24 kHz (2.6 MB for nine voices). No AndroidX, AppCompat or Compose.
- **Listening:** loudness only, on a low-priority thread, and only while the duck is visible and
  talking is switched on. The microphone stops with the screen, including after the system restarts
  the service in the dark.

## Licences

The app's code and the tools are Apache-2.0, like this repository.

The duck's **meshes** are CC BY-NC-SA 4.0 (microduck_rl's `LICENSE-MODELS.md`), and so is the
decimated `duck.glb` made from them. That is why the generated assets are rebuilt from pinned
sources and never committed, and why this app must not be sold. `build_assets.sh` writes the
attribution beside them.

## Status

Built, and its 73 unit tests pass. Among them:
- a ten-minute random soak that ends with the owner's seeds still being eaten;
- a regression test for every way the duck was found ignoring its owner, each confirmed to fail
  without its fix. That includes a sound that never became speech, seeds or a ball next to a wall,
  seeds left beyond a floor that shrank, a charger nap and a song.

On an Android 15 emulator, `tools/e2e.sh` lets the duck out over another app (Settings) and passes
every check. Seeds are eaten and the ball is kicked in each of these situations:
- at home, and out over the other app;
- after a noise that never became speech, and after being talked to;
- asleep, mid-dance, and after a tumble;
- after the screen was off, and after home was opened and closed;
- after a burst of forty random commands.

The test also confirms that every voice loads in every pitch, and that nothing crashed. Earlier
manual runs covered the home screen, walking over the launcher and other apps, the long-press menu
with the ✕, 📍 then a tap, and throwing.

It also runs on a real phone. A screen recording on an Android phone shows:
- the home screen, with the duck rolling and dancing, and the voice and pitch settings;
- the duck walking out over the app switcher and a browser;
- the long-press menu with the ✕, and 📍 then a tap;
- the duck chasing and kicking its ball.

Not yet checked on a phone:
- talking;
- dropping the duck on the ✕;
- the voices through a phone speaker, since the recording has no sound. They are designed and
  checked by measurement, and need a listen.

Frame rate and battery are unmeasured. The emulator renders in software at about 2 fps, and the
phone recording shows the duck running but measures neither.

To test on an emulator, start it with `-gpu swangle_indirect`. With `swiftshader_indirect` and
`host` the emulator process itself crashed once the duck was drawing. To get a screenshot of what
Filament drew, from a debuggable build:

```sh
adb shell am broadcast -a io.github.samimreza.virtualduck.CAPTURE -p io.github.samimreza.virtualduck
adb shell run-as io.github.samimreza.virtualduck cat files/captures/overlay.png > overlay.png
```

The package name, `io.github.samimreza.virtualduck`, is a placeholder for whoever adopts the app.
