# Petting recordings

Recordings are deliberately not stored in Git. They contain room audio, can contain speech, and
grow much faster than source code. Keep them locally while collecting and reviewing them; publish a
versioned dataset only after checking consent, privacy, and the recording's license.

## Layout

Copy `manifest.example.jsonl` to `manifest.jsonl`. Each line describes one WAV relative to this
directory:

```json
{"path":"clips/duck-a-kitchen-01.wav","label":"normal","session":"duck-a-kitchen-01","robot":"duck-a","environment":"kitchen, motors idle","license":"CC-BY-4.0"}
```

`path`, `label`, and `session` are required. Labels are `normal` and `petting`. `robot`,
`environment`, and `license` are optional but should be present in a published dataset.
Record at 16 kHz, mono, signed 16-bit PCM when possible so the files match the onboard microphone;
the shared Rust feature extractor also accepts and resamples other WAV rates and channel layouts.

A session is one uninterrupted capture under one condition. Several files may share a session when
they were cut from the same capture; the trainer keeps the whole session on one side of the split.
Do not use a room name alone as the session — two independent captures in the same room should have
different IDs.

Without a manifest, the trainer still accepts the old `normal/*.wav` and `petting/*.wav` layout and
treats each WAV as an independent session. That mode cannot record provenance and should not be used
to publish a model.

## What to record

Use several robots if possible and many short, independent sessions rather than one long recording.
The validation set should contain conditions the training set cannot nearly duplicate.

Petting sessions should vary hand, direction, pressure, speed, and the point scratched. Normal
sessions should include the hard cases that can otherwise become false positives: walking and
standing motors, being picked up without scratching, taps on the shell, speech, music, claps,
speaker playback, charging noise, fans, dishes, silence, and ordinary room activity.

Before publishing, listen for speech or identifying sounds, remove unsafe captures, document
consent, and choose an explicit dataset license. Robot names in the manifest should be anonymous.
