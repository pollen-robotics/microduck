# When `apply` correctly does nothing and that is the wrong answer

Status: **decision wanted, nothing implemented** · Date: 2026-08-07 · Owner: pierre

Two open items lifted out of `install-path-gap.md` so they can be decided on their own. Both were
proposed there while diagnosing an afternoon of `configd` updates that reported success and changed
nothing, and both are about the same moment: the board is in a state you need to get out of, and the
command you would reach for is a no-op.

Neither is urgent any more, and that is deliberately the first thing said here. The restart set is
now derived from the release (`engine.rs::units_to_restart`) and `updaterd` and `btd` restart
themselves a few seconds after the update replies (`RESTART_AFTER_REPLYING`), so the situation these
dig you out of is much harder to reach than it was when they were written. They are recorded because
they are still *right*, not because anything is on fire.

## 1. `already_current` compares the installed release, never the running one

`Engine::apply` returns `ApplyResult::AlreadyCurrent` when the manifest's version equals the
installed version (`engine.rs:366`, and again at `:833` for `select`). That is the correct answer to
the question it asks. It is the wrong question in exactly one case: the release is installed and the
process serving is from a different one.

That case cost two hours. The update reported success, the daemon kept answering on old code, and
`robotctl update apply` then said `already current` and did nothing — so the obvious recovery command
confirmed there was nothing to recover. Four correct `wifi` fixes were diagnosed as broken against
binaries that were never running.

`robotctl version` *does* diagnose it, in as many words: "configd is running X but the installed
daemon release is Y … either the restart did not happen, or it failed". Nobody ran it. The argument
this rests on is that a diagnostic nobody reaches for is worth about as much as one that does not
exist, so the *update* should notice rather than a separate command.

**The proposal.** Compare running revisions, not only the installed version, and stop returning
`AlreadyCurrent` when they disagree.

**What needs deciding.** Not whether to compare — that part is clearly right — but what to do on a
mismatch, and there are two defensible answers:

- **Report it and refuse.** `AlreadyCurrent` grows a variant, or a new `ApplyResult::Skewed` names
  both revisions and exits non-zero. Honest, cheap, and leaves the operator holding the problem.
- **Fix it.** Treat a running/installed mismatch as work to do: re-run the post-install hooks and
  the restart, without re-downloading or re-swapping. Strictly more useful, and it makes `apply`
  idempotent-with-repair rather than idempotent-and-inert.

The second is the same behaviour item 2 below proposes to put behind a flag, which is the real
question: **should repairing skew need a flag at all?** If `apply` repairs skew by default, item 2
mostly evaporates.

Also unsettled, and cheap to get wrong: *which* revision to compare. `robotctl version` asks each
daemon over its socket, which is the honest source — but `btd` serves no socket, so its running
revision cannot be read at all. Any design here has a `btd`-shaped hole in it and should say so
rather than quietly skip it.

## 2. `apply` has no `--force`

`install --force` exists and is the precedent (`updater/src/main.rs:145`, gated on `robotd` being
silent — see the `refusing --force` branch). It exists for a chicken-and-egg: a board cannot install
the release that fixes the gate rejecting it. `apply` has the same shape of problem and no such
escape.

**The proposal.** `apply --force` re-runs the hooks and the restart on an already-current release,
without re-downloading or re-swapping.

**What needs deciding.**

- **Does it subsume item 1, or depend on it?** A `--force` that always re-runs is simple and blunt.
  A `--force` that is only needed when item 1 chooses "report and refuse" is a smaller feature.
  Decide item 1 first.
- **What guard does it need?** `install --force` refuses while `robotd` answers, because it disables
  the health gate. `apply --force` would *keep* the gate — it is going through the daemon — so the
  same guard is probably wrong here, and copying it by symmetry would make the flag useless
  precisely when a robot is up and skewed. This needs its own reasoning, not `install`'s.
- **Does it re-run the hooks, or only the restart?** `hooks/postinstall` installs the release's
  units, and re-running it is idempotent by design. The restart is the part that matters. Doing both
  is probably right and should be a stated choice, because a hook that is *not* idempotent would
  make this a footgun.

## Recommendation

Decide item 1 first, and lean toward **repair rather than refuse** — the failure this comes from was
expensive because the recovery path was inert, and "report and refuse" leaves it inert with better
wording. If `apply` repairs skew by default, `--force` shrinks to a much smaller thing or is not
needed.

## Not implemented here

This document is a decision, not a design. No code accompanies it, on purpose: both items change
what `apply` does on a board, and the previous round of "correct and too narrow" fixes in this area
is what `install-path-gap.md` was written about.
