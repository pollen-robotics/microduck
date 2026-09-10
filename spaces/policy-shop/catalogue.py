"""What the Hub offers a duck, read the way the robot itself reads it.

`updater/src/policy.rs` searches `https://huggingface.co/api/models?search=microduck&limit=25`
and reads `manifest.json` out of each hit — no tag filter, because `microduck` in the name is
what the published policies have in common and a tag is something to add once there is something
to tag. This file makes the same two requests, so the gallery and `policy.search` cannot disagree
about what exists.

**It reads more than the robot's search answers.** `PolicySearchHit` carries an id, an origin and
two counters; a page that asks somebody to run a policy on their robot should show what the
policy claims to be first — `docs/policy-manifest.md` is the vocabulary, and every field in it is
the publisher's own. Untrusted, which here means "displayed, and never the thing acted on": the
skill that gets written comes from the robot's own reading of the manifest it downloaded, which
is what will actually run.

**Two shapes, one vocabulary.** A single-policy repo carries one `policy.onnx` with the fields at
the top level; the official set carries nine files and the same fields once per entry under
`policies`. `policy.fetch` takes a `file`, so an entry out of the set is one click like any other.

Runnable on its own — `uv run catalogue.py` prints what a duck would be offered — which is how
this is checked without a robot, a token or a Space.
"""

from __future__ import annotations

import json
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Any

import requests

HUB_API = "https://huggingface.co/api/models"
# What `origin_of_repo` calls official. Anything else is community, and the word is the only
# difference the robot draws.
OFFICIAL_ORG = "pollen-robotics"
# The nine-policy set, listed entry by entry rather than as one repo: fetching it whole is
# `policy.install`, which replaces what the robot walks with, and that is not a one-click.
SET_REPO = "pollen-robotics/microduck-policies"
SEARCH = "microduck"
TIMEOUT = 15

# What an entry in a set inherits from the manifest's top level: the claims that are about the
# robot and the convention, and nothing that is about a policy. Inheriting `description` gave
# every one of the official nine the same line — a sentence describing the set, printed nine
# times as though it described each file.
INHERITED = ("schema_version", "model_api", "obs_len", "action_len", "robot")


@dataclass
class Policy:
    """One thing a person could put on their duck. Every field below the repo is a claim."""

    repo: str
    # Which file in the repo, for an entry out of the set. `None` means "the only `.onnx` in it",
    # which is what `policy.fetch` defaults to and what every single-policy repo has.
    file: str | None = None
    name: str = ""
    description: str | None = None
    kind: str | None = None
    encoding: str | None = None
    duration_s: float | None = None
    unwind_s: float | None = None
    entry_pose: str | None = None
    slot: str | None = None
    chain: bool = False
    # `command.idle` — the twist that means "stop doing the thing". Its presence is what tells a
    # held perpetual policy apart from a gait; see `caution`.
    idle: list[float] | None = None
    obs_len: int | None = None
    action_len: int | None = None
    model_api: int | None = None
    schema_version: int | None = None
    likes: int | None = None
    downloads: int | None = None
    training: dict[str, Any] = field(default_factory=dict)

    @property
    def origin(self) -> str:
        return "official" if self.repo.split("/")[0] == OFFICIAL_ORG else "community"

    @property
    def key(self) -> str:
        return f"{self.repo}#{self.file}" if self.file else self.repo

    def headline(self) -> str:
        """The one line a row shows: what it is, and how long it takes."""
        bits = [self.kind or "kind not declared"]
        if self.duration_s:
            bits.append(f"{self.duration_s:g}s")
        elif (self.kind or "").lower() == "perpetual":
            bits.append("holds until told otherwise")
        if self.encoding and self.encoding != "constant":
            bits.append(f"command: {self.encoding}")
        return " · ".join(bits)


def refusal(policy: Policy) -> str | None:
    """Why this cannot be a one-shot skill, or `None`.

    **The daemon does not make this check and `robot.setSkill` would accept the entry.** A policy
    whose command the daemon has to generate — a phase for a ground pick, a flag for a sit↔stand —
    fed a constant instead is a robot moving plausibly and wrongly, which is worse than a
    refusal. `robotctl`'s `skill_encoding_refusal` is where this rule lives on the robot's side,
    and it is repeated here rather than relied on: `robotctl` is not in the path of a click.

    The shape claims — `obs_len`, `action_len`, `model_api`, `robot.model` — are deliberately not
    repeated. `policy.fetch` refuses on those itself, before the download, and a robot refusing
    is a better answer than a Space guessing what a robot is.
    """
    encoding = (policy.encoding or "constant").lower()
    if encoding == "phase":
        return (
            f"{policy.name} is driven by a phase the daemon generates, so it cannot be a "
            "one-shot — it is a ground pick, and it belongs in that slot "
            "(`robotctl policy load ground_pick …`)."
        )
    if encoding == "posture_flag":
        return (
            f"{policy.name} is driven by a posture flag the daemon flips, so it cannot be a "
            "one-shot — it is a sit↔stand, and it belongs in that slot "
            "(`robotctl policy load sitstand …`)."
        )
    if encoding != "constant":
        return (
            f"{policy.name}'s manifest says its command encoding is {policy.encoding!r}, which "
            "no daemon knows how to drive — a skill needs a constant command."
        )
    return None


def needs_a_length(policy: Policy) -> bool:
    """Whether somebody has to say how long to hold this one.

    A `perpetual` policy has no length of its own — that is what perpetual means — so a one-shot
    made out of one is a hold and an unwind, and `robotctl policy add` refuses without `--hold`
    rather than picking a number. The page asks instead.
    """
    return not policy.duration_s


def caution(policy: Policy) -> str | None:
    """What is worth saying before this runs, when it is not a refusal.

    **A perpetual policy that declares no way to stop is a gait, not a one-shot.** The manifest
    convention is explicit about the difference: `RemiFabre/microduck-flamingo-cycle` is a
    published perpetual policy meant to be held, and what makes that work is `command.idle` plus
    `unwind_s` — the twist that means "stop" and how long it needs to get there. A perpetual
    policy carrying neither is `alpha_walking`: a thing to load into the `walk` slot, which is
    `policy load` and not a click here. Held anyway it runs for the seconds asked and hands
    straight back to the gait, so this is a caution and not a refusal — the daemon guards the
    encoding, and this is a judgement about intent rather than about safety.
    """
    if (policy.kind or "").lower() != "perpetual":
        return None
    if policy.idle or policy.unwind_s:
        return None
    return (
        "declares no way to stop — no `command.idle`, no `unwind_s`. That reads as a gait for a "
        "slot rather than a one-shot; held for the seconds below it hands straight back to "
        "whatever the robot was walking with."
    )


def _policy_from(fields: dict[str, Any], repo: str, file: str | None) -> Policy:
    command = fields.get("command") or {}
    robot = fields.get("robot") or {}
    name = fields.get("name") or _stem(file) or _stem(repo.split("/")[-1])
    policy = Policy(
        repo=repo,
        file=file,
        name=name,
        description=fields.get("description"),
        kind=fields.get("kind"),
        encoding=command.get("encoding"),
        duration_s=_number(fields.get("duration_s")),
        unwind_s=_number(fields.get("unwind_s")),
        entry_pose=fields.get("entry_pose"),
        slot=fields.get("slot"),
        chain=bool(fields.get("chain")),
        idle=command.get("idle") if isinstance(command.get("idle"), list) else None,
        obs_len=_integer(fields.get("obs_len")),
        action_len=_integer(fields.get("action_len")),
        model_api=_integer(fields.get("model_api")),
        schema_version=_integer(fields.get("schema_version")),
        training=fields.get("training") or {},
    )
    # A manifest that names another robot is kept and labelled rather than dropped: `policy.fetch`
    # refuses on `robot.model` itself, and a row that says why is a better answer than a policy
    # that silently is not in the list somebody was told to look in.
    model = robot.get("model")
    if model and model != "microduck":
        policy.description = f"published for a {model}, not for a duck"
    return policy


def _merge(manifest: dict[str, Any], entry: dict[str, Any]) -> dict[str, Any]:
    """One entry of a set, with the set's shape claims and none of its prose."""
    return {**{k: v for k, v in manifest.items() if k in INHERITED}, **entry}


def _stem(text: str | None) -> str:
    if not text:
        return ""
    stem = text.rsplit(".", 1)[0] if text.endswith((".onnx", ".json")) else text
    return stem.removeprefix("microduck-")


def _number(value: Any) -> float | None:
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def _integer(value: Any) -> int | None:
    return int(value) if isinstance(value, int) and not isinstance(value, bool) else None


def _manifest(repo: str) -> dict[str, Any] | None:
    """A repo's `manifest.json`, or `None` if it has none.

    No token: these are public repos, and a Space that reads them signed in as whoever is looking
    would show a different catalogue to each visitor for no reason.
    """
    url = f"https://huggingface.co/{repo}/resolve/main/manifest.json"
    try:
        answer = requests.get(url, timeout=TIMEOUT)
    except requests.RequestException:
        return None
    if answer.status_code != 200:
        return None
    try:
        manifest = answer.json()
    except ValueError:
        return None
    return manifest if isinstance(manifest, dict) else None


def _hits() -> list[dict[str, Any]]:
    try:
        answer = requests.get(
            HUB_API, params={"search": SEARCH, "limit": 50}, timeout=TIMEOUT
        )
        answer.raise_for_status()
        hits = answer.json()
    except (requests.RequestException, ValueError):
        return []
    return hits if isinstance(hits, list) else []


def read_hub() -> tuple[list[Policy], str | None]:
    """Everything the Hub offers, and why the list is short if it is.

    An unreachable Hub is a fact to report rather than an error to raise — the same call
    `PolicyCheckResult.unreachable` makes — because a page that shows nothing and says nothing is
    indistinguishable from a Hub with nothing in it.
    """
    hits = _hits()
    if not hits:
        return [], (
            "the Hub answered nothing. Either it is unreachable from this Space, or no model "
            f"matches {SEARCH!r} — `huggingface.co/api/models?search={SEARCH}` is the request."
        )

    counters = {
        hit.get("modelId") or hit.get("id"): (hit.get("likes"), hit.get("downloads"))
        for hit in hits
        if isinstance(hit, dict)
    }
    repos = [repo for repo in counters if repo and repo != SET_REPO]

    with ThreadPoolExecutor(max_workers=8) as pool:
        manifests = dict(zip(repos, pool.map(_manifest, repos)))
    manifests[SET_REPO] = _manifest(SET_REPO)

    policies: list[Policy] = []

    # The set first: its entries are the ones a stock duck already has, so they are the row a
    # person recognises, and they are what says the transport works before anything new is tried.
    official = manifests.get(SET_REPO) or {}
    for entry in official.get("policies") or []:
        if not isinstance(entry, dict):
            continue
        file = entry.get("file")
        # A `file` with a path in it is skipped by the seeder and by `policy update` both
        # (`docs/policy-manifest.md`), so it is skipped here for the same reason.
        if not isinstance(file, str) or "/" in file or file.startswith("."):
            continue
        policies.append(_policy_from(_merge(official, entry), SET_REPO, file))

    for repo in repos:
        manifest = manifests.get(repo)
        if not manifest:
            continue
        # A set published by somebody else is listed entry by entry, exactly like ours.
        entries = manifest.get("policies")
        if isinstance(entries, list) and entries:
            for entry in entries:
                if isinstance(entry, dict) and isinstance(entry.get("file"), str):
                    policies.append(_policy_from(_merge(manifest, entry), repo, entry["file"]))
            continue
        policy = _policy_from(manifest, repo, None)
        policy.likes, policy.downloads = counters.get(repo, (None, None))
        policies.append(policy)

    # Official first, then by likes: a page whose first row is a stranger's untested policy is
    # asking for the wrong thing to be clicked first.
    policies.sort(key=lambda p: (p.origin != "official", -(p.likes or 0), p.name))
    return policies, None


def skill_for(fetched: dict[str, Any], hold: float | None) -> dict[str, Any]:
    """`robot.setSkill` parameters, from what the robot itself read off the manifest.

    Mirrors `robotctl policy add` field for field, and takes its values from `policy.fetch`'s
    answer rather than from this Space's own reading of the Hub: the robot downloaded the file
    and parsed the manifest beside it, so its answer is about the bytes that are going to run.

    `hold` is only consulted when the policy declares no length. A flag beating a manifest is
    `robotctl`'s rule too, and for the same reason — a person watching the robot is a better
    judge than a number measured in simulation — but a page cannot watch, so here the manifest
    wins wherever it has an opinion.
    """
    duration = fetched.get("duration_s") or hold
    params: dict[str, Any] = {
        "name": fetched.get("name") or _stem(fetched.get("file")) or "policy",
        "path": fetched["path"],
        "duration": float(duration),
        # Whether a held button chains another run is the policy's to say — the roulade does, a
        # kick does not — and the manifest is where it says it.
        "chain": bool(fetched.get("chain")),
    }
    # How to stop is the manifest's too: `command.idle` is the twist that means "stop doing the
    # thing", and `unwind_s` is how long the policy needs to get there.
    if fetched.get("idle"):
        params["unwind"] = fetched["idle"]
    if fetched.get("unwind_s"):
        params["unwind_s"] = float(fetched["unwind_s"])
    if fetched.get("action_scale"):
        params["action_scale"] = float(fetched["action_scale"])
    return params


if __name__ == "__main__":
    found, why = read_hub()
    if why:
        print(why)
    for one in found:
        mark = refusal(one)
        print(f"{one.origin:9} {one.key}")
        print(f"          {one.name} — {one.headline()}")
        if one.description:
            print(f"          {one.description}")
        if mark:
            print(f"          REFUSED: {mark}")
        else:
            if needs_a_length(one):
                print("          needs a hold: its manifest declares no length")
            if caution(one):
                print(f"          caution: {caution(one)}")
    print(f"\n{len(found)} policies")
