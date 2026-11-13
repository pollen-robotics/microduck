/**
 * What the Hub offers a duck, read the way the robot reads it.
 *
 * A port of `spaces/policy-playground/catalogue.py`, which makes the same two requests
 * `updater/src/policy.rs` makes — `?search=microduck` and a `manifest.json` per hit — so the
 * gallery and `policy.search` cannot disagree about what exists.
 *
 * **In a browser, and that is a change worth stating.** The Python version ran in a data centre,
 * which is what got it rate-limited by an edge that reads `python-requests` as a bot. Here the
 * requests come from whoever is looking, with their own browser's signature and their own address.
 * The Hub is public and no token is sent: a catalogue that differed per visitor would be a
 * catalogue nobody could talk about.
 *
 * Every field below the repo is the publisher's claim, not a fact — displayed, never acted on.
 * What gets installed comes from the robot's own reading of the manifest it downloaded.
 */

const HUB_API = "https://huggingface.co/api/models";
const OFFICIAL_ORG = "pollen-robotics";
const SET_REPO = "pollen-robotics/microduck-policies";
const SEARCH = "microduck";

/** Fields an entry in a set inherits: the claims about the robot, never the prose about the set. */
const INHERITED = ["schema_version", "model_api", "obs_len", "action_len", "robot"] as const;

export interface Policy {
  repo: string;
  file: string | null;
  name: string;
  description: string | null;
  kind: string | null;
  encoding: string | null;
  durationS: number | null;
  unwindS: number | null;
  idle: number[] | null;
  official: boolean;
  /** The robot the manifest says it is for, when that is not a plain duck. */
  forRobot: string | null;
  /** A short clip of it being done, when the repo ships one. */
  video: string | null;
  likes: number;
  key: string;
}

/**
 * Why this cannot be a trick you ask for, or `null`.
 *
 * The daemon does not make this check and `robot.setSkill` would accept the entry: a policy whose
 * command the daemon generates — a phase for a ground pick, a flag for a sit — fed a constant
 * instead is a robot moving plausibly and wrongly, which is worse than a refusal. `robotctl` is
 * where the rule lives on the robot's side and `robotctl` is not in the path of a click, so it is
 * written here a second time.
 *
 * Said in words a child can act on: this is not a trick, it is part of how the duck moves.
 */
export function notATrick(policy: Policy): string | null {
  const encoding = (policy.encoding ?? "constant").toLowerCase();
  // Deliberately generic. The first version named a ground pick, because `phase` was the ground
  // pick's encoding — and then `roller_crouch` arrived, which is also `phase` and is not a ground
  // pick, and the page told somebody it was. The encoding says the duck drives this one itself;
  // it does not say what it is.
  if (encoding === "phase") return "Your duck drives this one itself. It is part of how it moves, not a trick.";
  if (encoding === "posture_flag") return "This is how your duck sits down and stands up, not a trick.";
  if (encoding !== "constant") return "Your duck does not know how to drive this one.";
  return null;
}

/**
 * Whether this is something to show off, or part of how the duck gets around.
 *
 * **A trick has an ending.** `roulade` takes a second and finishes; `alpha_walking` walks until
 * something tells it not to, and it declares no way to be told — no `command.idle`, no `unwind_s`.
 * Held for three seconds it runs and hands straight back to whatever the duck was walking with,
 * which is not wrong and is not a trick either. The Python version made this a caution beside a
 * button; on a page for a ten-year-old a caution nobody reads is a mislabel, so these go in their
 * own section instead — listed, explained, and not offered as something to press.
 */
export function isATrick(policy: Policy): boolean {
  if (notATrick(policy)) return false;
  return Boolean(policy.durationS || policy.idle || policy.unwindS);
}

/** Whether somebody has to say how long to hold it: perpetual policies have no length of their own. */
export function needsALength(policy: Policy): boolean {
  return !policy.durationS;
}

/** How long it takes, in words rather than a field name. */
export function howLong(policy: Policy): string {
  if (policy.durationS) {
    const seconds = policy.durationS;
    return seconds < 1.5 ? "about a second" : `about ${Math.round(seconds)} seconds`;
  }
  return "keeps going until you stop it";
}

function stem(text: string | null): string {
  if (!text) return "";
  const base = text.endsWith(".onnx") || text.endsWith(".json") ? text.replace(/\.[^.]+$/, "") : text;
  return base.replace(/^microduck-/, "").replaceAll("_", " ").replaceAll("-", " ");
}

function number(value: unknown): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function policyFrom(
  fields: Record<string, unknown>,
  repo: string,
  file: string | null,
  likes = 0,
  video: string | null = null,
): Policy {
  const command = (fields.command ?? {}) as Record<string, unknown>;
  const robot = (fields.robot ?? {}) as Record<string, unknown>;
  // `||` and not `??`, which cost a nameless card: `stem(null)` is `""`, and `""` is not nullish,
  // so a repo whose manifest has no `name` and whose entry has no `file` fell through every
  // fallback and rendered as an empty heading with a button under it.
  const name = String(fields.name || stem(file) || stem(repo.split("/")[1] ?? repo) || repo);
  let description = fields.description == null ? null : String(fields.description);

  // A manifest naming another robot is kept and labelled rather than dropped: `policy.fetch`
  // refuses on `robot.model` itself, and a row that says why beats a policy that is silently
  // missing from the list somebody was told to look in.
  // **Labelled, not hidden, and still pressable.** `policy.fetch` compares this against the robot's
  // own model and refuses — but the comparison is the *robot's* to make, and this page does not
  // know which kind of duck is on the other end until it asks. Excluding a `full_shell` policy
  // would be exactly wrong for somebody holding a full shell, so the card says what it is for and
  // lets the duck answer.
  const model = robot.model == null ? null : String(robot.model);
  const forRobot = model && model !== "microduck" ? model : null;

  return {
    repo,
    file,
    name: stem(name) || name || repo,
    description,
    kind: fields.kind == null ? null : String(fields.kind),
    encoding: command.encoding == null ? null : String(command.encoding),
    durationS: number(fields.duration_s),
    unwindS: number(fields.unwind_s),
    idle: Array.isArray(command.idle) ? (command.idle as number[]) : null,
    official: repo.split("/")[0] === OFFICIAL_ORG,
    forRobot,
    video,
    likes,
    key: file ? `${repo}#${file}` : repo,
  };
}

function merge(manifest: Record<string, unknown>, entry: Record<string, unknown>): Record<string, unknown> {
  const inherited: Record<string, unknown> = {};
  for (const field of INHERITED) if (field in manifest) inherited[field] = manifest[field];
  return { ...inherited, ...entry };
}

/**
 * A clip of the trick being done, out of whatever the repo happens to ship.
 *
 * **A convention, not a field.** Nobody specified this, and most publishers landed on the same
 * two paths anyway — `media/preview.mp4` and `preview.mp4` — so those are taken first and
 * everything else is ranked rather than guessed at. The ranking matters: one repo carries
 * `experiments/2026-09-13/examples/rejected_phrase_609_25_ducks_front_split_v2.mp4`, which is a
 * training artefact and a rejected one, and picking the first `.mp4` in the list would have put it
 * on the card as though it were the trick.
 *
 * A `.gif` is taken only when there is no video at all: it is somebody's fallback for a viewer
 * that cannot play one, and this page can.
 */
function previewIn(files: string[]): string | null {
  const clips = files.filter((f) => /\.(mp4|webm|mov)$/i.test(f));
  const pick =
    clips.find((f) => f === "media/preview.mp4") ??
    clips.find((f) => f === "preview.mp4") ??
    clips.find((f) => /(^|\/)preview[^/]*$/i.test(f)) ??
    // Shallowest, then shortest: a file at the top of a repo is the one somebody meant to be
    // found, and a deep path is nearly always a run, an experiment or an evaluation.
    clips.sort((a, b) => a.split("/").length - b.split("/").length || a.length - b.length)[0] ??
    files.find((f) => /\.gif$/i.test(f));
  return pick ?? null;
}

async function manifestOf(repo: string): Promise<Record<string, unknown> | null> {
  try {
    const answer = await fetch(`https://huggingface.co/${repo}/resolve/main/manifest.json`);
    if (!answer.ok) return null;
    const body: unknown = await answer.json();
    return body && typeof body === "object" ? (body as Record<string, unknown>) : null;
  } catch {
    return null;
  }
}

/**
 * Everything the Hub offers, and why the list is short if it is.
 *
 * An unreachable Hub is a fact to report rather than an error to throw: a page that shows nothing
 * and says nothing is indistinguishable from a Hub with nothing in it.
 */
export async function readHub(): Promise<{ policies: Policy[]; trouble: string | null }> {
  let hits: Array<Record<string, unknown>>;
  try {
    // `full=true` carries each repo's file list in the request we were making anyway, which is
    // what makes the preview videos free. Without it they would be one extra request per repo.
    const answer = await fetch(`${HUB_API}?search=${SEARCH}&limit=50&full=true`);
    if (!answer.ok) throw new Error(`HTTP ${answer.status}`);
    hits = (await answer.json()) as Array<Record<string, unknown>>;
  } catch (e) {
    return {
      policies: [],
      trouble: `Could not ask the Hub what tricks exist (${e}). Try again in a moment.`,
    };
  }

  const likes = new Map<string, number>();
  const previews = new Map<string, string | null>();
  for (const hit of hits) {
    const id = String(hit.modelId ?? hit.id ?? "");
    if (!id) continue;
    likes.set(id, typeof hit.likes === "number" ? hit.likes : 0);
    const files = Array.isArray(hit.siblings)
      ? (hit.siblings as Array<{ rfilename?: unknown }>)
          .map((s) => String(s.rfilename ?? ""))
          .filter(Boolean)
      : [];
    const clip = previewIn(files);
    previews.set(id, clip ? `https://huggingface.co/${id}/resolve/main/${clip.split("/").map(encodeURIComponent).join("/")}` : null);
  }
  const repos = [...likes.keys()].filter((repo) => repo !== SET_REPO);

  const manifests = new Map<string, Record<string, unknown> | null>();
  await Promise.all(
    [...repos, SET_REPO].map(async (repo) => manifests.set(repo, await manifestOf(repo))),
  );

  const policies: Policy[] = [];

  // The official set first, entry by entry: those are the ones a stock duck already has, so they
  // are the row somebody recognises and the one that proves the connection works.
  const official = manifests.get(SET_REPO) ?? {};
  const entries = Array.isArray(official.policies) ? (official.policies as Record<string, unknown>[]) : [];
  for (const entry of entries) {
    const file = entry.file;
    // A `file` with a path in it is skipped by the seeder and by `policy update` both, so it is
    // skipped here for the same reason.
    if (typeof file !== "string" || file.includes("/") || file.startsWith(".")) continue;
    policies.push(policyFrom(merge(official, entry), SET_REPO, file));
  }

  for (const repo of repos) {
    const manifest = manifests.get(repo);
    if (!manifest) continue;
    const own = Array.isArray(manifest.policies) ? (manifest.policies as Record<string, unknown>[]) : [];
    if (own.length) {
      // A set published by somebody else is listed entry by entry, exactly like ours.
      for (const entry of own) {
        if (typeof entry.file === "string") policies.push(policyFrom(merge(manifest, entry), repo, entry.file));
      }
      continue;
    }
    policies.push(policyFrom(manifest, repo, null, likes.get(repo) ?? 0, previews.get(repo) ?? null));
  }

  // Official first, then most-liked: a page whose first card is a stranger's untested policy is
  // asking for the wrong thing to be pressed first.
  policies.sort((a, b) => Number(b.official) - Number(a.official) || b.likes - a.likes || a.name.localeCompare(b.name));
  return { policies, trouble: null };
}

/**
 * `robot.setSkill` parameters, from what the robot itself read off the manifest.
 *
 * Mirrors `robotctl policy add` field for field and takes its values from `policy.fetch`'s answer
 * rather than from this page's reading of the Hub: the robot downloaded the file and parsed the
 * manifest beside it, so its answer is about the bytes that are going to run.
 */
export function skillFor(fetched: Record<string, unknown>, hold: number): Record<string, unknown> {
  const duration = number(fetched.duration_s) ?? hold;
  const params: Record<string, unknown> = {
    name: String(fetched.name ?? stem(String(fetched.file ?? "")) ?? "policy").replaceAll(" ", "-"),
    path: fetched.path,
    duration,
    // Whether holding the button chains another run is the policy's to say, and the manifest is
    // where it says it.
    chain: Boolean(fetched.chain),
  };
  if (fetched.idle) params.unwind = fetched.idle;
  if (number(fetched.unwind_s)) params.unwind_s = number(fetched.unwind_s);
  if (number(fetched.action_scale)) params.action_scale = number(fetched.action_scale);
  return params;
}
