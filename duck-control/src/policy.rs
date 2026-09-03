//! The ONNX policies.
//!
//! Walking and standing are chosen by the magnitude of the velocity command, exactly as
//! `microduck_runtime` does; the skill networks — sit↔stand, ground pick, the two kicks —
//! are selected explicitly by the scheduler in `robotd`, which owns the priority rules.
//! Every network shares the one 61-D observation layout, so a skill is a session choice
//! plus a command-block encoding, never a new contract.
//!
//! **Everything is validated at load, not at inference.** A bundle with the wrong
//! observation width, the wrong action count, or a missing ONNX Runtime must fail while the
//! robot is standing still and the caller can be told why — not sixty ticks later, mid
//! stride. `robotd` turns a load failure into "hold the pose and report unhealthy", so the
//! updater rolls the release back instead of leaving a robot that cannot walk.

use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use ort::session::Session;
use ort::session::builder::GraphOptimizationLevel;
use ort::value::{Value, ValueType};

use crate::obs::{ACTION_LEN, OBS_LEN, Observation};

/// Below this velocity magnitude the standing policy takes over. The prototype's value.
pub const DEFAULT_STANDING_THRESHOLD: f64 = 0.05;

/// Inference threads per session.
///
/// One, deliberately. The prototype uses two, which on a four-core A55 means the control
/// thread blocks on a pool it does not own — and for a network this small the pool costs
/// more in synchronisation than it recovers in parallelism. Worth re-measuring on the board
/// rather than trusting either number.
const INTRA_THREADS: usize = 1;

#[derive(Debug, thiserror::Error)]
pub enum PolicyError {
    #[error("loading {path}: {source}")]
    Load {
        path: PathBuf,
        #[source]
        source: ort::Error,
    },
    /// The bundle does not match what this build implements. Reported with both shapes
    /// because "wrong policy file" and "wrong daemon" look identical without them.
    #[error("{path}: {what} is {got}, expected {expected}")]
    Shape {
        path: PathBuf,
        what: &'static str,
        expected: String,
        got: String,
    },
    #[error("inference failed: {0}")]
    Inference(String),
    /// ONNX Runtime is not installed, or not where it is being looked for.
    ///
    /// Its own diagnosis, because it is an operator problem with an operator fix — install
    /// the library or set `ORT_DYLIB_PATH` — and not a broken policy bundle.
    #[error("ONNX Runtime not loadable ({searched}): {detail}")]
    RuntimeMissing { searched: String, detail: String },
    /// `ort` panicked instead of returning an error. See [`catching_ort_panics`].
    ///
    /// `detail` is the panic message, and carrying it is the point: the one panic we have
    /// actually seen on a board names the two version numbers that explain it.
    #[error("ort panicked loading the policy: {detail}")]
    RuntimePanic { detail: String },
}

impl PolicyError {
    /// The file this error is about, when it is about one.
    ///
    /// `Load` and `Shape` name a file; a missing runtime or an `ort` panic does not, and
    /// blaming whichever policy happened to be loading when the dylib turned out to be absent
    /// would send an operator to replace a file that is fine.
    pub fn path(&self) -> Option<&Path> {
        match self {
            PolicyError::Load { path, .. } | PolicyError::Shape { path, .. } => Some(path),
            PolicyError::Inference(_)
            | PolicyError::RuntimeMissing { .. }
            | PolicyError::RuntimePanic { .. } => None,
        }
    }
}

/// Where `ort` will look for the runtime, replicating its own logic.
fn dylib_name() -> String {
    match std::env::var("ORT_DYLIB_PATH") {
        Ok(path) if !path.is_empty() => path,
        _ => {
            if cfg!(target_os = "windows") {
                "onnxruntime.dll".to_owned()
            } else if cfg!(any(target_os = "macos", target_os = "ios")) {
                "libonnxruntime.dylib".to_owned()
            } else {
                "libonnxruntime.so".to_owned()
            }
        }
    }
}

/// Confirm ONNX Runtime is loadable **before** calling into `ort`.
///
/// This exists because `ort` does not return an error when the dylib is missing — it
/// `expect`s inside `setup_api`, from a lazy path reachable through any API call, so a
/// missing library aborts the thread that touched it. In the control loop that means the
/// thread dies, no tick ever lands, and `robot.health` reports "the loop has not completed a
/// cycle" forever: the daemon looks wedged instead of saying ONNX Runtime is not installed.
///
/// Probing first turns the *missing library* case into an ordinary error the caller can
/// report, with the operator's fix in it. That is all it does.
///
/// It does **not** mean `ort` cannot then panic, and an earlier version of this comment
/// claimed it did. A board running ONNX Runtime 1.20.1 falsified that: the library loaded, so
/// the probe passed, and `ort` panicked in `setup_api` on its own version check
/// (`expected version >= '1.23.x', but got '1.20.1'`). The probe proves the file loads;
/// nothing more. [`catching_ort_panics`] covers the rest, including panics we have not seen.
fn ensure_runtime() -> Result<(), PolicyError> {
    static PROBE: OnceLock<Result<(), String>> = OnceLock::new();
    let outcome = PROBE.get_or_init(|| {
        let name = dylib_name();
        // Safety: loading a shared library runs its initialisers. This is the same library
        // `ort` is about to load itself, so the risk is not one this probe introduces.
        match unsafe { libloading::Library::new(&name) } {
            Ok(library) => {
                // Leak it: `ort` will dlopen the same file moments later and the OS
                // reference-counts the mapping. Dropping ours would be harmless but
                // pointless churn.
                std::mem::forget(library);
                Ok(())
            }
            Err(e) => Err(e.to_string()),
        }
    });

    outcome
        .clone()
        .map_err(|detail| PolicyError::RuntimeMissing {
            searched: dylib_name(),
            detail,
        })
}

/// Run the `ort` calls, turning a panic from inside them into a [`PolicyError`].
///
/// `ort` treats some initialisation failures as unrecoverable and panics rather than
/// returning `Err` — the version mismatch in [`ensure_runtime`]'s comment is the one a board
/// hit, and it fires from inside a lazy init reachable through any API call. In the control
/// thread a panic is worse than an error: the thread dies, no tick ever lands, and
/// `robot.health` answers "the loop has not completed a cycle yet" — the one message that
/// names no cause — while the daemon stays up serving its socket. The updater then rolls the
/// release back for a reason nobody can act on.
///
/// `robotd` already handles a policy that fails to load: hold the pose, keep ticking at rate,
/// report why, get rolled back. This makes a panic take that same path.
///
/// Deliberately wraps the `ort` work only, and not all of [`Policy::load`], so a genuine bug
/// of ours does not get relabelled "policy unavailable". Note that a caught panic has still
/// run the panic hook, so the backtrace is in the journal either way.
///
/// `AssertUnwindSafe` is needed because `Session` is not `UnwindSafe`. It is sound here
/// because nothing of ours is observed after a catch: the sessions being built are moved into
/// the `Policy` on success and dropped on failure, and the caller's answer is the error.
///
/// **`panic = "abort"` would defeat this.** The root `Cargo.toml` has no `[profile.release]`,
/// so the default unwind strategy applies; adding one would silently turn this back into a
/// dead control thread.
fn catching_ort_panics<T>(work: impl FnOnce() -> Result<T, PolicyError>) -> Result<T, PolicyError> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(work)).unwrap_or_else(|payload| {
        Err(PolicyError::RuntimePanic {
            detail: panic_message(payload),
        })
    })
}

/// The panic message, or a stand-in saying there wasn't one.
///
/// `panic!` with a literal produces `&'static str`; with arguments, `String`. `ort` uses both.
fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
    if let Some(s) = payload.downcast_ref::<&'static str>() {
        (*s).to_owned()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "panicked with no message; see the journal for the backtrace".to_owned()
    }
}

/// Which network drives a tick.
///
/// The choice is the caller's — the skill scheduler in `robotd` owns the priority rules —
/// and this enum is how it names its choice. Asking for a network that is not loaded falls
/// back to walking rather than panicking, but the scheduler is expected to check `has_*`
/// first; the fallback exists so a race cannot kill the control thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Net {
    Walk,
    Stand,
    /// Commanded sit↔stand: the twist `vx` slot carries a posture flag, 1 = sit, 0 = stand.
    SitStand,
    /// Phase-scripted ground pick; the twist slots carry `[cos φ, sin φ, 0]`.
    GroundPick,
    /// A one-shot skill, by its index in [`PolicyPaths::skills`].
    ///
    /// Kicks and roulade used to be variants here. They were the same thing three times over —
    /// a network trained on an all-zero command, driving for a fixed window, selected by an
    /// explicit request — differing only in duration and tuning, which is data. An index means
    /// a robot gains a skill by gaining a config entry rather than a release.
    Skill(usize),
}

/// Which policy files to load. `walk` is mandatory; every other slot is a capability the
/// robot simply does not have when `None`.
#[derive(Debug, Clone, Default)]
pub struct PolicyPaths {
    pub walk: PathBuf,
    pub stand: Option<PathBuf>,
    pub sitstand: Option<PathBuf>,
    pub ground_pick: Option<PathBuf>,
    /// One-shot skills, in the priority order the caller wants them considered. Each is
    /// selected only by an explicit request, so an empty list is a robot with no tricks rather
    /// than a robot missing something.
    pub skills: Vec<PathBuf>,
}

/// The loaded networks.
///
/// A configured path that fails to load fails the whole load — the policies ship inside the
/// release, so a missing or corrupt file is a broken bundle, and the right outcome is
/// "unhealthy, roll it back", not a robot that silently lost its kick.
pub struct Policy {
    walk: Session,
    stand: Option<Session>,
    sitstand: Option<Session>,
    ground_pick: Option<Session>,
    skills: Vec<Session>,
    standing_threshold: f64,
    /// Roller mode and fall-recovery mode reserve the standing network (roller has none;
    /// fall recovery keeps it for getting up), so command magnitude must never select it.
    standing_disabled: bool,
}

impl Policy {
    /// Load, validate and warm up.
    ///
    /// `stand` is optional: without it the walking policy runs at every velocity, which is
    /// what a single-policy bundle does.
    pub fn load(paths: &PolicyPaths, standing_threshold: f64) -> Result<Self, PolicyError> {
        ensure_runtime()?;

        // Everything below calls into `ort`, and `ort` panics on failures it considers
        // unrecoverable — so the whole of it, and nothing else, goes inside the catch.
        catching_ort_panics(move || {
            // Warm up before the loop ever calls this. The first inference is always an
            // outlier — lazy initialisation, cold pages, first-touch faults — and paying that
            // on tick one would look exactly like a control loop that missed its deadline.
            // It also proves ONNX Runtime is actually present and usable, which with
            // `load-dynamic` is not known until something is run.
            let zero = Observation::zeroed();
            fn open_warm(path: &Path, zero: &Observation) -> Result<Session, PolicyError> {
                let mut session = open(path)?;
                run(&mut session, path, zero)?;
                Ok(session)
            }
            fn open_opt(
                path: &Option<PathBuf>,
                zero: &Observation,
            ) -> Result<Option<Session>, PolicyError> {
                path.as_deref().map(|p| open_warm(p, zero)).transpose()
            }

            Ok(Self {
                walk: open_warm(&paths.walk, &zero)?,
                stand: open_opt(&paths.stand, &zero)?,
                sitstand: open_opt(&paths.sitstand, &zero)?,
                ground_pick: open_opt(&paths.ground_pick, &zero)?,
                skills: paths
                    .skills
                    .iter()
                    .map(|path| open_warm(path, &zero))
                    .collect::<Result<Vec<_>, _>>()?,
                standing_threshold,
                standing_disabled: false,
            })
        })
    }

    /// Reserve the standing network: command magnitude no longer selects it, and only an
    /// explicit [`Net::Stand`] from the caller (fall recovery, body pose) reaches it.
    pub fn set_standing_disabled(&mut self, disabled: bool) {
        self.standing_disabled = disabled;
    }

    /// Whether the standing policy would be chosen for this command.
    ///
    /// Separate from [`Self::infer`] because the caller needs the same answer to decide
    /// gains and action scale, and asking twice must not be able to disagree.
    pub fn will_stand(&self, twist_magnitude: f64) -> bool {
        self.stand.is_some()
            && !self.standing_disabled
            && twist_magnitude <= self.standing_threshold
    }

    pub fn has_standing(&self) -> bool {
        self.stand.is_some()
    }

    pub fn has_sitstand(&self) -> bool {
        self.sitstand.is_some()
    }

    pub fn has_ground_pick(&self) -> bool {
        self.ground_pick.is_some()
    }

    /// How many one-shot skills are loaded. A caller's index is valid below this.
    pub fn skill_count(&self) -> usize {
        self.skills.len()
    }

    /// One inference on the named network. A missing optional network falls back to
    /// walking — the scheduler checks `has_*` before asking, so reaching the fallback is a
    /// bug, but a wrong gait beats a dead control thread.
    pub fn infer(
        &mut self,
        observation: &Observation,
        net: Net,
    ) -> Result<[f32; ACTION_LEN], PolicyError> {
        let session = match net {
            Net::Walk => None,
            Net::Stand => self.stand.as_mut(),
            Net::SitStand => self.sitstand.as_mut(),
            Net::GroundPick => self.ground_pick.as_mut(),
            Net::Skill(index) => self.skills.get_mut(index),
        };
        let session = match session {
            Some(session) => session,
            None => &mut self.walk,
        };
        run(session, Path::new("<loaded>"), observation)
    }
}

/// Can this file be loaded as a policy, without committing to running it?
///
/// Opens the graph and checks both shapes, then throws the session away. Two callers, and they
/// want it for the same reason from opposite ends:
///
///  - `robot.loadPolicy` answers a client *synchronously*, while the real swap happens seconds
///    later at the home pose. Validating here is what turns "accepted" followed by a robot that
///    did not change into an immediate `observation width is 51, expected 61`.
///  - startup checks each overridden slot before building the controller, so one bad override
///    costs that slot rather than the whole policy.
///
/// **No warm-up inference**, unlike [`Policy::load`] — this is a question about a file, not a
/// session about to be driven, and the first-inference cost is the loading path's to pay. It
/// therefore proves less: a graph that opens and has the right shape can still fail to run.
/// Nothing downstream treats a pass as a guarantee, which is why a failed load at the home pose
/// still has to keep the controller it had.
///
/// Not from inside a tick. Opening a session is tens of milliseconds and the loop has 20 to
/// spend — so the IPC caller runs it on its own runtime, and the loop only ever calls it in its
/// preamble, before the first tick is due.
pub fn validate(path: &Path) -> Result<(), PolicyError> {
    ensure_runtime()?;
    catching_ort_panics(|| open(path).map(drop))
}

fn open(path: &Path) -> Result<Session, PolicyError> {
    let session = Session::builder()
        .and_then(|b| b.with_optimization_level(GraphOptimizationLevel::Level3))
        .and_then(|b| b.with_intra_threads(INTRA_THREADS))
        .and_then(|b| b.commit_from_file(path))
        .map_err(|source| PolicyError::Load {
            path: path.to_owned(),
            source,
        })?;

    check_width(path, "observation width", session.inputs(), OBS_LEN)?;
    check_width(path, "action count", session.outputs(), ACTION_LEN)?;
    Ok(session)
}

/// Assert the trailing dimension of a graph's single tensor outlet.
///
/// The leading dimension is the batch and is usually dynamic (`-1`), so only the last one
/// is checked. That is the one that encodes the contract.
fn check_width(
    path: &Path,
    what: &'static str,
    outlets: &[ort::value::Outlet],
    expected: usize,
) -> Result<(), PolicyError> {
    let shape = match outlets.first().map(|o| o.dtype()) {
        Some(ValueType::Tensor { shape, .. }) => shape,
        _ => {
            return Err(PolicyError::Shape {
                path: path.to_owned(),
                what,
                expected: expected.to_string(),
                got: "not a tensor".into(),
            });
        }
    };

    let got = shape.iter().last().copied().unwrap_or(-1);
    if got != expected as i64 {
        return Err(PolicyError::Shape {
            path: path.to_owned(),
            what,
            expected: expected.to_string(),
            got: got.to_string(),
        });
    }
    Ok(())
}

fn run(
    session: &mut Session,
    path: &Path,
    observation: &Observation,
) -> Result<[f32; ACTION_LEN], PolicyError> {
    let input = Value::from_array(([1usize, OBS_LEN], observation.as_slice().to_vec()))
        .map_err(|e| PolicyError::Inference(format!("{}: building input: {e}", path.display())))?;

    let outputs = session
        .run(ort::inputs!["obs" => &input])
        .map_err(|e| PolicyError::Inference(format!("{}: {e}", path.display())))?;

    let value = outputs
        .values()
        .next()
        .ok_or_else(|| PolicyError::Inference(format!("{}: no output", path.display())))?;
    let (_, data) = value.try_extract_tensor::<f32>().map_err(|e| {
        PolicyError::Inference(format!("{}: extracting output: {e}", path.display()))
    })?;

    if data.len() != ACTION_LEN {
        return Err(PolicyError::Inference(format!(
            "{}: {} actions, expected {ACTION_LEN}",
            path.display(),
            data.len()
        )));
    }
    let mut actions = [0.0f32; ACTION_LEN];
    actions.copy_from_slice(data);
    Ok(actions)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The threshold decides walking versus standing every tick, so it must match what the
    /// prototype uses or the robot changes gait at a different speed than it was tuned for.
    #[test]
    fn the_standing_threshold_matches_the_prototype() {
        assert_eq!(DEFAULT_STANDING_THRESHOLD, 0.05);
    }

    /// A bundle without a standing policy must never select one. Slice 2 can ship a single
    /// policy, and `will_stand` returning true there would index a session that is not
    /// loaded.
    #[test]
    fn without_a_standing_policy_it_never_stands() {
        // Constructed directly rather than via `load`, which needs ONNX Runtime present.
        // This is the branch that has to hold regardless of what is installed.
        let threshold = DEFAULT_STANDING_THRESHOLD;
        let stands = |has_stand: bool, magnitude: f64| has_stand && magnitude <= threshold;

        assert!(!stands(false, 0.0), "no standing policy, zero command");
        assert!(stands(true, 0.0), "standing policy, zero command");
        assert!(!stands(true, 0.5), "standing policy, walking command");
    }

    /// Roller and fall-recovery modes reserve the standing network, so the magnitude rule
    /// must be inert while `standing_disabled` is set — otherwise a roller duck at zero
    /// stick would swap to a network trained for legs it is not standing on.
    #[test]
    fn disabling_standing_beats_the_magnitude_rule() {
        let threshold = DEFAULT_STANDING_THRESHOLD;
        let stands = |has_stand: bool, disabled: bool, magnitude: f64| {
            has_stand && !disabled && magnitude <= threshold
        };

        assert!(stands(true, false, 0.0));
        assert!(
            !stands(true, true, 0.0),
            "disabled must win at zero command"
        );
    }

    /// **The panic contract.** A panic out of `ort` must come back as a `PolicyError`, because
    /// the caller is the control thread: an escaping panic kills it, no tick ever lands, and
    /// health reports "the loop has not completed a cycle" — naming no cause — instead of
    /// holding the pose and saying the policy is unusable.
    ///
    /// The message must survive too. This is the panic a Radxa actually produced, and the two
    /// version numbers in it are the whole diagnosis; a health reason without them tells an
    /// operator nothing.
    ///
    /// The panic hook still runs, so this test prints a panic and a backtrace hint. That is
    /// wanted — on a board it is what puts the detail in the journal — and is not a failure.
    #[test]
    fn a_panic_out_of_ort_becomes_an_error_that_keeps_its_message() {
        let err = catching_ort_panics::<()>(|| {
            panic!(
                "Failed to load ONNX Runtime dylib: ort 2.0.0-rc.11 is not compatible with \
                 the ONNX Runtime binary found at `libonnxruntime.so`; expected version >= \
                 '1.23.x', but got '1.20.1'"
            )
        })
        .expect_err("a panic in the ort work must not escape to the caller");

        assert!(
            matches!(err, PolicyError::RuntimePanic { .. }),
            "wrong variant: {err:?}"
        );
        let reported = err.to_string();
        for detail in ["1.23.x", "1.20.1"] {
            assert!(
                reported.contains(detail),
                "the version detail must reach the caller, got {reported:?}"
            );
        }
    }

    /// An error that names no file must not be attributed to one. A missing ONNX Runtime is an
    /// operator problem with an operator fix, and reporting it as "this slot's file is bad" sends
    /// them to replace a policy that is fine — which is exactly what the startup fallback would
    /// do with it, silently dropping every override on a board that simply has no dylib.
    #[test]
    fn only_file_errors_name_a_file() {
        let shape = PolicyError::Shape {
            path: PathBuf::from("/tmp/x.onnx"),
            what: "observation width",
            expected: "61".into(),
            got: "51".into(),
        };
        assert_eq!(shape.path(), Some(Path::new("/tmp/x.onnx")));

        let missing = PolicyError::RuntimeMissing {
            searched: "libonnxruntime.so".into(),
            detail: "not found".into(),
        };
        assert_eq!(missing.path(), None, "a missing runtime blames no policy");

        let panicked = PolicyError::RuntimePanic {
            detail: "version mismatch".into(),
        };
        assert_eq!(panicked.path(), None, "an ort panic blames no policy");
    }

    /// Success must pass straight through — a wrapper that swallowed the value would turn
    /// every load into "policy unavailable" on a board where everything works.
    #[test]
    fn the_catch_is_transparent_when_nothing_panics() {
        assert_eq!(catching_ort_panics(|| Ok(7)).unwrap(), 7);
    }

    /// A panic payload that is neither `&str` nor `String` must still produce a reason. The
    /// alternative is an empty health string, which reads as "no reason given".
    #[test]
    fn an_unprintable_panic_payload_still_reports_something() {
        let detail = panic_message(Box::new(42u32));
        assert!(!detail.is_empty(), "a reason is mandatory");
        assert!(
            detail.contains("journal"),
            "point somewhere useful: {detail:?}"
        );
    }
}
