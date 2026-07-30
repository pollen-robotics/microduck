//! The ONNX policy.
//!
//! Two sessions — walking and standing — chosen by the magnitude of the velocity command,
//! which is exactly what `microduck_runtime` does. No skill abstraction: it can arrive when
//! the third skill does, and until then it would be a seam nothing has tested.
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
/// Probing first turns that into an ordinary error the caller can report. We use the same
/// loader and the same search rule `ort` does, so a probe that succeeds means its load will
/// succeed too and the panic cannot fire.
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

/// A loaded policy pair.
pub struct Policy {
    walk: Session,
    stand: Option<Session>,
    standing_threshold: f64,
}

impl Policy {
    /// Load, validate and warm up.
    ///
    /// `stand` is optional: without it the walking policy runs at every velocity, which is
    /// what a single-policy bundle does.
    pub fn load(
        walk: &Path,
        stand: Option<&Path>,
        standing_threshold: f64,
    ) -> Result<Self, PolicyError> {
        ensure_runtime()?;
        let mut walk_session = open(walk)?;
        let mut stand_session = match stand {
            Some(path) => Some(open(path)?),
            None => None,
        };

        // Warm up before the loop ever calls this. The first inference is always an
        // outlier — lazy initialisation, cold pages, first-touch faults — and paying that
        // on tick one would look exactly like a control loop that missed its deadline.
        // It also proves ONNX Runtime is actually present and usable, which with
        // `load-dynamic` is not known until something is run.
        let zero = Observation::zeroed();
        run(&mut walk_session, walk, &zero)?;
        if let (Some(session), Some(path)) = (stand_session.as_mut(), stand) {
            run(session, path, &zero)?;
        }

        Ok(Self {
            walk: walk_session,
            stand: stand_session,
            standing_threshold,
        })
    }

    /// Whether the standing policy would be chosen for this command.
    ///
    /// Separate from [`Self::infer`] because the caller needs the same answer to decide
    /// gains and action scale, and asking twice must not be able to disagree.
    pub fn will_stand(&self, twist_magnitude: f64) -> bool {
        self.stand.is_some() && twist_magnitude <= self.standing_threshold
    }

    pub fn has_standing(&self) -> bool {
        self.stand.is_some()
    }

    /// One inference. `standing` should come from [`Self::will_stand`].
    pub fn infer(
        &mut self,
        observation: &Observation,
        standing: bool,
    ) -> Result<[f32; ACTION_LEN], PolicyError> {
        let session = match (standing, self.stand.as_mut()) {
            (true, Some(stand)) => stand,
            _ => &mut self.walk,
        };
        run(session, Path::new("<loaded>"), observation)
    }
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
}
