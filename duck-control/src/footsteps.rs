//! The footstep planner network on ONNX Runtime — `duck_footstep_control::StepNet` over an `ort` session.
//!
//! The planner is FootstepNet exported by microduck_rl `footstepnet/export_onnx.py`: `obs (N, 8) → step (N, 3)`,
//! with the limits and foot geometry it was trained with in the metadata key `options`. Loaded like a policy —
//! runtime probe, panic guard, shape check and a warm-up run, all before the loop drives — because it runs inside
//! the control tick at every touchdown.

use std::path::{Path, PathBuf};

use duck_footstep_control::{Planner, PlannerError, PlannerOptions, StepNet};
use ort::session::Session;
use ort::session::builder::GraphOptimizationLevel;
use ort::value::Value;

use crate::policy::{PolicyError, catching_ort_panics, check_matrix, ensure_runtime, outlet};

pub const PLANNER_OBS_LEN: usize = 8;
pub const PLANNER_STEP_LEN: usize = 3;

pub struct OnnxStepNet {
    session: Session,
    output: String,
    path: PathBuf,
}

impl StepNet for OnnxStepNet {
    fn step(
        &mut self,
        observation: &[f32; PLANNER_OBS_LEN],
    ) -> Result<[f32; PLANNER_STEP_LEN], PlannerError> {
        let fail = |e: String| PlannerError::Net(format!("{}: {e}", self.path.display()));
        if !observation.iter().all(|v| v.is_finite()) {
            return Err(fail("non-finite observation".into()));
        }
        let input = Value::from_array(([1usize, PLANNER_OBS_LEN], observation.to_vec()))
            .map_err(|e| fail(format!("building input: {e}")))?;
        let outputs = self
            .session
            .run(ort::inputs!["obs" => &input])
            .map_err(|e| fail(e.to_string()))?;
        let (_, step) = outputs[self.output.as_str()]
            .try_extract_tensor::<f32>()
            .map_err(|e| fail(e.to_string()))?;
        if step.len() != PLANNER_STEP_LEN {
            return Err(fail(format!(
                "expected {PLANNER_STEP_LEN} values, got {}",
                step.len()
            )));
        }
        Ok([step[0], step[1], step[2]])
    }
}

/// Open, check and warm up a planner. Not from inside a tick: opening a session costs tens of milliseconds.
pub fn load_planner(path: &Path) -> Result<Planner<OnnxStepNet>, PolicyError> {
    ensure_runtime()?;
    catching_ort_panics(|| {
        let session = Session::builder()
            .and_then(|b| b.with_optimization_level(GraphOptimizationLevel::Level3))
            .and_then(|b| b.with_intra_threads(1))
            .and_then(|b| b.commit_from_file(path))
            .map_err(|source| PolicyError::Load {
                path: path.to_owned(),
                source,
            })?;
        check_matrix(
            path,
            outlet(path, session.inputs(), "obs")?,
            PLANNER_OBS_LEN,
        )?;
        let output = outlet(path, session.outputs(), "step")?;
        check_matrix(path, output, PLANNER_STEP_LEN)?;
        let output = output.name().to_owned();
        let shape_error = |got: String| PolicyError::Shape {
            path: path.to_owned(),
            what: "planner metadata `options`",
            expected: "the JSON footstepnet/export_onnx.py writes".into(),
            got,
        };
        let options = session
            .metadata()
            .ok()
            .and_then(|m| m.custom("options"))
            .ok_or_else(|| shape_error("missing".into()))?;
        let options =
            PlannerOptions::from_json(&options).map_err(|e| shape_error(e.to_string()))?;
        let mut planner = Planner::new(
            OnnxStepNet {
                session,
                output,
                path: path.to_owned(),
            },
            options,
        );
        // Warm-up: the first inference is the slow one, and it proves the graph runs.
        let mut zero = [0.0; PLANNER_OBS_LEN];
        zero[2] = 1.0;
        planner
            .step(&zero, duck_footstep_control::Foot::Right)
            .map_err(|e| PolicyError::Inference(e.to_string()))?;
        Ok(planner)
    })
}
