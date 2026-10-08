//! Controller edits use the same schema and atomic writer as the terminal editor.
use duck_ipc_proto as proto;
use robotd_params::{
    edit::{Edit, Model},
    registry::{Kind, REGISTRY},
};
use serde_json::Value;
use std::path::Path;

fn controller_key(key: &str) -> bool {
    matches!(
        key.split('.').next(),
        Some(
            "pad"
                | "pad_axes"
                | "pad_drive"
                | "pad_head"
                | "pad_body"
                | "pad_roller"
                | "pad_imu_head_control"
        )
    )
}

fn value(kind: Kind, text: &str) -> Result<Value, String> {
    match kind {
        Kind::Bool => text
            .parse::<bool>()
            .map(Value::Bool)
            .map_err(|e| e.to_string()),
        Kind::Float => text
            .parse::<f64>()
            .ok()
            .and_then(serde_json::Number::from_f64)
            .map(Value::Number)
            .ok_or_else(|| "expected finite number".to_owned()),
        Kind::Choice(_) | Kind::Text => Ok(Value::String(text.to_owned())),
        _ => Err("unsupported controller setting".to_owned()),
    }
}

pub fn report(path: &Path, skills: Vec<String>) -> Result<proto::PadConfigResult, String> {
    let model = Model::load(path)?;
    let settings = model
        .rows()
        .into_iter()
        .filter(|row| controller_key(row.entry.key))
        .map(|row| {
            let (kind, choices) = match row.entry.kind {
                Kind::Bool => (proto::PadSettingKind::Boolean, vec![]),
                Kind::Float => (proto::PadSettingKind::Number, vec![]),
                Kind::Choice(choices) => (
                    proto::PadSettingKind::Choice,
                    choices.iter().map(|s| (*s).to_owned()).collect(),
                ),
                Kind::Text => (proto::PadSettingKind::Skill, skills.clone()),
                _ => return Err("unsupported controller setting".to_owned()),
            };
            Ok(proto::PadSetting {
                key: row.entry.key.to_owned(),
                kind,
                description: row.entry.doc.to_owned(),
                value: value(row.entry.kind, row.effective())?,
                default_value: value(row.entry.kind, &row.default)?,
                overridden: row.overridden(),
                choices,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    Ok(proto::PadConfigResult { settings })
}

pub fn apply(path: &Path, patch: &proto::PadConfigPatch, skills: &[String]) -> proto::IntentResult {
    match apply_checked(path, patch, skills) {
        Ok(()) => proto::IntentResult::accepted(),
        Err(reason) => proto::IntentResult::refused(reason),
    }
}

fn apply_checked(
    path: &Path,
    patch: &proto::PadConfigPatch,
    skills: &[String],
) -> Result<(), String> {
    let mut model = Model::load(path)?;
    for (key, input) in &patch.changes {
        let entry = REGISTRY
            .iter()
            .find(|entry| entry.key == key && controller_key(key))
            .ok_or_else(|| format!("not a controller setting: {key}"))?;
        let Some(input) = input else {
            model.pending.insert(entry.key, Edit::Clear);
            continue;
        };
        let text = match (entry.kind, input) {
            (Kind::Bool, Value::Bool(v)) => v.to_string(),
            (Kind::Float, Value::Number(v)) => v.to_string(),
            (Kind::Choice(_), Value::String(v)) => v.clone(),
            (Kind::Text, Value::String(v)) if v.is_empty() || skills.contains(v) => v.clone(),
            (Kind::Text, Value::String(v)) => {
                return Err(format!("this robot has no skill called {v:?}"));
            }
            _ => return Err(format!("wrong value type for {key}")),
        };
        model.edit(entry, &text)?;
    }
    // Validation and persistence happen once: no partial save if one field is invalid.
    if !model.pending.is_empty() {
        model.save()?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    #[test]
    fn report_uses_the_robots_defaults_and_lists_only_controller_settings() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, "[pad_axes.drive.vyaw]\ngain = 0.5\n").unwrap();
        let report = report(&path, vec!["sit_toggle".to_owned()]).unwrap();
        assert!(report.settings.iter().all(|s| controller_key(&s.key)));
        let gain = report
            .settings
            .iter()
            .find(|s| s.key == "pad_axes.drive.vyaw.gain")
            .unwrap();
        assert_eq!(gain.value, json!(0.5));
        assert_eq!(gain.default_value, json!(1.0));
        assert!(gain.overridden);
        let source = report
            .settings
            .iter()
            .find(|s| s.key == "pad_axes.drive.vyaw.source")
            .unwrap();
        assert_eq!(source.value, json!("right_x"));
        assert!(source.choices.contains(&"left_y".to_owned()));
    }
    #[test]
    fn batch_edits_are_atomic_typed_and_controller_only() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        let original = "# keep this\n[control]\nhz = 50\n[pad_axes.drive.vyaw]\ngain = 0.5\n";
        std::fs::write(&path, original).unwrap();
        for changes in [
            json!({"pad.a":"sit_toggle", "pad_axes.deadzone":1.0}),
            json!({"control.hz":30}),
            json!({"pad.a":"missing_skill"}),
            json!({"pad_axes.deadzone":"0.2"}),
            json!({"pad_axes.drive.vx.source":"typo"}),
        ] {
            let patch = proto::PadConfigPatch {
                changes: serde_json::from_value(changes).unwrap(),
            };
            assert!(!apply(&path, &patch, &["sit_toggle".to_owned()]).accepted);
            assert_eq!(std::fs::read_to_string(&path).unwrap(), original);
        }
        let patch = proto::PadConfigPatch { changes: serde_json::from_value(json!({"pad.a":"", "pad_axes.drive.vyaw.gain":null, "pad_axes.drive.vx.source":"right_y"})).unwrap() };
        assert!(apply(&path, &patch, &[]).accepted);
        let params = robotd_params::Params::load(&path, true).unwrap();
        assert_eq!(params.pad.a, "");
        assert_eq!(params.pad_axes.drive.vyaw.gain, 1.0);
        assert_eq!(
            params.pad_axes.drive.vx.source,
            robotd_params::pad_axes::AxisSource::RightY
        );
        let written = std::fs::read_to_string(&path).unwrap();
        assert!(written.contains("# keep this"));
        assert!(written.contains("hz = 50"));
        assert!(!written.contains("gain"));
    }
}
