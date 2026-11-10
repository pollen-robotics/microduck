//! Observational BLE status, published outside the radio loop like mediad's diagnostics.

use duck_ipc_proto::{BlePresenceScan, BlePresenceStatus};
use tokio::sync::watch;

pub(crate) type Sender = watch::Sender<BlePresenceStatus>;

pub(crate) fn now() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |age| age.as_secs().min(i64::MAX as u64) as i64)
}

pub(crate) fn initial(scan: BlePresenceScan) -> BlePresenceStatus {
    BlePresenceStatus {
        observed_at: now(),
        requested: None,
        scan,
        attempts: 0,
        last_sighting: None,
        last_error: None,
    }
}

pub(crate) fn bounded_error(error: impl std::fmt::Display) -> String {
    let text = error.to_string();
    let mut end = text.len().min(512);
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    text[..end].to_owned()
}

pub(crate) fn start() -> (Sender, tokio::task::JoinHandle<()>) {
    let (tx, rx) = watch::channel(initial(BlePresenceScan::Unavailable {
        reason: "waiting for a Bluetooth adapter".into(),
    }));
    let task = tokio::spawn(publish_loop(duck_ipc_proto::ble_presence_path(), rx));
    (tx, task)
}

async fn publish_loop(path: std::path::PathBuf, mut rx: watch::Receiver<BlePresenceStatus>) {
    let mut tick = tokio::time::interval(std::time::Duration::from_secs(1));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            changed = rx.changed() => if changed.is_err() { return; },
            _ = tick.tick() => {
                let snapshot = rx.borrow_and_update().clone();
                // File I/O must not stall btd's single-threaded GATT dispatcher. One
                // publisher awaits each write, coalescing radio updates to at most 1 Hz.
                let path = path.clone();
                let result = tokio::task::spawn_blocking(move || {
                    duck_ipc_proto::publish_ble_presence(
                        &path, &snapshot,
                    )
                }).await;
                match result {
                    Ok(Ok(())) => {}
                    Ok(Err(error)) => tracing::warn!(%error, "cannot publish BLE presence status"),
                    Err(error) => tracing::warn!(%error, "BLE presence publisher failed"),
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Truncating a D-Bus error in the middle of a UTF-8 code point must not panic.
    #[test]
    fn error_summary_is_bounded_on_utf8_boundaries() {
        let original = "界".repeat(200);
        let summary = bounded_error(&original);
        assert!(summary.len() <= 512);
        assert!(original.starts_with(&summary));
        assert_eq!(summary.chars().count(), 170);
    }

    /// A live file writer must not turn a frozen radio observation into fresh telemetry.
    #[tokio::test]
    async fn publishing_preserves_owner_freshness_and_writes_latest_snapshot() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("chorale.json");
        let mut first = initial(BlePresenceScan::Disconnected);
        first.observed_at = 1;
        let (tx, rx) = watch::channel(first);
        let mut publisher = tokio::task::JoinSet::new();
        publisher.spawn(publish_loop(path.clone(), rx));
        wait_for(&path, 1).await;
        tx.send_modify(|s| {
            s.observed_at = 2;
            s.scan = BlePresenceScan::Off;
        });
        let current = wait_for(&path, 2).await;
        assert!(matches!(current.scan, BlePresenceScan::Off));
        assert_eq!(current.observed_at, 2);
        drop(tx);
        tokio::time::timeout(std::time::Duration::from_secs(2), publisher.join_next())
            .await
            .unwrap()
            .unwrap()
            .unwrap();
    }

    async fn wait_for(path: &std::path::Path, at: i64) -> BlePresenceStatus {
        tokio::time::timeout(std::time::Duration::from_secs(3), async {
            loop {
                if let Some(status) = duck_ipc_proto::read_ble_presence_at(path)
                    && status.observed_at == at
                {
                    return status;
                }
                tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            }
        })
        .await
        .unwrap()
    }
}
