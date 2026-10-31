//! `vetbook.share` against a fake Hugging Face and fake `robotd` and `configd` sockets: what it
//! creates, what it commits, and what it refuses to do without an account.

use std::sync::{Arc, Mutex};

use axum::Router;
use axum::extract::{Path, State};
use axum::http::{HeaderMap, StatusCode};
use axum::routing::post;
use base64::Engine as _;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt};
use updater::proto;

#[derive(Default)]
struct Hub {
    /// Bodies of `POST /api/repos/create`.
    created: Vec<serde_json::Value>,
    /// `(repo, ndjson body)` of every commit.
    commits: Vec<(String, String)>,
    /// Bearer tokens seen.
    tokens: Vec<String>,
}

type Shared = Arc<Mutex<Hub>>;

fn bearer(headers: &HeaderMap) -> String {
    headers
        .get("authorization")
        .and_then(|v| v.to_str().ok())
        .unwrap_or_default()
        .to_owned()
}

async fn fake_hub(exists: bool) -> (String, Shared) {
    let hub: Shared = Arc::default();
    let app = Router::new()
        .route(
            "/api/repos/create",
            post(
                move |State(hub): State<Shared>, headers: HeaderMap, body: String| async move {
                    let mut h = hub.lock().unwrap();
                    h.tokens.push(bearer(&headers));
                    h.created.push(serde_json::from_str(&body).unwrap());
                    if exists {
                        (
                            StatusCode::CONFLICT,
                            "You already created this dataset repo",
                        )
                    } else {
                        (StatusCode::OK, "{}")
                    }
                },
            ),
        )
        .route(
            "/api/datasets/{user}/{repo}/commit/main",
            post(
                |State(hub): State<Shared>,
                 Path((user, repo)): Path<(String, String)>,
                 headers: HeaderMap,
                 body: String| async move {
                    let mut h = hub.lock().unwrap();
                    h.tokens.push(bearer(&headers));
                    h.commits.push((format!("{user}/{repo}"), body));
                    (StatusCode::OK, "{}")
                },
            ),
        )
        .with_state(Arc::clone(&hub));
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let base = format!("http://{}", listener.local_addr().unwrap());
    tokio::spawn(async move {
        let _ = axum::serve(listener, app).await;
    });
    (base, hub)
}

/// A daemon that answers every request on `path` with `result`.
fn fake_daemon(path: std::path::PathBuf, result: serde_json::Value) {
    let listener = tokio::net::UnixListener::bind(&path).unwrap();
    tokio::spawn(async move {
        loop {
            let Ok((stream, _)) = listener.accept().await else {
                return;
            };
            let result = result.clone();
            tokio::spawn(async move {
                let (read, mut write) = stream.into_split();
                let mut line = String::new();
                tokio::io::BufReader::new(read)
                    .read_line(&mut line)
                    .await
                    .unwrap();
                let request: serde_json::Value = serde_json::from_str(&line).unwrap();
                let reply =
                    serde_json::json!({ "jsonrpc": "2.0", "id": request["id"], "result": result });
                let mut out = serde_json::to_vec(&reply).unwrap();
                out.push(b'\n');
                write.write_all(&out).await.unwrap();
            });
        }
    });
}

fn book() -> serde_json::Value {
    let mut book = proto::vetbook::Vetbook::open(1_760_000_000);
    book.life.falls = 7;
    book.last_boot_id = "boot-1234".into();
    book.event(1_760_000_100, "note", "the spare key is under the mat");
    serde_json::to_value(book).unwrap()
}

struct Fixture {
    _dir: tempfile::TempDir,
    account: updater::account::Account,
    sources: updater::vetbook::Sources,
}

async fn fixture(endpoint: &str, signed_in: bool) -> Fixture {
    let dir = tempfile::tempdir().unwrap();
    let token = dir.path().join("hf-token");
    if signed_in {
        std::fs::write(
            &token,
            r#"{"access_token":"hf_test_token","username":"apirrone","expires_at":4102444800}"#,
        )
        .unwrap();
    }
    let robot_socket = dir.path().join("robotd.sock");
    let config_socket = dir.path().join("configd.sock");
    fake_daemon(robot_socket.clone(), book());
    fake_daemon(
        config_socket.clone(),
        serde_json::json!({ "name": "Graphite Bang", "serial": null, "uptime_seconds": 1 }),
    );
    Fixture {
        account: updater::account::account_for_test(&token, endpoint.to_owned()),
        sources: updater::vetbook::Sources {
            robot_socket,
            config_socket,
        },
        _dir: dir,
    }
}

fn committed_files(body: &str) -> (serde_json::Value, Vec<(String, String)>) {
    let mut lines = body
        .lines()
        .map(|l| serde_json::from_str::<serde_json::Value>(l).unwrap());
    let header = lines.next().unwrap();
    let files = lines
        .map(|l| {
            assert_eq!(l["key"], "file");
            assert_eq!(l["value"]["encoding"], "base64");
            let bytes = base64::engine::general_purpose::STANDARD
                .decode(l["value"]["content"].as_str().unwrap())
                .unwrap();
            (
                l["value"]["path"].as_str().unwrap().to_owned(),
                String::from_utf8(bytes).unwrap(),
            )
        })
        .collect();
    (header, files)
}

#[tokio::test]
async fn a_share_creates_a_public_dataset_and_commits_the_book() {
    let (endpoint, hub) = fake_hub(false).await;
    let fx = fixture(&endpoint, true).await;

    let result = updater::vetbook::share(&fx.account, &fx.sources, &Default::default())
        .await
        .unwrap();

    assert_eq!(result.dataset, "apirrone/graphite-bang-vetbook");
    assert_eq!(
        result.url,
        format!("{}/d/apirrone/graphite-bang", updater::vetbook::SPACE_URL)
    );
    // A new image address each share, so the README's image and link previews are not stale.
    assert!(
        result
            .image
            .starts_with(&format!("{}/card.png?v=", result.url))
    );
    assert!(updater::vetbook::CARDS.contains(&result.card.as_str()));

    let hub = hub.lock().unwrap();
    assert_eq!(
        hub.created,
        vec![
            serde_json::json!({ "type": "dataset", "name": "graphite-bang-vetbook", "private": false })
        ]
    );
    assert!(hub.tokens.iter().all(|t| t == "Bearer hf_test_token"));
    assert_eq!(hub.commits.len(), 1);
    let (repo, body) = &hub.commits[0];
    assert_eq!(repo, "apirrone/graphite-bang-vetbook");

    let (header, files) = committed_files(body);
    assert_eq!(header["key"], "header");
    let names: Vec<&str> = files.iter().map(|(p, _)| p.as_str()).collect();
    assert_eq!(names, ["vetbook.json", "README.md"]);

    let shared: serde_json::Value = serde_json::from_str(&files[0].1).unwrap();
    assert_eq!(shared["life"]["falls"], 7);
    assert_eq!(shared["share"]["robot"], "Graphite Bang");
    assert_eq!(shared["share"]["owner"], "apirrone");
    assert_eq!(shared["share"]["card"], result.card.as_str());
    assert!(
        !files[0].1.contains("spare key"),
        "a diary note was published"
    );
    assert!(
        !files[0].1.contains("boot-1234"),
        "robotd's bookkeeping was published"
    );
    assert!(files[1].1.contains("Graphite Bang's vet book"));
    assert!(files[1].1.contains(&result.image));
    // Not asked to be listed: the book is public, but out of the gallery.
    assert!(!result.listed);
    assert_eq!(shared["share"]["listed"], false);
    assert!(!files[1].1.contains(updater::vetbook::GALLERY_TAG));
}

#[tokio::test]
async fn a_listed_share_says_so_in_the_book_and_the_tags() {
    let (endpoint, hub) = fake_hub(false).await;
    let fx = fixture(&endpoint, true).await;
    let params = proto::VetbookShareParams { listed: true };
    let result = updater::vetbook::share(&fx.account, &fx.sources, &params)
        .await
        .unwrap();
    assert!(result.listed);

    let hub = hub.lock().unwrap();
    let (_, files) = committed_files(&hub.commits[0].1);
    let shared: serde_json::Value = serde_json::from_str(&files[0].1).unwrap();
    assert_eq!(shared["share"]["listed"], true);
    assert!(
        files[1]
            .1
            .contains(&format!("- {}\n", updater::vetbook::GALLERY_TAG)),
        "{}",
        files[1].1
    );
}

#[tokio::test]
async fn sharing_again_reuses_the_dataset() {
    let (endpoint, hub) = fake_hub(true).await;
    let fx = fixture(&endpoint, true).await;
    updater::vetbook::share(&fx.account, &fx.sources, &Default::default())
        .await
        .unwrap();
    assert_eq!(hub.lock().unwrap().commits.len(), 1);
}

#[tokio::test]
async fn a_robot_with_no_account_is_told_to_sign_in() {
    let (endpoint, hub) = fake_hub(false).await;
    let fx = fixture(&endpoint, false).await;
    let err = updater::vetbook::share(&fx.account, &fx.sources, &Default::default())
        .await
        .unwrap_err();
    assert!(err.to_string().contains("robotctl account login"), "{err}");
    assert!(hub.lock().unwrap().created.is_empty());
}
