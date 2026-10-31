//! Sharing the vet book: `vetbook.share`.
//!
//! The book is `robotd`'s (`duck_ipc_proto::vetbook`). This publishes a copy of it to a **public**
//! dataset on the signed-in Hugging Face account, `<user>/<robot name>-vetbook`, and answers with
//! the link to its card on the `microduck-vetbook` Space (`spaces/vetbook/`), which draws the card
//! from the dataset and is what a person pastes into a post.
//!
//! Here rather than in `robotd` because this daemon holds the account and the network stack, and
//! `robotd` has neither on purpose. It asks `robotd` for the book and `configd` for the name, and
//! does the rest itself:
//!
//! 1. **Create the dataset**, public, if it is not there. A `409` means it is, which is the
//!    ordinary case from the second share on.
//! 2. **Commit two files**, `vetbook.json` and a `README.md` dataset card, in one commit. Every
//!    share replaces both; the dataset's history keeps the old books.
//!
//! **What leaves the robot is not quite the book.** Diary notes are free text somebody typed on
//! the robot, never written with an audience in mind, so `note` events are dropped; and the two
//! bookkeeping fields `robotd` uses to tell a boot from a restart are dropped with them. Everything
//! else is counters, dates and servo wear. A `share` block is added: the robot's name, the owner,
//! when, from which release, and which card this share drew.
//!
//! **Listing is the owner's choice, made on every share.** The dataset is public either way —
//! the card's link reads it — but only a share that says `listed` puts the duck in the Space's
//! gallery, where strangers browse, search and sort ducks. Two marks say so, and the Space wants
//! both: the dataset card's [`GALLERY_TAG`], which is how it finds the books without reading every
//! one, and `share.listed` in the book, which is what it believes. Anybody can tag a dataset; only
//! this share writes `listed`. Sharing unlisted rewrites both, which takes the duck back out.
//!
//! **The card is drawn at random**, one of three, when the book is shared — so a link keeps its
//! card for as long as it stands, and sharing again re-rolls it.
//!
//! **The repo name follows the robot's name**, so renaming a duck starts a new dataset and the old
//! one stays where it was. The user decided that; the alternative, a dataset keyed on something
//! stable but meaningless, makes a worse link.
//!
//! The account's token is used as it is, and it can write to the owner's repositories:
//! `remote-access-design.md` §2.4 has why it carries that scope and what narrowing it would cost.

use std::path::{Path, PathBuf};
use std::time::Duration;

use base64::Engine as _;

use crate::Error;
use crate::proto;

type Result<T> = std::result::Result<T, Error>;

/// Where the cards are drawn: the `microduck-vetbook` Space's own domain, so the link's preview
/// image is the card rather than Hugging Face's page around it.
pub const SPACE_URL: &str = "https://pollen-robotics-microduck-vetbook.hf.space";

/// The cards a share can draw, as the Space names them. The Space draws any of its own for a name
/// it does not know, so an older robot drawing from a shorter list still gets a card.
pub const CARDS: [&str; 8] = [
    "notebook", "purikura", "profile", "receipt", "passport", "gameboy", "polaroid", "fifa",
];

/// The dataset tag a listed book carries: the gallery asks the Hub for datasets with it.
pub const GALLERY_TAG: &str = "microduck-vetbook-gallery";

/// How long `robotd` and `configd` each get to answer. Both answer from memory.
const ASK_TIMEOUT: Duration = Duration::from_secs(5);

/// One Hugging Face round trip.
const HUB_TIMEOUT: Duration = Duration::from_secs(30);

/// Who to ask for the book and the name.
#[derive(Debug, Clone)]
pub struct Sources {
    pub robot_socket: PathBuf,
    pub config_socket: PathBuf,
}

impl Default for Sources {
    fn default() -> Self {
        Self {
            robot_socket: PathBuf::from(proto::socket::ROBOT),
            config_socket: PathBuf::from(proto::socket::CONFIG),
        }
    }
}

/// `vetbook.share`.
pub async fn share(
    account: &crate::account::Account,
    sources: &Sources,
    params: &proto::VetbookShareParams,
) -> Result<proto::VetbookShareResult> {
    let listed = params.listed;
    // A token near its end is renewed first, so a share is not the thing that finds out.
    if let Err(e) = account.refresh_if_due().await {
        tracing::warn!(error = %e, "vet book share: token refresh failed; trying the one we have");
    }
    let Some(stored) = account.store().load() else {
        return Err(Error::Network(
            "this robot is not signed in to Hugging Face, so there is no account to share to: \
             run `robotctl account login` first"
                .into(),
        ));
    };
    let user = match stored.username.as_deref() {
        Some(user) if !user.is_empty() && user != "unknown" => user.to_owned(),
        _ => {
            // The login's last step, reading who the account is, can fail on a bad network; the
            // maintenance loop fills it in later. Ask now rather than wait for it.
            let _ = account.name_if_unknown().await;
            account
                .store()
                .load()
                .and_then(|s| s.username)
                .filter(|u| !u.is_empty() && u != "unknown")
                .ok_or_else(|| {
                    Error::Network(
                        "cannot tell which Hugging Face account this robot is signed in to; \
                         check its network and try again"
                            .into(),
                    )
                })?
        }
    };

    let Some(book) = crate::robot::ask_socket(
        &sources.robot_socket,
        &proto::Call::RobotVetbook,
        ASK_TIMEOUT,
    )
    .await
    else {
        return Err(Error::Network(format!(
            "robotd did not hand over the vet book (asked at {}); is it running?",
            sources.robot_socket.display()
        )));
    };
    let name = robot_name(&sources.config_socket).await;

    let card = draw_card();
    let slug = slug(&name);
    let repo = format!("{user}/{slug}-vetbook");
    let url = format!("{SPACE_URL}/d/{user}/{slug}");
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs() as i64);
    let shared = shared_copy(book, &name, &user, card, now, listed);
    // This share's own image address: the Space draws the latest book whatever `v` says, but a new
    // address is what makes Hugging Face's image proxy (the README) and a social network's preview
    // fetch it again rather than show the last share's card.
    let image = format!("{url}/card.png?v={now}");
    let readme = dataset_card(&name, &user, &url, &image, listed);

    let endpoint = account.config().endpoint.trim_end_matches('/').to_owned();
    let client = crate::source::http::client()?;
    create_dataset(
        &client,
        &endpoint,
        &stored.access_token,
        &format!("{slug}-vetbook"),
    )
    .await?;
    commit(
        &client,
        &endpoint,
        &stored.access_token,
        &repo,
        &format!("{name}'s vet book, shared from the robot"),
        &[
            (
                "vetbook.json",
                serde_json::to_vec_pretty(&shared).expect("a JSON value serialises"),
            ),
            ("README.md", readme.into_bytes()),
        ],
    )
    .await?;

    tracing::info!(%repo, card, listed, "vet book shared");
    Ok(proto::VetbookShareResult {
        image,
        url,
        dataset: repo,
        card: card.to_owned(),
        listed,
    })
}

/// The robot's name from `configd`, or the hostname, or `microduck`.
async fn robot_name(config_socket: &Path) -> String {
    crate::robot::ask_socket(config_socket, &proto::Call::SystemInfo, ASK_TIMEOUT)
        .await
        .and_then(|v| serde_json::from_value::<proto::SystemInfoResult>(v).ok())
        .map(|info| info.name)
        .filter(|n| !n.trim().is_empty())
        .or_else(|| {
            std::fs::read_to_string("/etc/hostname")
                .ok()
                .map(|h| h.trim().to_owned())
                .filter(|h| !h.is_empty())
        })
        .unwrap_or_else(|| "microduck".to_owned())
}

/// One of [`CARDS`], from the clock's nanoseconds — a share is not a lottery, it only has to vary.
fn draw_card() -> &'static str {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.subsec_nanos());
    CARDS[(nanos as usize / 1000) % CARDS.len()]
}

/// A robot name as a repo name: lowercase, `[a-z0-9-_.]`, no runs of dashes, at most 80 characters
/// so `-vetbook` still fits Hugging Face's 96.
pub fn slug(name: &str) -> String {
    let mut out = String::new();
    for c in name.trim().chars().flat_map(char::to_lowercase) {
        let c = if c.is_ascii_alphanumeric() || matches!(c, '_' | '.') {
            c
        } else {
            '-'
        };
        if c == '-' && out.ends_with('-') {
            continue;
        }
        out.push(c);
    }
    let out: String = out
        .trim_matches(|c| c == '-' || c == '.')
        .chars()
        .take(80)
        .collect();
    let out = out.trim_end_matches(['-', '.']).to_owned();
    if out.is_empty() {
        "microduck".to_owned()
    } else {
        out
    }
}

/// The book as it is published: without diary notes and `robotd`'s bookkeeping, with a `share`
/// block saying whose, when, which card, and whether the gallery may show it.
pub fn shared_copy(
    mut book: serde_json::Value,
    name: &str,
    user: &str,
    card: &str,
    now: i64,
    listed: bool,
) -> serde_json::Value {
    if let Some(obj) = book.as_object_mut() {
        obj.remove("last_boot_id");
        obj.remove("last_volts");
        if let Some(events) = obj.get_mut("events").and_then(|e| e.as_array_mut()) {
            events.retain(|e| e.get("kind").and_then(|k| k.as_str()) != Some("note"));
        }
        obj.insert(
            "share".into(),
            serde_json::json!({
                "robot": name,
                "owner": user,
                "card": card,
                "shared_at": now,
                "daemon": env!("CARGO_PKG_VERSION"),
                "listed": listed,
            }),
        );
    }
    book
}

/// The dataset's `README.md`: what this is, the card, and where it came from. A listed book's
/// carries [`GALLERY_TAG`].
pub fn dataset_card(name: &str, user: &str, url: &str, image: &str, listed: bool) -> String {
    let gallery = if listed {
        format!("- {GALLERY_TAG}\n")
    } else {
        String::new()
    };
    format!(
        "---\npretty_name: \"{name}'s vet book\"\ntags:\n- microduck\n- robotics\n- pollen-robotics\n\
         - vetbook\n{gallery}---\n\n# {name}'s vet book\n\n[![{name}'s vet book card]({image})]({url})\n\n\
         The lifetime record of **{name}**, a [microduck](https://pollen-robotics.com) belonging to \
         [@{user}](https://huggingface.co/{user}): hours awake, distance walked, falls, cuddles, \
         tricks, and every servo it has been through. Shared from the robot with \
         `robotctl vetbook`.\n\n`vetbook.json` is the book itself. Each share replaces it; this \
         dataset's history keeps the older ones.\n\n[Open the card]({url})\n"
    )
}

async fn create_dataset(
    client: &reqwest::Client,
    endpoint: &str,
    token: &str,
    name: &str,
) -> Result<()> {
    let url = format!("{endpoint}/api/repos/create");
    let response = client
        .post(&url)
        .bearer_auth(token)
        .timeout(HUB_TIMEOUT)
        .json(&serde_json::json!({ "type": "dataset", "name": name, "private": false }))
        .send()
        .await
        .map_err(|e| Error::Network(format!("creating the dataset: {e}")))?;
    match response.status().as_u16() {
        200..=299 | 409 => Ok(()),
        status => Err(refused("creating the dataset", status, response).await),
    }
}

async fn commit(
    client: &reqwest::Client,
    endpoint: &str,
    token: &str,
    repo: &str,
    summary: &str,
    files: &[(&str, Vec<u8>)],
) -> Result<()> {
    let b64 = base64::engine::general_purpose::STANDARD;
    let mut body = serde_json::to_string(&serde_json::json!({
        "key": "header",
        "value": { "summary": summary, "description": "" },
    }))
    .expect("a JSON value serialises");
    for (path, content) in files {
        body.push('\n');
        body.push_str(
            &serde_json::to_string(&serde_json::json!({
                "key": "file",
                "value": { "path": path, "content": b64.encode(content), "encoding": "base64" },
            }))
            .expect("a JSON value serialises"),
        );
    }
    let url = format!("{endpoint}/api/datasets/{repo}/commit/main");
    let response = client
        .post(&url)
        .bearer_auth(token)
        .timeout(HUB_TIMEOUT)
        .header(reqwest::header::CONTENT_TYPE, "application/x-ndjson")
        .body(body)
        .send()
        .await
        .map_err(|e| Error::Network(format!("uploading the vet book: {e}")))?;
    let status = response.status().as_u16();
    if (200..300).contains(&status) {
        Ok(())
    } else {
        Err(refused("uploading the vet book", status, response).await)
    }
}

/// A Hub refusal, worded for whoever pressed share.
async fn refused(doing: &str, status: u16, response: reqwest::Response) -> Error {
    let detail: String = response
        .text()
        .await
        .unwrap_or_default()
        .chars()
        .take(300)
        .collect();
    let hint = match status {
        401 => ": the robot's sign-in is no longer valid; run `robotctl account login` again",
        403 => ": the account's token may not write repositories",
        _ => "",
    };
    Error::Network(format!(
        "{doing}: Hugging Face answered {status}{hint} ({})",
        detail.trim()
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_robot_name_becomes_a_repo_name() {
        assert_eq!(slug("graphite-bang"), "graphite-bang");
        assert_eq!(slug("  Lavender Star!! "), "lavender-star");
        assert_eq!(slug("duck.v2__beta"), "duck.v2__beta");
        assert_eq!(slug("---"), "microduck");
        assert_eq!(slug("ça va"), "a-va");
        assert!(slug(&"x".repeat(200)).len() <= 80);
    }

    #[test]
    fn notes_and_bookkeeping_stay_on_the_robot() {
        let book = serde_json::json!({
            "life": { "falls": 3 },
            "last_boot_id": "abc",
            "last_volts": 7.9,
            "events": [
                { "kind": "opened", "text": "the vet book was opened" },
                { "kind": "note", "text": "my wifi password is hunter2" },
                { "kind": "swap", "text": "left_knee (id 23): new servo" },
            ],
        });
        let shared = shared_copy(book, "graphite-bang", "apirrone", "purikura", 42, false);
        let text = shared.to_string();
        assert!(!text.contains("hunter2"));
        assert!(!text.contains("last_boot_id"));
        assert!(!text.contains("last_volts"));
        assert_eq!(shared["events"].as_array().unwrap().len(), 2);
        assert_eq!(shared["life"]["falls"], 3);
        assert_eq!(shared["share"]["card"], "purikura");
        assert_eq!(shared["share"]["owner"], "apirrone");
        assert_eq!(shared["share"]["robot"], "graphite-bang");
        assert_eq!(shared["share"]["listed"], false);
    }

    #[test]
    fn only_a_listed_book_carries_the_gallery_tag() {
        let args = (
            "graphite-bang",
            "apirrone",
            "https://x/d/a/g",
            "https://x/d/a/g/card.png",
        );
        let listed = dataset_card(args.0, args.1, args.2, args.3, true);
        let unlisted = dataset_card(args.0, args.1, args.2, args.3, false);
        assert!(
            listed.contains(&format!("\n- {GALLERY_TAG}\n---\n")),
            "{listed}"
        );
        assert!(!unlisted.contains(GALLERY_TAG), "{unlisted}");
        assert!(unlisted.contains("\n- vetbook\n---\n"), "{unlisted}");
    }

    #[test]
    fn every_card_can_be_drawn() {
        let mut seen = std::collections::HashSet::new();
        for _ in 0..2000 {
            seen.insert(draw_card());
            std::thread::sleep(Duration::from_micros(7));
        }
        assert_eq!(seen.len(), CARDS.len());
    }
}
