//! The photos the photo sequence keeps, for a phone to show: listed, read a chunk at a time, and
//! deleted — `photo.list`, `photo.get` and `photo.delete` on `nfcd`'s socket.
//!
//! **`nfcd` answers these because it takes the photos.** The directory is its `StateDirectory=`,
//! so it is the one process that may write there anyway, and a preview it makes sits beside the
//! picture it was made from. The socket is served a thread per connection, apart from the poll
//! loop, so a phone's gallery is answered while a sequence is running — the moment someone most
//! wants to look.
//!
//! **A preview is made on first asking, not when the photo is taken.** The sequence is a robot
//! holding a pose while a person watches, and shrinking a picture there would be a pause nobody
//! asked for. Made here, it costs one decode the first time a phone shows it, and photos taken
//! before previews existed get one too. It lives in [`PREVIEWS`], a hidden directory, so a listing
//! of the photos is still a listing of photos.
//!
//! **A name is checked, not resolved.** What a client names is a [`proto::Photo::name`] — letters,
//! digits, `_` and `-` — and anything else is refused before it is joined to a path, so no name
//! reaches outside the directory however it is spelled.
//!
//! Pure functions of a directory, so all of it is tested here on a laptop.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::UNIX_EPOCH;

use base64::Engine as _;
use duck_ipc_proto as proto;

/// Where the previews are kept, inside the photo directory.
pub const PREVIEWS: &str = ".previews";

/// A preview's JPEG quality: a tile in a grid, looked at for a second.
const PREVIEW_QUALITY: u8 = 80;

/// The longest name accepted. The sequence's are 19 characters, 22 with a `-NN`.
const MAX_NAME: usize = 64;

/// Every photo kept in `dir`, newest first. A directory not made yet is no photos.
pub fn list(dir: &Path) -> Result<proto::PhotoListResult, proto::Error> {
    let entries = match std::fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Ok(proto::PhotoListResult::default());
        }
        Err(e) => return Err(internal(dir, e)),
    };
    let mut photos: Vec<proto::Photo> = entries
        .filter_map(Result::ok)
        .filter_map(|entry| {
            let path = entry.path();
            // `.jpg` only: a `.jpg.partial` is a photo still being written.
            if path.extension()? != "jpg" {
                return None;
            }
            let name = path.file_stem()?.to_str()?;
            if !valid(name) {
                return None;
            }
            let meta = entry.metadata().ok().filter(|m| m.is_file())?;
            let taken_s = meta
                .modified()
                .ok()
                .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
                .map_or(0, |d| d.as_secs());
            Some(proto::Photo {
                name: name.to_owned(),
                bytes: meta.len(),
                taken_s,
            })
        })
        .collect();
    // By time, then by name: two in the same second are `-2` after the first, and a robot whose
    // clock jumped still lists in an order the names explain.
    photos.sort_by(|a, b| (b.taken_s, &b.name).cmp(&(a.taken_s, &a.name)));
    Ok(proto::PhotoListResult { photos })
}

/// One chunk of a photo in `dir`, or of its preview.
pub fn get(dir: &Path, params: &proto::PhotoGetParams) -> Result<proto::PhotoChunk, proto::Error> {
    let photo = existing(dir, &params.name)?;
    let path = if params.preview {
        preview(dir, &params.name, &photo)?
    } else {
        photo
    };
    let bytes = std::fs::read(&path).map_err(|e| internal(&path, e))?;
    let total = bytes.len() as u64;
    let start = params.offset.min(total) as usize;
    let end = (start + params.chunk_len() as usize).min(bytes.len());
    Ok(proto::PhotoChunk {
        name: params.name.clone(),
        total,
        offset: params.offset,
        data: base64::engine::general_purpose::STANDARD.encode(&bytes[start..end]),
    })
}

/// Delete a photo in `dir`, and its preview. No such photo is `deleted: false`, not an error.
pub fn delete(dir: &Path, name: &str) -> Result<proto::PhotoDeleteResult, proto::Error> {
    if !valid(name) {
        return Err(not_a_name(name));
    }
    let photo = dir.join(format!("{name}.jpg"));
    let deleted = match std::fs::remove_file(&photo) {
        Ok(()) => true,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => false,
        Err(e) => return Err(internal(&photo, e)),
    };
    // A preview left behind would only be remade for a photo of the same name; still, tidy.
    let _ = std::fs::remove_file(preview_path(dir, name));
    Ok(proto::PhotoDeleteResult { deleted })
}

/// Is this a name the sequence could have given a photo — and nothing that could leave `dir`?
pub fn valid(name: &str) -> bool {
    !name.is_empty()
        && name.len() <= MAX_NAME
        && !name.starts_with('-')
        && name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

/// The photo's path, once it is a valid name of a photo that is there.
fn existing(dir: &Path, name: &str) -> Result<PathBuf, proto::Error> {
    if !valid(name) {
        return Err(not_a_name(name));
    }
    let path = dir.join(format!("{name}.jpg"));
    if !path.is_file() {
        return Err(proto::Error::new(
            proto::code::INVALID_PARAMS,
            format!("no photo named {name}"),
        ));
    }
    Ok(path)
}

fn preview_path(dir: &Path, name: &str) -> PathBuf {
    dir.join(PREVIEWS).join(format!("{name}.jpg"))
}

/// The preview of `photo`, made now if there is none newer than the photo.
fn preview(dir: &Path, name: &str, photo: &Path) -> Result<PathBuf, proto::Error> {
    let path = preview_path(dir, name);
    let modified = |p: &Path| std::fs::metadata(p).and_then(|m| m.modified()).ok();
    if let (Some(made), Some(taken)) = (modified(&path), modified(photo))
        && made >= taken
    {
        return Ok(path);
    }
    let jpeg = std::fs::read(photo).map_err(|e| internal(photo, e))?;
    let small = shrink(&jpeg).map_err(|why| {
        proto::Error::new(
            proto::code::INTERNAL_ERROR,
            format!("a preview of {name}: {why}"),
        )
    })?;
    let parent = path.parent().unwrap_or(dir);
    std::fs::create_dir_all(parent).map_err(|e| internal(parent, e))?;
    // Two phones asking at once each write their own, and the last rename wins: either is whole.
    static WRITES: AtomicU64 = AtomicU64::new(0);
    let partial = path.with_extension(format!(
        "jpg.partial-{}-{}",
        std::process::id(),
        WRITES.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::write(&partial, &small).map_err(|e| internal(&partial, e))?;
    std::fs::rename(&partial, &path).map_err(|e| {
        let _ = std::fs::remove_file(&partial);
        internal(&path, e)
    })?;
    Ok(path)
}

/// A JPEG no longer than [`proto::PhotoGetParams::PREVIEW_SIDE`] on its longer side.
pub fn shrink(jpeg: &[u8]) -> Result<Vec<u8>, String> {
    let side = proto::PhotoGetParams::PREVIEW_SIDE;
    let picture = image::load_from_memory_with_format(jpeg, image::ImageFormat::Jpeg)
        .map_err(|e| format!("reading the photo: {e}"))?
        .thumbnail(side, side)
        .to_rgb8();
    let mut out = Vec::new();
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, PREVIEW_QUALITY)
        .encode(
            &picture,
            picture.width(),
            picture.height(),
            image::ExtendedColorType::Rgb8,
        )
        .map_err(|e| format!("encoding the preview: {e}"))?;
    Ok(out)
}

fn not_a_name(name: &str) -> proto::Error {
    proto::Error::new(
        proto::code::INVALID_PARAMS,
        format!("{name:?} is not a photo's name: letters, digits, _ and - only"),
    )
}

fn internal(path: &Path, e: std::io::Error) -> proto::Error {
    proto::Error::new(
        proto::code::INTERNAL_ERROR,
        format!("{}: {e}", path.display()),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A directory of its own under the system's temporary one, removed when dropped.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(test: &str) -> Self {
            static N: AtomicU64 = AtomicU64::new(0);
            let dir = std::env::temp_dir().join(format!(
                "nfc-gallery-{test}-{}-{}",
                std::process::id(),
                N.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&dir).unwrap();
            Self(dir)
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// A real JPEG, `w` by `h`, of a gradient, so decoding it means something.
    fn jpeg(w: u32, h: u32) -> Vec<u8> {
        let picture = image::RgbImage::from_fn(w, h, |x, y| {
            image::Rgb([(x % 256) as u8, (y % 256) as u8, 128])
        });
        let mut out = Vec::new();
        image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, 90)
            .encode(&picture, w, h, image::ExtendedColorType::Rgb8)
            .unwrap();
        out
    }

    fn put(dir: &Path, name: &str, bytes: &[u8], taken_s: u64) {
        let path = dir.join(name);
        std::fs::write(&path, bytes).unwrap();
        let file = std::fs::File::options().write(true).open(&path).unwrap();
        file.set_modified(UNIX_EPOCH + std::time::Duration::from_secs(taken_s))
            .unwrap();
    }

    fn get_all(dir: &Path, name: &str, preview: bool) -> Vec<u8> {
        let mut out = Vec::new();
        loop {
            let chunk = get(
                dir,
                &proto::PhotoGetParams {
                    name: name.into(),
                    preview,
                    offset: out.len() as u64,
                    len: None,
                },
            )
            .unwrap();
            let bytes = base64::engine::general_purpose::STANDARD
                .decode(&chunk.data)
                .unwrap();
            if bytes.is_empty() {
                assert_eq!(out.len() as u64, chunk.total);
                return out;
            }
            out.extend(bytes);
        }
    }

    #[test]
    fn a_directory_not_made_yet_is_no_photos() {
        let scratch = Scratch::new("none");
        let result = list(&scratch.0.join("photos")).unwrap();
        assert!(result.photos.is_empty());
    }

    #[test]
    fn photos_list_newest_first_and_nothing_else_does() {
        let scratch = Scratch::new("list");
        let dir = &scratch.0;
        put(dir, "2026-11-11_14-03-27.jpg", b"one", 1_000);
        put(dir, "2026-11-11_14-03-27-2.jpg", b"two!", 1_000);
        put(dir, "2026-11-12_09-00-00.jpg", b"three", 2_000);
        // Half a photo, something else, a hidden file, a directory named like a photo.
        put(dir, "2026-11-12_10-00-00.jpg.partial", b"half", 3_000);
        put(dir, "notes.txt", b"x", 3_000);
        put(dir, ".hidden.jpg", b"x", 3_000);
        std::fs::create_dir_all(dir.join("2026-11-13_00-00-00.jpg")).unwrap();
        std::fs::create_dir_all(dir.join(PREVIEWS)).unwrap();

        let names: Vec<_> = list(dir)
            .unwrap()
            .photos
            .into_iter()
            .map(|p| (p.name, p.bytes, p.taken_s))
            .collect();
        assert_eq!(
            names,
            vec![
                ("2026-11-12_09-00-00".into(), 5, 2_000),
                ("2026-11-11_14-03-27-2".into(), 4, 1_000),
                ("2026-11-11_14-03-27".into(), 3, 1_000),
            ]
        );
    }

    #[test]
    fn a_photo_reads_back_whole_a_chunk_at_a_time() {
        let scratch = Scratch::new("chunks");
        let picture = jpeg(1280, 720);
        assert!(
            picture.len() > proto::PhotoGetParams::MAX_LEN as usize,
            "the test wants several chunks"
        );
        put(&scratch.0, "p.jpg", &picture, 1);
        assert_eq!(get_all(&scratch.0, "p", false), picture);
    }

    #[test]
    fn a_chunk_is_capped_and_past_the_end_is_empty() {
        let scratch = Scratch::new("cap");
        put(&scratch.0, "p.jpg", &vec![7u8; 40_000], 1);
        let ask = |offset, len| {
            get(
                &scratch.0,
                &proto::PhotoGetParams {
                    name: "p".into(),
                    preview: false,
                    offset,
                    len,
                },
            )
            .unwrap()
        };
        let decoded = |c: &proto::PhotoChunk| {
            base64::engine::general_purpose::STANDARD
                .decode(&c.data)
                .unwrap()
                .len()
        };
        let huge = ask(0, Some(1_000_000));
        assert_eq!(decoded(&huge), proto::PhotoGetParams::MAX_LEN as usize);
        assert_eq!(huge.total, 40_000);
        assert_eq!(decoded(&ask(39_990, None)), 10);
        let past = ask(50_000, None);
        assert_eq!((past.data.as_str(), past.offset), ("", 50_000));
    }

    #[test]
    fn a_preview_is_small_kept_and_remade_for_a_newer_photo() {
        let scratch = Scratch::new("preview");
        let dir = &scratch.0;
        put(dir, "p.jpg", &jpeg(1920, 1080), 1_000);

        let small = get_all(dir, "p", true);
        let decoded = image::load_from_memory(&small).unwrap();
        assert_eq!(
            (decoded.width(), decoded.height()),
            (proto::PhotoGetParams::PREVIEW_SIDE, 203)
        );
        assert!(small.len() <= proto::PhotoGetParams::MAX_LEN as usize);
        let kept = dir.join(PREVIEWS).join("p.jpg");
        assert_eq!(std::fs::read(&kept).unwrap(), small);

        // A photo replaced under the same name gets a preview of itself, not of the old one.
        put(dir, "p.jpg", &jpeg(400, 800), 5_000_000_000);
        let again = image::load_from_memory(&get_all(dir, "p", true)).unwrap();
        assert_eq!((again.width(), again.height()), (180, 360));
        // And the listing never shows the previews.
        assert_eq!(list(dir).unwrap().photos.len(), 1);
    }

    #[test]
    fn names_that_are_not_a_photos_are_refused_before_any_path() {
        let scratch = Scratch::new("names");
        let outside = scratch.0.join("secret.jpg");
        std::fs::write(&outside, b"no").unwrap();
        let dir = scratch.0.join("photos");
        std::fs::create_dir_all(&dir).unwrap();
        for name in ["", "../secret", "a/b", ".previews/p", "-rf", "p.jpg", "p q"] {
            let asked = proto::PhotoGetParams {
                name: name.into(),
                preview: false,
                offset: 0,
                len: None,
            };
            let refusal = get(&dir, &asked).unwrap_err();
            assert_eq!(refusal.code, proto::code::INVALID_PARAMS, "{name:?}");
            assert!(delete(&dir, name).is_err(), "{name:?}");
        }
        assert!(outside.exists());
        assert!(valid("2026-11-11_14-03-27-2"));
    }

    #[test]
    fn a_photo_not_there_is_said_so() {
        let scratch = Scratch::new("missing");
        let asked = proto::PhotoGetParams {
            name: "gone".into(),
            preview: true,
            offset: 0,
            len: None,
        };
        let refusal = get(&scratch.0, &asked).unwrap_err();
        assert!(refusal.message.contains("no photo named gone"));
    }

    #[test]
    fn delete_takes_the_photo_and_its_preview_and_twice_is_no_error() {
        let scratch = Scratch::new("delete");
        let dir = &scratch.0;
        put(dir, "p.jpg", &jpeg(64, 64), 1);
        get_all(dir, "p", true);
        assert!(dir.join(PREVIEWS).join("p.jpg").exists());

        assert!(delete(dir, "p").unwrap().deleted);
        assert!(!dir.join("p.jpg").exists());
        assert!(!dir.join(PREVIEWS).join("p.jpg").exists());
        assert!(!delete(dir, "p").unwrap().deleted);
    }
}
