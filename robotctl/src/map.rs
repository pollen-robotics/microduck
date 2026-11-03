//! `robotctl map`: where the robot is in its map, the map drawn in the terminal, wiping it, and
//! putting another robot's map in its place.
//!
//! `mapd`'s, on its own socket — like the depth stream, this is a client of a daemon that is not
//! `robotd`.

use std::path::{Path, PathBuf};

use clap::Subcommand;
use duck_ipc_proto as proto;

use crate::{Client, Failure, decode, exit, result_of};

#[derive(Subcommand, Debug)]
pub enum MapCommand {
    /// Where the robot is in its map, and how the map is doing. The default.
    Status,
    /// Draw the map in the terminal: `#` obstacles, `.` floor seen clear, `o` stops, `@` the robot.
    Show {
        /// Cell size, metres. Coarser fits a bigger house on the screen.
        #[arg(long, default_value_t = 0.1)]
        res: f64,
        /// Also write the grid as a PGM image here (unknown grey, floor white, obstacles black).
        #[arg(long)]
        pgm: Option<PathBuf>,
    },
    /// Forget the map. The robot starts a new one at its next stop, from wherever it stands.
    Wipe {
        /// Do it without asking.
        #[arg(long)]
        yes: bool,
    },
    /// Freeze the map and only find the robot in it (`localize`), or map again (`map`). Not kept
    /// across a restart of mapd, which always starts mapping. `robotctl stress` freezes it for
    /// its run.
    Mode {
        #[arg(value_parser = ["localize", "map"])]
        mode: String,
    },
    /// Replace this robot's map with FILE, a `map.json` copied off another robot — so robots
    /// tested in the same room keep off the same walls (`robotctl stress`). Stops `mapd`, keeps
    /// the map it had beside the new one as `map.json.before-load-<seconds>`, and starts it
    /// again. Needs root.
    ///
    /// The robot is lost in the new map until it recognises the room: a stop or two somewhere the
    /// map has seen.
    Load {
        /// The map to install: `/var/lib/mapd/map.json` from the robot that built it.
        file: PathBuf,
        /// The config naming where the map is kept (`[map] path`).
        #[arg(long, default_value = robotd_params::DEFAULT_PATH)]
        config: PathBuf,
    },
}

pub fn run(socket: &Path, command: Option<MapCommand>) -> Result<(), Failure> {
    if let Some(MapCommand::Load { file, config }) = &command {
        return load(file, config);
    }
    let mut client = Client::connect_to("mapd", socket)?;
    match command.unwrap_or(MapCommand::Status) {
        MapCommand::Status => {
            let s: proto::MapStatusResult =
                decode(&result_of(client.call(&proto::Call::MapStatus)?)?)?;
            print!("{}", render_status(&s));
            Ok(())
        }
        MapCommand::Show { res, pgm } => {
            let g: proto::MapGridResult = decode(&result_of(client.call(
                &proto::Call::MapGrid(proto::MapGridParams { res_m: Some(res) }),
            )?)?)?;
            if g.width == 0 {
                println!(
                    "nothing mapped yet — the map grows where the robot stops and looks around"
                );
                return Ok(());
            }
            print!("{}", render_grid(&g));
            if let Some(path) = pgm {
                std::fs::write(&path, pgm_bytes(&g)).map_err(|e| {
                    Failure::new(exit::FAILED, format!("writing {}: {e}", path.display()))
                })?;
                eprintln!("wrote {}", path.display());
            }
            Ok(())
        }
        MapCommand::Load { .. } => unreachable!("handled before connecting"),
        MapCommand::Mode { mode } => {
            let want = mode == "localize";
            let r: proto::MapModeResult = decode(&result_of(client.call(
                &proto::Call::MapMode(proto::MapModeParams {
                    localize_only: want,
                }),
            )?)?)?;
            match (want, r.localize_only) {
                (true, true) => println!(
                    "localizing only: the map's {} stops are frozen, and nothing is added",
                    r.keyframes
                ),
                (true, false) => {
                    return Err(Failure::new(
                        exit::REFUSED,
                        "nothing mapped to localize in: still mapping".into(),
                    ));
                }
                (false, _) => println!("mapping: stops add to the map again"),
            }
            Ok(())
        }
        MapCommand::Wipe { yes } => {
            if !yes {
                eprint!(
                    "Forget the whole map? The robot will have to learn its home again. [y/N] "
                );
                let mut answer = String::new();
                let _ = std::io::stdin().read_line(&mut answer);
                if !answer.trim().eq_ignore_ascii_case("y") {
                    return Err(Failure::new(exit::REFUSED, "left as it was".into()));
                }
            }
            let r: proto::MapWipeResult = decode(&result_of(client.call(&proto::Call::MapWipe)?)?)?;
            println!(
                "forgot {} stops; a new map starts at the next one",
                r.keyframes
            );
            Ok(())
        }
    }
}

/// What a map file holds, as far as `robotctl` checks it before handing it to `mapd`.
#[derive(Debug, PartialEq)]
struct MapFile {
    keyframes: usize,
    /// Stops not joined to the map: kept apart, never drawn.
    island_keyframes: usize,
    /// The file, with the robot that built it's last pose taken out.
    bytes: Vec<u8>,
}

/// Check that `bytes` is a `mapd` map, and forget where the robot that built it last stood: that
/// is `mapd`'s boot hint, and it is not where this robot is.
///
/// Only the shape is checked here; `mapd` knows its own format, and sets aside a file it cannot
/// read rather than lose it.
fn prepare(bytes: &[u8]) -> Result<MapFile, String> {
    let mut v: serde_json::Value =
        serde_json::from_slice(bytes).map_err(|e| format!("not JSON: {e}"))?;
    let obj = v.as_object_mut().ok_or("not a map: not a JSON object")?;
    if !obj.get("version").is_some_and(serde_json::Value::is_u64) {
        return Err("not a map: no format version".into());
    }
    let keyframes = obj
        .get("keyframes")
        .and_then(serde_json::Value::as_array)
        .ok_or("not a map: no keyframes")?
        .len();
    if keyframes == 0 {
        return Err("the map is empty".into());
    }
    let island: Vec<u64> = obj
        .get("island")
        .and_then(serde_json::Value::as_array)
        .map(|a| a.iter().filter_map(serde_json::Value::as_u64).collect())
        .unwrap_or_default();
    // The map is the island of node 0.
    let island_keyframes = island
        .iter()
        .filter(|&&i| Some(&i) != island.first())
        .count();
    obj.insert("last_pose".into(), serde_json::Value::Null);
    let bytes = serde_json::to_vec(&v).map_err(|e| e.to_string())?;
    Ok(MapFile {
        keyframes: keyframes - island_keyframes,
        island_keyframes,
        bytes,
    })
}

fn systemctl(verb: &str) -> Result<(), Failure> {
    let ok = std::process::Command::new("systemctl")
        .args([verb, "mapd.service"])
        .status()
        .is_ok_and(|s| s.success());
    if ok {
        Ok(())
    } else {
        Err(Failure::new(
            exit::FAILED,
            format!("systemctl {verb} mapd.service failed"),
        ))
    }
}

fn load(file: &Path, config: &Path) -> Result<(), Failure> {
    // SAFETY: no arguments, no memory.
    if unsafe { libc::geteuid() } != 0 {
        return Err(Failure::new(
            exit::REFUSED,
            "loading a map stops and starts mapd: run it with sudo".into(),
        ));
    }
    let params = robotd_params::Params::load(config, false)
        .map_err(|e| Failure::new(exit::FAILED, format!("{}: {e}", config.display())))?;
    let dest = params.map.path.clone();
    let bytes = std::fs::read(file)
        .map_err(|e| Failure::new(exit::FAILED, format!("reading {}: {e}", file.display())))?;
    let map = prepare(&bytes)
        .map_err(|e| Failure::new(exit::REFUSED, format!("{}: {e}", file.display())))?;

    // Stopped first: mapd saves its map on the way out, and would write over this one.
    systemctl("stop")?;
    let installed = (|| -> std::io::Result<Option<PathBuf>> {
        let mut kept = None;
        if dest.exists() {
            let secs = crate::unix_now();
            let mut aside = dest.as_os_str().to_owned();
            aside.push(format!(".before-load-{secs}"));
            std::fs::rename(&dest, &aside)?;
            kept = Some(PathBuf::from(aside));
        }
        let mut tmp = dest.as_os_str().to_owned();
        tmp.push(".tmp");
        let tmp = PathBuf::from(tmp);
        std::fs::write(&tmp, &map.bytes)?;
        // mapd's, readable by `robot` — as mapd leaves its own saves.
        let owned = std::process::Command::new("chown")
            .arg("mapd:robot")
            .arg(&tmp)
            .status()?;
        if !owned.success() {
            return Err(std::io::Error::other("chown mapd:robot failed"));
        }
        std::fs::set_permissions(
            &tmp,
            <std::fs::Permissions as std::os::unix::fs::PermissionsExt>::from_mode(0o640),
        )?;
        std::fs::rename(&tmp, &dest)?;
        Ok(kept)
    })();
    // Started again whatever happened: a robot left without mapd is worse than either map.
    let started = systemctl("start");
    let kept = installed
        .map_err(|e| Failure::new(exit::FAILED, format!("installing {}: {e}", dest.display())))?;
    started?;

    println!(
        "installed {} stops from {} as {}",
        map.keyframes,
        file.display(),
        dest.display()
    );
    if map.island_keyframes > 0 {
        println!(
            "  and {} stops the robot that built it never placed in the map: kept, not drawn",
            map.island_keyframes
        );
    }
    if let Some(kept) = kept {
        println!("the map this robot had is kept as {}", kept.display());
    }
    if !params.map.enabled {
        println!(
            "[map] enabled is off on this robot: mapd will not use it until it is on (robotctl configure)"
        );
    }
    println!(
        "mapd restarted; it is lost until it recognises the room — a stop or two somewhere the map has seen"
    );
    Ok(())
}

fn render_status(s: &proto::MapStatusResult) -> String {
    let mut out = String::new();
    let st = &s.state;
    match (&st.pose, &st.lost) {
        (Some(p), _) => out.push_str(&format!(
            "pose        x {:+.2} m  y {:+.2} m  heading {:+.0}°\n",
            p.x,
            p.y,
            p.yaw.to_degrees()
        )),
        (None, Some(why)) => out.push_str(&format!(
            "pose        lost ({}) — {}\n",
            why,
            match why.as_str() {
                "boot" => "it recognises the map after a stop or two somewhere it has been",
                "carried" => "put down, it looks for itself at the next stops",
                "bumped" => "knocked or stumbled; it looks for itself nearby at the next stops",
                "sat" => "it stood up; it may have been carried while seated, so the next stops check where it is",
                "fell" => "it looks for itself near where it fell",
                _ => "what it sees stopped matching the map; it is searching",
            }
        )),
        (None, None) => out.push_str("pose        not yet — the map starts at the first stop\n"),
    }
    out.push_str(&format!(
        "map         {} stops{}\n",
        st.keyframes,
        if s.island_keyframes > 0 {
            format!(", {} more not yet placed in it", s.island_keyframes)
        } else {
            String::new()
        }
    ));
    if s.localize_only {
        out.push_str("mode        localizing only — the map is frozen\n");
    }
    out.push_str(&format!(
        "since start {} loop closures, {} relocalizations\n",
        s.loops, s.relocalizations
    ));
    out.push_str(&format!(
        "now         {}\n",
        if st.collecting {
            "standing still — this stop is being added"
        } else {
            "moving, or not stopped long enough to look"
        }
    ));
    out.push_str(&format!("file        {}", s.path));
    if let Some(t) = s.saved_at {
        let age = crate::unix_now() - t as i64;
        out.push_str(&format!(" (saved {} ago)", crate::describe_age(age)));
    }
    out.push('\n');
    if let Some(why) = &s.unavailable {
        out.push_str(&format!("NOT MAPPING {why}\n"));
    }
    out
}

/// The grid, north up, two characters per cell so a square cell looks square in a terminal.
fn render_grid(g: &proto::MapGridResult) -> String {
    let (w, h) = (g.width as usize, g.height as usize);
    let cells: Vec<char> = g.cells.chars().collect();
    let mut marks = vec![None; w * h];
    let mut mark = |x: f64, y: f64, c: char| {
        let i = ((x - g.origin[0]) / g.res_m).floor();
        let j = ((y - g.origin[1]) / g.res_m).floor();
        if i >= 0.0 && j >= 0.0 && (i as usize) < w && (j as usize) < h {
            marks[j as usize * w + i as usize] = Some(c);
        }
    };
    for k in &g.keyframes {
        mark(k.x, k.y, 'o');
    }
    if let Some(p) = g.pose {
        mark(p.x, p.y, '@');
    }
    let mut out = String::new();
    for j in (0..h).rev() {
        for i in 0..w {
            let k = j * w + i;
            let s = match (marks[k], cells.get(k)) {
                (Some(c), _) => format!("{c} "),
                (None, Some('#')) => "██".to_owned(),
                (None, Some('.')) => "· ".to_owned(),
                _ => "  ".to_owned(),
            };
            out.push_str(&s);
        }
        out.push('\n');
    }
    out.push_str(&format!(
        "{} × {} cells of {} m; o = a stop, @ = the robot\n",
        w, h, g.res_m
    ));
    out
}

fn pgm_bytes(g: &proto::MapGridResult) -> Vec<u8> {
    let (w, h) = (g.width as usize, g.height as usize);
    let cells = g.cells.as_bytes();
    let mut out = format!("P5\n{w} {h}\n255\n").into_bytes();
    for j in (0..h).rev() {
        for i in 0..w {
            out.push(match cells.get(j * w + i) {
                Some(b'#') => 0,
                Some(b'.') => 255,
                _ => 160,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_grid_is_drawn_north_up_with_the_robot_on_it() {
        let g = proto::MapGridResult {
            res_m: 1.0,
            origin: [0.0, 0.0],
            width: 3,
            height: 2,
            // Row 0 is the lowest y: a wall along the bottom, floor above it.
            cells: "###...".into(),
            keyframes: vec![],
            pose: Some(proto::MapPose {
                x: 1.5,
                y: 1.5,
                yaw: 0.0,
            }),
        };
        let text = render_grid(&g);
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[0], "· @ · ");
        assert_eq!(lines[1], "██████");
    }

    #[test]
    fn a_lost_robot_says_why_and_what_it_will_do() {
        let s = proto::MapStatusResult {
            state: proto::MapState {
                lost: Some("carried".into()),
                keyframes: 12,
                ..Default::default()
            },
            path: "/var/lib/mapd/map.json".into(),
            ..Default::default()
        };
        let text = render_status(&s);
        assert!(
            text.contains("lost (carried)") && text.contains("12 stops"),
            "{text}"
        );
    }
}

#[cfg(test)]
mod load_tests {
    use super::*;

    #[test]
    fn a_map_loses_its_last_pose_and_counts_its_islands() {
        let file = br#"{"version":1,"keyframes":[{},{},{},{}],"graph":{"nodes":[],"edges":[]},
            "island":[0,0,0,3],"last_pose":{"x":1.0,"y":2.0,"yaw":0.5}}"#;
        let m = prepare(file).unwrap();
        assert_eq!((m.keyframes, m.island_keyframes), (3, 1));
        let v: serde_json::Value = serde_json::from_slice(&m.bytes).unwrap();
        assert!(v["last_pose"].is_null());
        assert_eq!(v["graph"]["nodes"], serde_json::json!([]));
    }

    #[test]
    fn what_is_not_a_map_is_refused() {
        assert!(prepare(b"[]").is_err());
        assert!(prepare(br#"{"keyframes":[{}]}"#).is_err());
        assert!(prepare(br#"{"version":1,"keyframes":[]}"#).is_err());
        assert!(prepare(b"not json").is_err());
    }
}
