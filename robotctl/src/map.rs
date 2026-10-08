//! `robotctl map`: where the robot is in its map, the map drawn in the terminal, and wiping it.
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
}

pub fn run(socket: &Path, command: Option<MapCommand>) -> Result<(), Failure> {
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
