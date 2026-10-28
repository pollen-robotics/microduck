//! `robotctl robot goto tag <id>`: find a printed AprilTag with the head camera and stand on its footprints.
//!
//! The sheet (`scripts/footstep_tag_sheet.py`, one per tag id) carries a tag36h11 tag [`TAG_SIZE_M`] wide and,
//! [`STANCE_FROM_TAG_M`] in front of it, the robot's two standing footprints, toes towards the tag. Lying on the
//! floor, the tag's printed "up" points away from the footprints, so the goal is: stance centre that far back
//! along it, facing it.
//!
//! Everything here is a client of what the daemons already publish — nothing new runs on the robot:
//!
//! 1. the head sweeps the floor in front with `robot.look` (the gaze IK on the robot's own model);
//! 2. a raw frame comes from `mediad` (`media.frame`), its Y plane is the grey image AprilTag wants;
//! 3. the tag's corners are undistorted with this robot's calibration (`[media.intrinsics]`, else the
//!    family's), turned by the frame's `rotate` into the upright camera — the frame `robot.state`'s
//!    `frames.camera` describes — and the tag's pose solved from the four of them;
//! 4. `robot.state` gives camera-in-trunk, the trunk's orientation (IMU) and position (odometry): the tag in the
//!    odometry's world, and so the goal, sent as a `robot.footstepGoal` in that frame.
//!
//! AprilTag rather than OpenCV's ArUco module: the reference C detector builds from source with the board's
//! toolchain (no OpenCV on a robot), and tag36h11 is a dictionary OpenCV's ArUco reads too
//! (`DICT_APRILTAG_36h11`), so the tags are ArUco-compatible.

use std::io::{BufRead, Read};
use std::path::Path;
use std::time::{Duration, Instant};

use apriltag::{DetectorBuilder, Family, Image};
use duck_ipc_proto as proto;
use kinematics::{Pose, Quat};

use crate::{Client, Failure, exit};

/// The tag's black square, edge to edge — what the sheet prints and the pose is solved for.
pub const TAG_SIZE_M: f64 = 0.100;
/// Tag centre → stance centre (midway between the printed sole centres), along the tag's "down".
pub const STANCE_FROM_TAG_M: f64 = 0.130;

/// Where the head looks for the tag: azimuths (degrees, left positive) at two floor distances, nearest-first
/// for straight ahead. The camera's ~62° field covers the gaps between neighbours.
const SCAN: [(f64, f64); 9] = [
    (0.0, 0.55),
    (30.0, 0.55),
    (-30.0, 0.55),
    (60.0, 0.55),
    (-60.0, 0.55),
    (0.0, 1.0),
    (30.0, 1.0),
    (-30.0, 1.0),
    (0.0, 0.35),
];
/// Head travel plus a frame or two: what a gaze change takes before the image is the new one.
const SETTLE: Duration = Duration::from_millis(900);
/// Sightings averaged into the goal, from the gaze that first saw the tag.
const SIGHTINGS: usize = 3;

/// The camera, as the detector needs it: pinhole + OpenCV's `k1 k2 p1 p2 k3`, for the frame as sent.
struct Camera {
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    k: [f64; 5],
}

impl Camera {
    /// This robot's calibration (`[media.intrinsics]`), else its board's camera family's, scaled to a `width`×`height` frame.
    /// Refuses another aspect ratio: a crop is not a resize, and which part was cropped is not knowable.
    fn for_frame(config: &Path, width: u32, height: u32) -> Result<Self, String> {
        let own = robotd_params::Params::load(config, false)
            .ok()
            .and_then(|p| p.media.intrinsics);
        // Else this board's camera family, as `mediad` publishes it: an alpha's lens on a beta is ~1.5×
        // off in focal length, which would put a tag at two-thirds of its distance.
        let family = || match robotd_params::board::Board::detected()
            .unwrap_or_default()
            .camera_sensor()
        {
            robotd_params::CameraSensor::Imx219 => robotd_params::CameraIntrinsics::alpha(),
            robotd_params::CameraSensor::Gc2093 => robotd_params::CameraIntrinsics::beta(),
        };
        let cal = own.unwrap_or_else(family);
        let s = f64::from(width) / f64::from(cal.width);
        if (f64::from(height) / f64::from(cal.height) - s).abs() > 1e-3 {
            return Err(format!(
                "the camera sends {width}×{height}, the calibration is for {}×{} — another crop",
                cal.width, cal.height
            ));
        }
        let mut k = [0.0; 5];
        for (dst, src) in k.iter_mut().zip(&cal.distortion) {
            *dst = *src;
        }
        Ok(Self {
            fx: cal.fx * s,
            fy: cal.fy * s,
            cx: cal.cx * s,
            cy: cal.cy * s,
            k,
        })
    }

    /// A pixel → its undistorted normalized coordinates (x/z, y/z), by fixed-point iteration on OpenCV's model.
    fn undistort(&self, px: [f64; 2]) -> [f64; 2] {
        let [k1, k2, p1, p2, k3] = self.k;
        let (xd, yd) = ((px[0] - self.cx) / self.fx, (px[1] - self.cy) / self.fy);
        let (mut x, mut y) = (xd, yd);
        for _ in 0..20 {
            let r2 = x * x + y * y;
            let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
            let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            x = (xd - dx) / radial;
            y = (yd - dy) / radial;
        }
        [x, y]
    }
}

/// Normalized coordinates in the frame as sent → in the upright camera (the image turned `rotate`° clockwise):
/// a ray (X, Y, Z) of the sent frame is (−Y, X, Z) upright for 90°, (−X, −Y, Z) for 180°, (Y, −X, Z) for 270°
/// (duckslam's `rotate_intrinsics`, which overlays `frames.camera` on upright images).
fn upright(n: [f64; 2], rotate: u32) -> Result<[f64; 2], String> {
    Ok(match rotate % 360 {
        0 => n,
        90 => [-n[1], n[0]],
        180 => [-n[0], -n[1]],
        270 => [n[1], -n[0]],
        other => return Err(format!("camera mount rotated {other}°, not a quarter turn")),
    })
}

/// The tag in the camera (cv2 axes), from its four corners in normalized upright coordinates, in the
/// detector's order: the printed tag's top-right, top-left, bottom-left, bottom-right. Tag frame: x right,
/// y up as printed, z out of the paper; origin at the centre.
///
/// The planar homography from the tag plane to the image, decomposed: `H ~ [r1 r2 t]`, scale from the two
/// rotation columns, the rotation re-orthonormalised. Centimetre-good at a metre, which is what a goal needs.
fn tag_pose(corners: [[f64; 2]; 4], size: f64) -> Option<Pose> {
    let h = size / 2.0;
    let object = [[h, h], [-h, h], [-h, -h], [h, -h]];
    // DLT, h33 = 1: two rows per correspondence, 8 unknowns.
    let mut a = [[0.0f64; 9]; 8];
    for (i, (o, n)) in object.iter().zip(&corners).enumerate() {
        let (x, y, u, v) = (o[0], o[1], n[0], n[1]);
        a[2 * i] = [x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y, u];
        a[2 * i + 1] = [0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y, v];
    }
    let hv = solve8(a)?;
    let col = |c: usize| [hv[c], hv[3 + c], if c == 2 { 1.0 } else { hv[6 + c] }];
    let (h1, h2, h3) = (col(0), col(1), col(2));
    let norm = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let mut lambda = 2.0 / (norm(h1) + norm(h2));
    if h3[2] * lambda < 0.0 {
        lambda = -lambda; // the tag is in front of the camera
    }
    let scale = |v: [f64; 3], s: f64| [v[0] * s, v[1] * s, v[2] * s];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let r1 = scale(h1, lambda);
    let r1 = scale(r1, 1.0 / norm(r1));
    let r2 = scale(h2, lambda);
    let d = dot(r1, r2);
    let r2 = [r2[0] - d * r1[0], r2[1] - d * r1[1], r2[2] - d * r1[2]];
    let r2 = scale(r2, 1.0 / norm(r2));
    let r3 = cross(r1, r2);
    let t = scale(h3, lambda);
    Some(Pose::new(t, quat_from_columns(r1, r2, r3)))
}

/// Gaussian elimination with partial pivoting on an 8×9 augmented system.
fn solve8(mut a: [[f64; 9]; 8]) -> Option<[f64; 8]> {
    for col in 0..8 {
        let pivot = (col..8).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-12 {
            return None;
        }
        a.swap(col, pivot);
        let pivot_row = a[col];
        for (row, r) in a.iter_mut().enumerate() {
            if row != col {
                let f = r[col] / pivot_row[col];
                for (v, p) in r.iter_mut().zip(pivot_row).skip(col) {
                    *v -= f * p;
                }
            }
        }
    }
    Some(std::array::from_fn(|i| a[i][8] / a[i][i]))
}

/// The rotation whose columns are the frame's axes, as a quaternion.
fn quat_from_columns(x: [f64; 3], y: [f64; 3], z: [f64; 3]) -> Quat {
    let (m00, m11, m22) = (x[0], y[1], z[2]);
    let trace = m00 + m11 + m22;
    let (w, qx, qy, qz) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (
            0.25 * s,
            (y[2] - z[1]) / s,
            (z[0] - x[2]) / s,
            (x[1] - y[0]) / s,
        )
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        (
            (y[2] - z[1]) / s,
            0.25 * s,
            (y[0] + x[1]) / s,
            (z[0] + x[2]) / s,
        )
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        (
            (z[0] - x[2]) / s,
            (y[0] + x[1]) / s,
            0.25 * s,
            (z[1] + y[2]) / s,
        )
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        (
            (x[1] - y[0]) / s,
            (z[0] + x[2]) / s,
            (z[1] + y[2]) / s,
            0.25 * s,
        )
    };
    Quat::new(w, qx, qy, qz).normalized()
}

/// Where to stand for a tag at `tag` (world frame): the stance centre [`STANCE_FROM_TAG_M`] back along the
/// tag's printed "up", facing it. `None` when the tag is not lying flat (its face more than ~35° off up): a tag
/// on a wall is not a sheet on the floor, and its "up" says nothing about where its footprints are.
fn stance_for(tag: Pose) -> Option<[f64; 3]> {
    if tag.quat.rotate([0.0, 0.0, 1.0])[2] < 0.82 {
        return None;
    }
    let up = tag.quat.rotate([0.0, 1.0, 0.0]);
    let yaw = up[1].atan2(up[0]);
    Some([
        tag.pos[0] - STANCE_FROM_TAG_M * yaw.cos(),
        tag.pos[1] - STANCE_FROM_TAG_M * yaw.sin(),
        yaw,
    ])
}

/// One raw frame from `mediad`: the header, then exactly `bytes` of UYVY.
fn grab_frame() -> Result<(proto::MediaFrameHeader, Vec<u8>), Failure> {
    let mut media = Client::connect_to("mediad", Path::new(proto::socket::MEDIA))?;
    media.hello()?;
    media.send(&proto::Request {
        jsonrpc: "2.0".into(),
        id: Some(proto::Id::Number(9)),
        method: proto::method::MEDIA_FRAME.into(),
        params: None,
    })?;
    let mut line = String::new();
    media
        .reader
        .read_line(&mut line)
        .map_err(|e| Failure::new(exit::FAILED, format!("camera: {e}")))?;
    let response: proto::Response = serde_json::from_str(&line)
        .map_err(|e| Failure::new(exit::FAILED, format!("camera: {e}")))?;
    if let Some(error) = response.error {
        return Err(Failure::new(
            exit::FAILED,
            format!("camera: {}", error.message),
        ));
    }
    let header: proto::MediaFrameHeader = response
        .result
        .and_then(|r| serde_json::from_value(r).ok())
        .ok_or_else(|| Failure::new(exit::FAILED, "camera: no frame header".into()))?;
    if !header.valid_uyvy() {
        return Err(Failure::new(
            exit::FAILED,
            "camera: not a UYVY frame".into(),
        ));
    }
    let mut data = vec![0; header.bytes];
    media
        .reader
        .read_exact(&mut data)
        .map_err(|e| Failure::new(exit::FAILED, format!("camera: {e}")))?;
    Ok((header, data))
}

/// The tag's four corners (pixels of the frame as sent), if tag `id` is in the frame.
fn find_tag(header: &proto::MediaFrameHeader, uyvy: &[u8], id: u32) -> Option<[[f64; 2]; 4]> {
    let (w, h) = (header.width as usize, header.height as usize);
    let mut image = Image::zeros_with_stride(w, h, w).ok()?;
    for y in 0..h {
        for x in 0..w {
            // UYVY: U Y0 V Y1 — luma at every odd byte.
            image[(x, y)] = uyvy[(y * w + x) * 2 + 1];
        }
    }
    let mut detector = DetectorBuilder::new()
        .add_family_bits(Family::tag_36h11(), 1)
        .build()
        .ok()?;
    // Full resolution: a few frames per scan, and the beta's ~85° lens makes a 100 mm tag ~70 px at 1 m —
    // halved, too close to the detector's limit.
    detector.set_decimation(1.0);
    detector
        .detect(&image)
        .into_iter()
        .find(|d| d.id() == id as usize && d.decision_margin() > 30.0)
        .map(|d| {
            let c = d.corners();
            [c[0], c[1], c[2], c[3]]
        })
}

/// The latest `robot.state`, a few frames into a fresh subscription so it is current.
fn robot_state(robot_socket: &Path) -> Result<proto::RobotState, Failure> {
    let mut stream = Client::connect_to("robotd", robot_socket)?;
    stream.hello()?;
    stream.send(&proto::Request::call(
        proto::Id::Number(1),
        &proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(30) }),
    ))?;
    let mut latest = None;
    let mut seen = 0;
    let mut line = String::new();
    while seen < 3 {
        line.clear();
        if stream.reader.read_line(&mut line).unwrap_or(0) == 0 {
            break;
        }
        if let Some(state) = serde_json::from_str::<proto::Request>(&line)
            .ok()
            .and_then(|r| r.as_state())
        {
            latest = Some(state);
            seen += 1;
        }
    }
    latest.ok_or_else(|| Failure::new(exit::UNREACHABLE, "robotd sent no state".into()))
}

/// The tag in the odometry's world, from one frame and the state read right after it (the robot stands still
/// with its head settled, so the two describe the same instant to within a tick).
fn locate(config: &Path, robot_socket: &Path, id: u32) -> Result<Option<Pose>, Failure> {
    let (header, data) = grab_frame()?;
    let Some(corners) = find_tag(&header, &data, id) else {
        return Ok(None);
    };
    let camera = Camera::for_frame(config, header.width, header.height)
        .map_err(|e| Failure::new(exit::FAILED, e))?;
    let mut normalized = [[0.0; 2]; 4];
    for (n, c) in normalized.iter_mut().zip(corners) {
        *n = upright(camera.undistort(c), header.rotate)
            .map_err(|e| Failure::new(exit::FAILED, e))?;
    }
    let Some(in_camera) = tag_pose(normalized, TAG_SIZE_M) else {
        return Ok(None);
    };
    let state = robot_state(robot_socket)?;
    let (Some(frames), Some(imu)) = (state.frames, state.imu) else {
        return Err(Failure::new(
            exit::FAILED,
            "robot.state has no camera pose or IMU".into(),
        ));
    };
    let pose = |p: proto::PoseState| {
        Pose::new(p.pos, Quat::new(p.quat[0], p.quat[1], p.quat[2], p.quat[3]))
    };
    let [w, x, y, z] = imu.quat;
    let trunk = Pose::new(state.odom.position, Quat::new(w, x, y, z).normalized());
    Ok(Some(trunk * pose(frames.camera) * in_camera))
}

/// The staging pose: this far behind the footprints, facing the tag — where the last sighting is taken. The
/// tag is then ~35 cm away and straight ahead (on a beta that measured 6 mm in range, ~3 cm across), while on the
/// footprints themselves it is under the head, out of the gaze the policy allows (seen: the loop lost it there).
const STAGE_M: f64 = 0.22;
/// At the staging pose when a fresh sighting puts it this close to the feet; then the last stretch is walked
/// blind, straight ahead.
const STAGED_M: f64 = 0.03;
const STAGED_RAD: f64 = 0.09;
/// Longest leg walked on one sighting. Farther goals are approached in legs, re-seeing the tag after each: the
/// estimate improves as the robot closes in, and FootstepNet was trained on goals within ±1 m.
const LEG_M: f64 = 0.8;
/// Sighting → walk → re-sighting rounds before reporting where it got to.
const ROUNDS: usize = 5;

/// Where the robot stands now: the midpoint of its sole centres (7 mm ahead of the foot sites) and their mean
/// heading, in the odometry's world — forward kinematics on the measured joints, oriented by the IMU, placed by
/// the odometry, as `robotd`'s own odometry does.
fn stance_now(state: &proto::RobotState) -> Option<[f64; 3]> {
    stance_from(&state.joints, state.odom.position, state.imu.as_ref()?.quat)
}

/// [`stance_now`] from its inputs: joints in `JOINT_NAMES` order, trunk position, IMU `[w, x, y, z]`.
fn stance_from(joints: &[f64], position: [f64; 3], imu: [f64; 4]) -> Option<[f64; 3]> {
    let model = kinematics::Model::alpha();
    let mut angles = vec![0.0; model.num_joints()];
    for (name, angle) in proto::JOINT_NAMES.iter().zip(joints) {
        if let Some(i) = model.joint_index(name) {
            angles[i] = *angle;
        }
    }
    let [w, x, y, z] = imu;
    let trunk = Pose::new(position, Quat::new(w, x, y, z).normalized());
    let mut soles = Vec::new();
    for site in ["left_foot", "right_foot"] {
        let foot = trunk * model.site_pose(model.site(site)?, &angles);
        let centre = foot.transform_point([0.007, 0.0, 0.0]);
        soles.push([centre[0], centre[1], foot.quat.yaw()]);
    }
    let yaw = (soles[0][2].sin() + soles[1][2].sin()).atan2(soles[0][2].cos() + soles[1][2].cos());
    Some([
        (soles[0][0] + soles[1][0]) / 2.0,
        (soles[0][1] + soles[1][1]) / 2.0,
        yaw,
    ])
}

fn wrap(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Hold the gaze on a trunk-frame point while it settles, re-sent so the idle head never takes over.
fn hold_look(client: &mut Client, target: [f64; 3]) -> Result<(), Failure> {
    let start = Instant::now();
    while start.elapsed() < SETTLE {
        client.call(&proto::Call::RobotLook(proto::LookParams {
            x: target[0],
            y: target[1],
            z: target[2],
            neck_pitch: 0.0,
        }))?;
        std::thread::sleep(Duration::from_millis(150));
    }
    Ok(())
}

/// A sighting: the tag's world position and the stance its sheet implies.
type Sighting = ([f64; 3], [f64; 3]);

/// Up to [`SIGHTINGS`] sightings of the tag from the current gaze, averaged: the tag's world pose (position and
/// the stance it implies). `None` if it is not in view.
fn sight(config: &Path, robot_socket: &Path, id: u32) -> Result<Option<Sighting>, Failure> {
    let mut found: Vec<Sighting> = Vec::new();
    for _ in 0..SIGHTINGS + 2 {
        if let Some(tag) = locate(config, robot_socket, id)? {
            match stance_for(tag) {
                Some(stance) => found.push((tag.pos, stance)),
                None => eprintln!("  tag {id} seen, but not lying flat — ignored"),
            }
        }
        if found.len() >= SIGHTINGS {
            break;
        }
    }
    if found.is_empty() {
        return Ok(None);
    }
    let n = found.len() as f64;
    let mean = |f: fn(&Sighting) -> f64| found.iter().map(f).sum::<f64>() / n;
    let (s, c) = found.iter().fold((0.0, 0.0), |(s, c), (_, p)| {
        (s + p[2].sin(), c + p[2].cos())
    });
    Ok(Some((
        [mean(|f| f.0[0]), mean(|f| f.0[1]), mean(|f| f.0[2])],
        [mean(|f| f.1[0]), mean(|f| f.1[1]), s.atan2(c)],
    )))
}

/// A world point in the trunk frame of `state`, for the gaze IK.
fn in_trunk(state: &proto::RobotState, world: [f64; 3]) -> Option<[f64; 3]> {
    Some(point_in_trunk(
        state.odom.position,
        state.imu.as_ref()?.quat,
        world,
    ))
}

fn point_in_trunk(position: [f64; 3], imu: [f64; 4], world: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = imu;
    let q = Quat::new(w, -x, -y, -z).normalized();
    q.rotate([
        world[0] - position[0],
        world[1] - position[1],
        world[2] - position[2],
    ])
}

/// Send one footstep goal (odometry world) and wait until the footstep policy has walked it and handed back.
fn walk_to(robot: &mut Client, robot_socket: &Path, goal: [f64; 3]) -> Result<(), Failure> {
    let result = crate::result_of(robot.call(&proto::Call::RobotFootstepGoal(
        proto::FootstepGoalParams {
            x: goal[0],
            y: goal[1],
            yaw: goal[2],
            foot: None,
            odom: true,
        },
    ))?)?;
    let outcome: proto::IntentResult = crate::decode(&result)?;
    if !outcome.accepted {
        return Err(Failure::new(
            exit::REFUSED,
            outcome.reason.unwrap_or_else(|| "the robot refused".into()),
        ));
    }
    let start = Instant::now();
    let mut walking = false;
    let mut quiet = 0;
    while start.elapsed() < Duration::from_secs(60) {
        std::thread::sleep(Duration::from_millis(250));
        let state = robot_state(robot_socket)?;
        if state.movement.requested.iter().any(|v| v.abs() > 0.05) {
            return Err(Failure::new(
                exit::FAILED,
                "cancelled: the stick moved".into(),
            ));
        }
        if state.policy == "footsteps" {
            walking = true;
            quiet = 0;
        } else if walking || start.elapsed() > Duration::from_secs(3) {
            quiet += 1;
            if quiet >= 3 {
                return Ok(());
            }
        }
    }
    Err(Failure::new(
        exit::FAILED,
        "still walking after 60 s".into(),
    ))
}

/// Look for tag `id` without moving: where the vision chain puts the tag and its footprints, relative to the
/// robot's current stance (x forward, y left, heading) — to compare with a tape measure, which separates a camera
/// error from a walking one.
pub fn look_for_tag(
    config: &Path,
    robot_socket: &Path,
    id: u32,
) -> Result<([f64; 3], [f64; 3]), Failure> {
    let mut robot = Client::connect_to("robotd", robot_socket)?;
    robot.hello()?;
    let trunk_height = robot_state(robot_socket)?.odom.position[2].max(0.08);
    let mut seen = None;
    for (azimuth, distance) in SCAN {
        eprintln!("looking {azimuth:+.0}° at {distance:.2} m…");
        let a = azimuth.to_radians();
        hold_look(
            &mut robot,
            [distance * a.cos(), distance * a.sin(), -trunk_height],
        )?;
        seen = sight(config, robot_socket, id)?;
        if seen.is_some() {
            break;
        }
    }
    robot.call(&proto::Call::RobotHead(proto::HeadParams::default()))?;
    let (tag, goal) =
        seen.ok_or_else(|| Failure::new(exit::FAILED, format!("tag {id} not seen")))?;
    let here = stance_now(&robot_state(robot_socket)?)
        .ok_or_else(|| Failure::new(exit::FAILED, "robot.state has no IMU or joints".into()))?;
    let rel = |p: [f64; 2], yaw: f64| {
        let (s, c) = here[2].sin_cos();
        let (dx, dy) = (p[0] - here[0], p[1] - here[1]);
        [c * dx + s * dy, -s * dx + c * dy, wrap(yaw - here[2])]
    };
    let tag_up = goal[2]; // the stance faces the tag's printed "up"
    Ok((
        rel([tag[0], tag[1]], tag_up),
        rel([goal[0], goal[1]], goal[2]),
    ))
}

/// Find tag `id` and stand on its footprints, closing the loop on the camera: sight it, walk (at most [`LEG_M`]) to
/// the staging pose [`STAGE_M`] behind its footprints, look straight at where it should be, sight it again —
/// until a sighting puts the staging pose within [`STAGED_M`] / [`STAGED_RAD`] of the feet — then walk the last
/// stretch straight onto the footprints from that sighting. Seen on a beta: one far sighting carried every error
/// into the stance (10–15 cm off), and aiming the loop at the footprints themselves lost the tag under the head.
/// Returns the staging miss of the sighting the last stretch was walked on (m, rad).
pub fn goto_tag(config: &Path, robot_socket: &Path, id: u32) -> Result<(f64, f64), Failure> {
    let mut robot = Client::connect_to("robotd", robot_socket)?;
    robot.hello()?;
    let trunk_height = robot_state(robot_socket)?.odom.position[2].max(0.08);
    let mut tag_world: Option<[f64; 3]> = None;
    let mut miss = (f64::INFINITY, f64::INFINITY);
    for round in 0..ROUNDS {
        // Look where the tag was last seen; else (first round, or lost) sweep the floor in front.
        let mut seen = None;
        if let Some(world) = tag_world {
            let state = robot_state(robot_socket)?;
            if let Some(target) = in_trunk(&state, world) {
                eprintln!("round {}: looking at the tag…", round + 1);
                hold_look(&mut robot, target)?;
                seen = sight(config, robot_socket, id)?;
            }
        }
        if seen.is_none() {
            for (azimuth, distance) in SCAN {
                eprintln!(
                    "round {}: looking {azimuth:+.0}° at {distance:.2} m…",
                    round + 1
                );
                let a = azimuth.to_radians();
                hold_look(
                    &mut robot,
                    [distance * a.cos(), distance * a.sin(), -trunk_height],
                )?;
                seen = sight(config, robot_socket, id)?;
                if seen.is_some() {
                    break;
                }
            }
        }
        robot.call(&proto::Call::RobotHead(proto::HeadParams::default()))?;
        let Some((tag, goal)) = seen else {
            return Err(Failure::new(
                exit::FAILED,
                format!("tag {id} not seen anywhere in front of the robot"),
            ));
        };
        tag_world = Some(tag);
        let here = stance_now(&robot_state(robot_socket)?)
            .ok_or_else(|| Failure::new(exit::FAILED, "robot.state has no IMU or joints".into()))?;
        let stage = [
            goal[0] - STAGE_M * goal[2].cos(),
            goal[1] - STAGE_M * goal[2].sin(),
            goal[2],
        ];
        let (dx, dy) = (stage[0] - here[0], stage[1] - here[1]);
        let distance = dx.hypot(dy);
        miss = (distance, wrap(stage[2] - here[2]).abs());
        eprintln!(
            "  staging pose {:.0} mm, {:.1}° from the feet",
            miss.0 * 1000.0,
            miss.1.to_degrees()
        );
        if miss.0 < STAGED_M && miss.1 < STAGED_RAD {
            eprintln!("  last {:.2} m onto the footprints…", STAGE_M);
            walk_to(&mut robot, robot_socket, goal)?;
            return Ok(miss);
        }
        // Far: a leg toward the footprints, facing the tag so the next sighting has it ahead.
        let target = if distance > LEG_M {
            let k = LEG_M / distance;
            [
                here[0] + dx * k,
                here[1] + dy * k,
                (tag[1] - here[1]).atan2(tag[0] - here[0]),
            ]
        } else {
            stage
        };
        eprintln!(
            "  walking {:.2} m…",
            (target[0] - here[0]).hypot(target[1] - here[1])
        );
        walk_to(&mut robot, robot_socket, target)?;
    }
    Ok(miss)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tag lying flat 0.6 m ahead of a level camera looking down 40°, projected exactly, comes back where it
    /// is — position to a millimetre, "up" along +x — and its stance is in front of it, facing it. Pins the
    /// corner order (detector: top-right, top-left, bottom-left, bottom-right) and the homography signs.
    #[test]
    fn a_projected_tag_comes_back_where_it_is() {
        // Camera in cv2 axes, 0.2 m up, looking along world +x and pitched 40° down.
        let pitch = 40f64.to_radians();
        let (s, c) = pitch.sin_cos();
        // Columns: camera x (right) = world −y; camera y (down); camera z (forward, down-tilted).
        let cam_z = [c, 0.0, -s];
        let cam_x = [0.0, -1.0, 0.0];
        let cam_y = [
            cam_z[1] * cam_x[2] - cam_z[2] * cam_x[1],
            cam_z[2] * cam_x[0] - cam_z[0] * cam_x[2],
            cam_z[0] * cam_x[1] - cam_z[1] * cam_x[0],
        ];
        let camera = Pose::new([0.0, 0.0, 0.2], quat_from_columns(cam_x, cam_y, cam_z));
        // Tag flat on the floor at (0.6, 0.05), printed "up" along world +x: tag x = world −y, y = +x, z = up.
        let tag = Pose::new(
            [0.6, 0.05, 0.0],
            quat_from_columns([0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        );
        let h = TAG_SIZE_M / 2.0;
        let inv = |p: Pose| {
            let q = Quat::new(p.quat.w, -p.quat.x, -p.quat.y, -p.quat.z);
            let t = q.rotate(p.pos);
            Pose::new([-t[0], -t[1], -t[2]], q)
        };
        let to_cam = inv(camera) * tag;
        let corners = [[h, h], [-h, h], [-h, -h], [h, -h]].map(|o| {
            let p = to_cam.transform_point([o[0], o[1], 0.0]);
            [p[0] / p[2], p[1] / p[2]]
        });
        let found = camera * tag_pose(corners, TAG_SIZE_M).expect("solvable");
        for i in 0..3 {
            assert!(
                (found.pos[i] - tag.pos[i]).abs() < 1e-6,
                "{:?} vs {:?}",
                found.pos,
                tag.pos
            );
        }
        let stance = stance_for(found).expect("flat");
        assert!(
            (stance[0] - (0.6 - STANCE_FROM_TAG_M)).abs() < 1e-6 && (stance[1] - 0.05).abs() < 1e-6,
            "{stance:?}"
        );
        assert!(stance[2].abs() < 1e-6, "faces the tag: {stance:?}");
    }

    fn yaw_quat(yaw: f64) -> [f64; 4] {
        [(yaw / 2.0).cos(), 0.0, 0.0, (yaw / 2.0).sin()]
    }

    /// The stance is under the trunk and heads where it heads: the convergence test compares it with the
    /// footprints, so a frame slip here is a robot that "arrives" beside the sheet.
    #[test]
    fn stance_now_follows_the_trunk() {
        for yaw in [0.0, 1.2, -2.5] {
            let joints = vec![0.0; proto::JOINT_NAMES.len()];
            let s = stance_from(&joints, [0.4, -0.3, 0.17], yaw_quat(yaw)).expect("stance");
            assert!(wrap(s[2] - yaw).abs() < 1e-9, "{s:?} vs yaw {yaw}");
            assert!(
                (s[0] - 0.4).hypot(s[1] + 0.3) < 0.03,
                "stance {s:?} not under the trunk"
            );
        }
    }

    /// The gaze target is the world point seen from the trunk: a point straight ahead of a yawed robot is
    /// straight ahead in its trunk frame.
    #[test]
    fn in_trunk_undoes_the_trunk_pose() {
        let yaw: f64 = 0.9;
        let ahead = [1.0 + 0.5 * yaw.cos(), 2.0 + 0.5 * yaw.sin(), 0.0];
        let t = point_in_trunk([1.0, 2.0, 0.17], yaw_quat(yaw), ahead);
        assert!(
            (t[0] - 0.5).abs() < 1e-9 && t[1].abs() < 1e-9 && (t[2] + 0.17).abs() < 1e-9,
            "{t:?}"
        );
    }

    /// A quarter-turn mount maps rays the way duckslam's `rotate_intrinsics` does, and undoing it with the
    /// opposite turn gets the ray back.
    #[test]
    fn upright_turns_rays_like_the_image() {
        let n = [0.2, -0.1];
        assert_eq!(upright(n, 90).unwrap(), [0.1, 0.2]);
        assert_eq!(upright(upright(n, 90).unwrap(), 270).unwrap(), n);
        assert_eq!(upright(n, 180).unwrap(), [-0.2, 0.1]);
        assert!(upright(n, 45).is_err());
    }

    /// Undistortion inverts OpenCV's distortion model on the family calibration, out to the frame's corner.
    #[test]
    fn undistort_inverts_the_distortion_model() {
        let cal = robotd_params::CameraIntrinsics::alpha();
        let cam = Camera {
            fx: cal.fx,
            fy: cal.fy,
            cx: cal.cx,
            cy: cal.cy,
            k: [
                cal.distortion[0],
                cal.distortion[1],
                cal.distortion[2],
                cal.distortion[3],
                cal.distortion[4],
            ],
        };
        for (x, y) in [(0.0, 0.0), (0.3, -0.2), (-0.5, 0.35)] {
            let [k1, k2, p1, p2, k3] = cam.k;
            let r2 = x * x + y * y;
            let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
            let xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            let yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            let px = [cam.fx * xd + cam.cx, cam.fy * yd + cam.cy];
            let back = cam.undistort(px);
            assert!(
                (back[0] - x).abs() < 1e-6 && (back[1] - y).abs() < 1e-6,
                "{back:?} vs ({x}, {y})"
            );
        }
    }
}
