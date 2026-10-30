use kinematics::tof::{COLS, Posture, ROWS, Reprojector, Zone};

const CENTRE: usize = 3 * COLS + 3;

fn one_return(r: f64) -> [Option<f64>; ROWS * COLS] {
    let mut ranges = [None; ROWS * COLS];
    ranges[CENTRE] = Some(r);
    ranges
}

// Distance to the line through the origin parallel to gravity: |p × g| / |g|.
// This oracle does not use the production levelling rotation, so a shared
// quaternion/sign mistake cannot make both the implementation and test agree.
fn distance_to_axis(p: [f64; 3], g: [f64; 3]) -> f64 {
    let cross = [
        p[1] * g[2] - p[2] * g[1],
        p[2] * g[0] - p[0] * g[2],
        p[0] * g[1] - p[1] * g[0],
    ];
    cross[0].hypot(cross[1]).hypot(cross[2]) / g[0].hypot(g[1]).hypot(g[2])
}

#[test]
fn hit_range_is_distance_to_gravity_axis_across_grid_and_head_poses() {
    let rp = Reprojector::alpha();
    for head in [
        [0.0; 4],
        [0.3, 0.2, 0.0, 0.0],
        [0.1, -0.2, 0.8, 0.3],
        [-0.1, 0.2, -1.2, -0.3],
    ] {
        for gravity in [
            [0.0, 0.0, -1.0],
            [0.3, -0.4, -0.8],
            [0.6, -0.8, -1.6], // gravity magnitude must not affect the distance
            [0.0, 0.0, 1.0],   // the antiparallel levelling branch
        ] {
            let posture = Posture {
                gravity,
                // Isolate range geometry from the floor filter, including
                // downward beams; floor classification has its own tests.
                trunk_height_m: Some(10.0),
            };
            for slant in [0.3, 1.0, 2.5] {
                for (i, zone) in rp
                    .project(&[Some(slant); ROWS * COLS], head, &posture)
                    .into_iter()
                    .enumerate()
                {
                    let Zone::Hit { point, range } = zone else {
                        panic!("expected hit: {zone:?}, head {head:?}, zone {i}");
                    };
                    let expected = distance_to_axis(point, gravity);
                    assert!(
                        (range - expected).abs() < 1e-9,
                        "range {range}, axis distance {expected}, head {head:?}, gravity {gravity:?}, slant {slant}, zone {i}"
                    );
                }
            }
        }
    }
}

#[test]
fn noise_threshold_stays_sensor_relative_even_when_trunk_range_is_larger() {
    let rp = Reprojector::alpha();
    for gravity in [[0.0, 0.0, -1.0], [0.3, -0.4, -0.8]] {
        let posture = Posture {
            gravity,
            trunk_height_m: Some(10.0),
        };
        let head = [0.0; 4];
        let sensor = rp.sensor_in_trunk(head);
        let dir = sensor.quat.rotate(rp.beams()[CENTRE]);
        let horizontal_per_metre = distance_to_axis(dir, gravity);
        for delta in [-1e-6, 1e-6] {
            let slant = (Reprojector::MIN_RANGE_M + delta) / horizontal_per_metre;
            let point = sensor.transform_point(rp.beams()[CENTRE].map(|v| v * slant));
            assert!(distance_to_axis(point, gravity) > Reprojector::MIN_RANGE_M + 0.05);
            let zones = rp.project(&one_return(slant), head, &posture);
            if delta < 0.0 {
                assert_eq!(zones[CENTRE], Zone::TooClose);
            } else {
                assert!(matches!(zones[CENTRE], Zone::Hit { .. }));
            }
            assert_eq!(zones[0], Zone::Empty);
        }
    }
}

#[test]
fn untrusted_gravity_keeps_the_upright_range_fallback() {
    let rp = Reprojector::alpha();
    let ranges = one_return(0.3);
    let upright = rp.project(&ranges, [0.0; 4], &Posture::default());
    for gravity in [[0.0; 3], [0.1, -0.1, -0.1]] {
        let posture = Posture {
            gravity,
            ..Posture::default()
        };
        assert_eq!(rp.project(&ranges, [0.0; 4], &posture), upright);
    }
}
