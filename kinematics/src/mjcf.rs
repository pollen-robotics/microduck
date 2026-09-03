//! Minimal MJCF reader: the kinematic tree and nothing else.
//!
//! Bodies, hinge joints and sites are what FK needs; geoms, inertials,
//! actuators, sensors, assets and defaults are ignored. The tree is rooted at
//! `<body name="trunk_base">` pinned to identity, so every pose this crate
//! produces is in the trunk frame — world placement is the estimator's job,
//! not the model's. Quaternions are MJCF order, `[w, x, y, z]`.

use thiserror::Error;

use crate::math::{Pose, Quat};

#[derive(Debug, Error)]
pub enum ParseError {
    #[error("xml parse error: {0}")]
    Xml(#[from] roxmltree::Error),
    #[error("missing <worldbody> in MJCF")]
    NoWorldbody,
    #[error("missing <body name=\"trunk_base\"> in MJCF")]
    NoTrunkBase,
    #[error("body {0} has no name attribute")]
    UnnamedBody(String),
    #[error("failed to parse float in attribute {attr:?} on <{tag}>: {value:?}")]
    BadFloat {
        tag: String,
        attr: String,
        value: String,
    },
    #[error("attribute {attr:?} on <{tag}> must have {expected} components, got {got}")]
    BadVector {
        tag: String,
        attr: String,
        expected: usize,
        got: usize,
    },
}

pub(crate) struct Body {
    /// Index into `Tree::bodies`. `None` only for the root (trunk_base).
    pub parent: Option<usize>,
    /// Rest pose of this body in its parent's frame (identity for the root).
    pub rest: Pose,
    /// Hinge joint connecting this body to its parent. `None` = welded.
    pub joint: Option<Joint>,
}

pub(crate) struct Joint {
    pub name: String,
    /// Unit rotation axis in the body frame. MJCF's default is +z.
    pub axis: [f64; 3],
    /// `[lo, hi]` travel limits, radians. `None` when the MJCF declares none.
    pub range: Option<(f64, f64)>,
}

pub(crate) struct Site {
    pub name: String,
    pub body: usize,
    pub rest: Pose,
}

pub(crate) struct Tree {
    /// In tree order: a body's parent always precedes it.
    pub bodies: Vec<Body>,
    pub sites: Vec<Site>,
    /// Where the scene drops `trunk_base` into the world. FK ignores it (the
    /// trunk frame is the root), but its Z is the trunk's standing height above
    /// the floor — which a floor filter wants, from the asset, not a constant.
    pub trunk_pos: [f64; 3],
}

pub(crate) fn parse(xml: &str) -> Result<Tree, ParseError> {
    let doc = roxmltree::Document::parse(xml)?;
    let worldbody = doc
        .root_element()
        .children()
        .find(|n| n.is_element() && n.tag_name().name() == "worldbody")
        .ok_or(ParseError::NoWorldbody)?;

    let trunk = worldbody
        .children()
        .find(|n| {
            n.is_element()
                && n.tag_name().name() == "body"
                && n.attribute("name") == Some("trunk_base")
        })
        .ok_or(ParseError::NoTrunkBase)?;

    let mut tree = Tree {
        bodies: Vec::new(),
        sites: Vec::new(),
        trunk_pos: parse_vec3(trunk, "pos")?.unwrap_or([0.0; 3]),
    };

    // The root is anchored at identity — its MJCF `pos` is where MuJoCo drops
    // the robot into the world, which trunk-frame FK must not inherit.
    tree.bodies.push(Body {
        parent: None,
        rest: Pose::IDENTITY,
        joint: None,
    });
    collect_sites(trunk, 0, &mut tree.sites)?;

    for child in element_children(trunk) {
        if child.tag_name().name() == "body" {
            walk_body(child, 0, &mut tree)?;
        }
    }

    Ok(tree)
}

fn walk_body(node: roxmltree::Node, parent: usize, tree: &mut Tree) -> Result<(), ParseError> {
    if node.attribute("name").is_none() {
        return Err(ParseError::UnnamedBody(format!(
            "child of body index {parent}"
        )));
    }

    let joint = node
        .children()
        .find(|n| n.is_element() && n.tag_name().name() == "joint")
        .map(|j| -> Result<Joint, ParseError> {
            Ok(Joint {
                name: j.attribute("name").unwrap_or("").to_owned(),
                axis: normalize(parse_vec3(j, "axis")?.unwrap_or([0.0, 0.0, 1.0])),
                range: parse_floats_attr(j, "range", 2)?.map(|v| (v[0], v[1])),
            })
        })
        .transpose()?;

    let idx = tree.bodies.len();
    tree.bodies.push(Body {
        parent: Some(parent),
        rest: rest_pose(node)?,
        joint,
    });
    collect_sites(node, idx, &mut tree.sites)?;

    for child in element_children(node) {
        if child.tag_name().name() == "body" {
            walk_body(child, idx, tree)?;
        }
    }
    Ok(())
}

fn collect_sites(
    node: roxmltree::Node,
    body: usize,
    sites: &mut Vec<Site>,
) -> Result<(), ParseError> {
    for child in element_children(node) {
        if child.tag_name().name() == "site" {
            // A nameless site cannot be asked for, so there is nothing to keep.
            let Some(name) = child.attribute("name") else {
                continue;
            };
            sites.push(Site {
                name: name.to_owned(),
                body,
                rest: rest_pose(child)?,
            });
        }
    }
    Ok(())
}

/// The `pos`/`quat` pair every placeable MJCF element carries, with MuJoCo's
/// defaults (origin, identity) where an attribute is absent.
fn rest_pose(node: roxmltree::Node) -> Result<Pose, ParseError> {
    let pos = parse_vec3(node, "pos")?.unwrap_or([0.0; 3]);
    let quat = match parse_floats_attr(node, "quat", 4)? {
        Some(v) => Quat::new(v[0], v[1], v[2], v[3]).normalized(),
        None => Quat::IDENTITY,
    };
    Ok(Pose::new(pos, quat))
}

fn element_children<'a, 'input>(
    node: roxmltree::Node<'a, 'input>,
) -> impl Iterator<Item = roxmltree::Node<'a, 'input>> {
    node.children().filter(|n| n.is_element())
}

fn parse_vec3(node: roxmltree::Node, attr: &str) -> Result<Option<[f64; 3]>, ParseError> {
    Ok(parse_floats_attr(node, attr, 3)?.map(|v| [v[0], v[1], v[2]]))
}

fn parse_floats_attr(
    node: roxmltree::Node,
    attr: &str,
    expected: usize,
) -> Result<Option<Vec<f64>>, ParseError> {
    let Some(s) = node.attribute(attr) else {
        return Ok(None);
    };
    let tag = node.tag_name().name();
    let v: Vec<f64> = s
        .split_whitespace()
        .map(|tok| {
            tok.parse::<f64>().map_err(|_| ParseError::BadFloat {
                tag: tag.to_owned(),
                attr: attr.to_owned(),
                value: tok.to_owned(),
            })
        })
        .collect::<Result<_, _>>()?;
    if v.len() != expected {
        return Err(ParseError::BadVector {
            tag: tag.to_owned(),
            attr: attr.to_owned(),
            expected,
            got: v.len(),
        });
    }
    Ok(Some(v))
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let n = v[2].mul_add(v[2], v[1].mul_add(v[1], v[0] * v[0])).sqrt();
    if n < 1e-12 {
        // A zero axis is a broken model; +z (MJCF's own default) keeps the
        // parse usable and the error visible in FK rather than as NaN.
        return [0.0, 0.0, 1.0];
    }
    [v[0] / n, v[1] / n, v[2] / n]
}
