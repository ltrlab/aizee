//! Robot-independent, persistent native proximity queries. No ROS, devices or IO.
use parry3d_f64::{bounding_volume::Aabb, math::{Pose, Rotation, Vector}, query, shape::SharedShape};
use serde::Deserialize;
use std::{collections::{BTreeMap, HashSet}, panic::{catch_unwind, AssertUnwindSafe}, slice};

fn body_group() -> String { "body".into() }

#[derive(Deserialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
enum Geometry {
    Convex { vertices: Vec<[f64; 3]> },
    Box { size: [f64; 3] },
    Sphere { radius: f64 },
    Cylinder { radius: f64, length: f64 },
}

#[derive(Deserialize)]
struct Instance {
    id: String,
    link: String,
    #[serde(default)]
    obstacle: bool,
    #[serde(default = "body_group")]
    group: String,
    #[serde(flatten)]
    geometry: Geometry,
}

#[derive(Deserialize)]
struct Model {
    shapes: Vec<Instance>,
    #[serde(default)]
    ignored_links: Vec<String>,
}

struct LinkBlock { links: (usize, usize), slots: Vec<usize> }
struct GroupBlock { groups: (usize, usize), scope: u32, links: Vec<LinkBlock> }

pub struct Scene {
    shapes: Vec<SharedShape>,
    corrections: Vec<Pose>,
    pairs: Vec<(usize, usize)>,
    link_shapes: Vec<Vec<usize>>,
    group_shapes: Vec<Vec<usize>>,
    blocks: Vec<GroupBlock>,
}

fn positive(x: f64) -> bool { x.is_finite() && x > 0.0 }

fn separated(a: &Aabb, b: &Aabb, threshold: f64) -> bool {
    let gap = (a.mins - b.maxs).max(b.mins - a.maxs).max(Vector::ZERO);
    gap.length_squared() > threshold * threshold
}

fn union_bounds(indices: &[usize], bounds: &[Aabb]) -> Aabb {
    let mut bound = bounds[indices[0]];
    for &index in &indices[1..] {
        bound.mins = bound.mins.min(bounds[index].mins);
        bound.maxs = bound.maxs.max(bounds[index].maxs);
    }
    bound
}

impl Scene {
    pub fn from_json(bytes: &[u8]) -> Result<Self, String> {
        let model: Model = serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
        if model.shapes.is_empty() || model.shapes.len() > 4096 {
            return Err("Expected 1..4096 collision instances".into());
        }
        let ignored: HashSet<_> = model.ignored_links.iter().collect();
        if ignored.len() != model.ignored_links.len() || ignored.iter().any(|name|
            !model.shapes.iter().any(|s| &s.link == *name && !s.obstacle)) {
            return Err("Ignored links must identify distinct robot collision links".into());
        }
        let mut ids = HashSet::new();
        let mut shapes = Vec::new();
        let mut corrections = Vec::new();
        for item in &model.shapes {
            if item.id.is_empty() || item.link.is_empty() || item.group.is_empty() || !ids.insert(&item.id) {
                return Err("Invalid or duplicate instance identity".into());
            }
            let mut correction = Pose::IDENTITY;
            let shape = match &item.geometry {
                Geometry::Convex { vertices } => {
                    if vertices.len() < 4 || vertices.len() > 100_000 || !vertices.iter().flatten().all(|x| x.is_finite()) {
                        return Err("Invalid convex vertices".into());
                    }
                    let points: Vec<_> = vertices.iter().map(|v| Vector::from_array(*v)).collect();
                    SharedShape::convex_hull(&points).ok_or("Degenerate convex hull")?
                }
                Geometry::Box { size } if size.iter().all(|x| positive(*x)) =>
                    SharedShape::cuboid(size[0]/2.0, size[1]/2.0, size[2]/2.0),
                Geometry::Sphere { radius } if positive(*radius) => SharedShape::ball(*radius),
                Geometry::Cylinder { radius, length } if positive(*radius) && positive(*length) => {
                    // URDF cylinders use Z; Parry cylinders use Y.
                    correction = Pose::from_parts(Vector::ZERO, Rotation::from_rotation_x(std::f64::consts::FRAC_PI_2));
                    SharedShape::cylinder(*length/2.0, *radius)
                }
                _ => return Err("Nonpositive or nonfinite primitive".into()),
            };
            shapes.push(shape);
            corrections.push(correction);
        }
        let pairs: Vec<_> = model.shapes.iter().enumerate().flat_map(|(i, a)| {
            let ignored = &ignored;
            model.shapes.iter().enumerate().skip(i+1)
                .filter(move |(_, b)| a.link != b.link && !(a.obstacle && b.obstacle)
                    && !ignored.contains(&a.link) && !ignored.contains(&b.link)).map(move |(j, _)| (i, j))
        }).collect();
        // Compile a two-level hierarchy once. Output slots retain the original
        // cross-link pair order regardless of hierarchy traversal order.
        let mut link_ids = BTreeMap::new();
        let mut group_ids = BTreeMap::new();
        let mut identities = Vec::new();
        let mut link_shapes: Vec<Vec<usize>> = Vec::new();
        let mut group_shapes: Vec<Vec<usize>> = Vec::new();
        for (i, shape) in model.shapes.iter().enumerate() {
            let next = group_ids.len();
            let group = *group_ids.entry(&shape.group).or_insert(next);
            if group == group_shapes.len() { group_shapes.push(Vec::new()); }
            group_shapes[group].push(i);
            let next = link_ids.len();
            let (link, old_group, old_obstacle) = *link_ids.entry(&shape.link)
                .or_insert((next, group, shape.obstacle));
            if old_group != group || old_obstacle != shape.obstacle {
                return Err("Inconsistent collision link membership".into());
            }
            if link == link_shapes.len() { link_shapes.push(Vec::new()); }
            link_shapes[link].push(i);
            identities.push((link, group));
        }
        let mut hierarchy = BTreeMap::<(usize, usize, u32), BTreeMap<(usize, usize), Vec<usize>>>::new();
        for (slot, &(i, j)) in pairs.iter().enumerate() {
            let (a, b) = (&model.shapes[i], &model.shapes[j]);
            let scope = if a.obstacle || b.obstacle { 8 }
                else if a.group == "body" || b.group == "body" { 4 }
                else if a.group == b.group { 2 } else { 1 };
            let ((la, ga), (lb, gb)) = (identities[i], identities[j]);
            hierarchy.entry((ga.min(gb), ga.max(gb), scope)).or_default()
                .entry((la.min(lb), la.max(lb))).or_default().push(slot);
        }
        let blocks = hierarchy.into_iter().map(|((ga, gb, scope), links)| GroupBlock {
            groups: (ga, gb), scope,
            links: links.into_iter().map(|(links, slots)| LinkBlock { links, slots }).collect(),
        }).collect();
        Ok(Self { shapes, corrections, pairs, link_shapes, group_shapes, blocks })
    }

    /// Populate distances in stable cross-link pair order. Infinity means outside
    /// the requested range, not an exact distance. Contact/containment is zero.
    pub fn proximity(&self, transforms: &[[f64; 7]], threshold: f64, output: &mut [f64]) -> Result<(), String> {
        self.proximity_scoped(transforms, threshold, 15, output, &mut [0; 8])
    }

    /// Counters: eligible pairs, group tests/rejections, link tests/rejections,
    /// part AABB tests, convex distance tests, fine-phase candidates.
    pub fn proximity_scoped(&self, transforms: &[[f64; 7]], threshold: f64, scope: u32,
                            output: &mut [f64], stats: &mut [u64; 8]) -> Result<(), String> {
        if transforms.len() != self.shapes.len() || output.len() != self.pairs.len()
            || !threshold.is_finite() || threshold < 0.0 || scope == 0 || scope & !15 != 0 {
            return Err("Invalid query dimensions or threshold".into());
        }
        let mut poses = Vec::with_capacity(transforms.len());
        for (i, t) in transforms.iter().enumerate() {
            if !t.iter().all(|x| x.is_finite()) || (t[3..].iter().map(|x| x*x).sum::<f64>()-1.0).abs() > 1e-6 {
                return Err("Transforms require finite translations and unit xyzw quaternions".into());
            }
            poses.push(Pose::from_parts(Vector::new(t[0], t[1], t[2]),
                Rotation::from_xyzw(t[3], t[4], t[5], t[6])) * self.corrections[i]);
        }
        let bounds: Vec<_> = self.shapes.iter().zip(&poses).map(|(s,p)| s.compute_aabb(p)).collect();
        let links: Vec<_> = self.link_shapes.iter().map(|indices| union_bounds(indices, &bounds)).collect();
        let groups: Vec<_> = self.group_shapes.iter().map(|indices| union_bounds(indices, &bounds)).collect();
        output.fill(f64::INFINITY);
        stats.fill(0);
        for block in &self.blocks {
            if block.scope & scope == 0 { continue; }
            stats[0] += block.links.iter().map(|link| link.slots.len() as u64).sum::<u64>();
            stats[1] += 1;
            if separated(&groups[block.groups.0], &groups[block.groups.1], threshold) {
                stats[2] += 1;
                continue;
            }
            for link in &block.links {
                stats[3] += 1;
                if separated(&links[link.links.0], &links[link.links.1], threshold) {
                    stats[4] += 1;
                    continue;
                }
                for &slot in &link.slots {
                    let (i, j) = self.pairs[slot];
                    stats[5] += 1;
                    if separated(&bounds[i], &bounds[j], threshold) { continue; }
                    stats[6] += 1;
                    let d = query::distance(&poses[i], self.shapes[i].as_ref(), &poses[j], self.shapes[j].as_ref())
                        .map_err(|_| "Unsupported proximity query")?;
                    if !d.is_finite() || d < 0.0 { return Err("Invalid native distance".into()); }
                    if d <= threshold { output[slot] = d; stats[7] += 1; }
                }
            }
        }
        Ok(())
    }
}

// C ABI: handles own immutable scenes. Buffers belong to the caller. No strings,
// allocation ownership or panics cross the ABI. Invalid input returns failure.
#[no_mangle]
pub extern "C" fn rpc_abi_version() -> u32 { 3 }

/// # Safety
/// `data` must reference `len` readable bytes for this call.
#[no_mangle]
pub unsafe extern "C" fn rpc_create(data: *const u8, len: usize) -> *mut Scene {
    if data.is_null() || len == 0 || len > 256*1024*1024 { return std::ptr::null_mut(); }
    catch_unwind(AssertUnwindSafe(|| {
        Scene::from_json(slice::from_raw_parts(data,len)).ok().map(|s| Box::into_raw(Box::new(s)))
            .unwrap_or(std::ptr::null_mut())
    })).unwrap_or(std::ptr::null_mut())
}

/// # Safety
/// `scene` must be a live handle from rpc_create. No query may overlap this call.
#[no_mangle]
pub unsafe extern "C" fn rpc_destroy(scene: *mut Scene) {
    if !scene.is_null() { drop(Box::from_raw(scene)); }
}

/// # Safety
/// The live handle and buffers must remain valid, correctly sized and nonaliasing
/// for this call. Each transform is xyz followed by quaternion xyzw.
#[no_mangle]
pub unsafe extern "C" fn rpc_query(scene: *const Scene, poses: *const f64, count: usize,
    threshold: f64, scope: u32, output: *mut f64, output_count: usize, stats: *mut u64) -> i32 {
    // stats must address eight writable u64 counters.
    if scene.is_null() || poses.is_null() || output.is_null() || stats.is_null() { return -1; }
    catch_unwind(AssertUnwindSafe(|| {
        let scene = &*scene;
        if count != scene.shapes.len() || output_count != scene.pairs.len() { return -1; }
        match scene.proximity_scoped(slice::from_raw_parts(poses.cast::<[f64;7]>(), count), threshold, scope,
                              slice::from_raw_parts_mut(output, output_count), &mut *stats.cast::<[u64;8]>()) {
            Ok(()) => 0, Err(_) => -1,
        }
    })).unwrap_or(-2)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn scene() -> Scene {
        Scene::from_json(br#"{"shapes":[{"id":"a/0","link":"a","shape":"box","size":[2,2,2]},
        {"id":"b/0","link":"b","shape":"sphere","radius":0.2}]}"#).unwrap()
    }
    #[test]
    fn distance_contact_containment_and_repeated_poses() {
        let s=scene(); let mut out=[0.0];
        for (x, expected) in [(2.0,0.8),(0.0,0.0),(1.2,0.0),(2.0,0.8)] {
            s.proximity(&[[0.,0.,0.,0.,0.,0.,1.],[x,0.,0.,0.,0.,0.,1.]], 1.0, &mut out).unwrap();
            assert!((out[0]-expected).abs()<1e-6, "{:?}",out);
        }
    }
    #[test]
    fn invalid_queries_fail() {
        let s=scene(); let mut out=[0.0];
        assert!(s.proximity(&[[0.;7];2],1.,&mut out).is_err());
        assert!(s.proximity(&[[0.,0.,0.,0.,0.,0.,1.];2],f64::NAN,&mut out).is_err());
        assert!(Scene::from_json(br#"{"shapes":[]}"#).is_err());
    }
    #[test]
    fn threshold_and_same_link_filtering() {
        let s=scene(); let mut out=[0.0];
        s.proximity(&[[0.,0.,0.,0.,0.,0.,1.],[2.,0.,0.,0.,0.,0.,1.]],0.1,&mut out).unwrap();
        assert_eq!(out[0],f64::INFINITY);
        let s=Scene::from_json(br#"{"shapes":[{"id":"a/0","link":"a","shape":"sphere","radius":1},
        {"id":"a/1","link":"a","shape":"sphere","radius":1}]}"#).unwrap();
        assert!(s.pairs.is_empty());
    }

    #[test]
    fn hierarchy_scopes_match_exhaustive_sphere_distances() {
        let groups = ["branch:a", "branch:a", "branch:b", "body", "environment", "environment"];
        let raw = serde_json::json!({"shapes": groups.iter().enumerate().map(|(i,g)|
            serde_json::json!({"id": format!("{i}/0"), "link": i.to_string(), "group": g,
                              "obstacle": i >= 4, "shape":"sphere", "radius":0.1})).collect::<Vec<_>>()});
        let s = Scene::from_json(&serde_json::to_vec(&raw).unwrap()).unwrap();
        let mut output = vec![0.; s.pairs.len()];
        let mut stats = [0;8];
        for sample in 0..100 {
            let poses: Vec<_> = (0..6).map(|i| [((sample * (i+1)) as f64 * 0.1).sin(),
                ((sample + i*19) as f64 * 0.1).cos(), 0., 0., 0., 0., 1.]).collect();
            for scope in [1,2,4,8,15] {
                s.proximity_scoped(&poses, 0.05, scope, &mut output, &mut stats).unwrap();
                let mut eligible = 0;
                for (slot, &(i,j)) in s.pairs.iter().enumerate() {
                    let kind = if i >= 4 || j >= 4 {8} else if i == 3 || j == 3 {4}
                        else if groups[i] == groups[j] {2} else {1};
                    let selected = kind & scope != 0;
                    eligible += u64::from(selected);
                    let distance = ((poses[i][0]-poses[j][0]).hypot(poses[i][1]-poses[j][1])-0.2).max(0.);
                    if selected && distance <= 0.05 {
                        assert!((output[slot]-distance).abs() < 1e-8);
                    } else { assert!(output[slot].is_infinite()); }
                }
                assert_eq!(stats[0], eligible);
            }
        }
        assert!(s.proximity_scoped(&[[0.,0.,0.,0.,0.,0.,1.];6],0.,16,&mut output,&mut stats).is_err());
    }

    #[test]
    fn distant_branches_reject_before_part_tests() {
        let s = Scene::from_json(br#"{"shapes":[
            {"id":"a/0","link":"a","group":"branch:a","shape":"sphere","radius":0.1},
            {"id":"b/0","link":"b","group":"branch:b","shape":"sphere","radius":0.1}]}"#).unwrap();
        let mut out = [0.]; let mut stats = [0;8];
        s.proximity_scoped(&[[0.,0.,0.,0.,0.,0.,1.],[2.,0.,0.,0.,0.,0.,1.]],0.01,1,&mut out,&mut stats).unwrap();
        assert_eq!(stats,[1,1,1,0,0,0,0,0]);
    }

    #[test]
    fn ignored_links_filter_pairs_without_changing_shape_indices() {
        let shapes = ["arm_a", "finger", "arm_b"].iter().map(|name|
            serde_json::json!({"id":format!("{name}/0"),"link":name,"shape":"sphere","radius":0.1}))
            .collect::<Vec<_>>();
        let raw = serde_json::json!({"shapes":shapes, "ignored_links":["finger"]});
        let s = Scene::from_json(&serde_json::to_vec(&raw).unwrap()).unwrap();
        assert_eq!(s.pairs, vec![(0,2)]);
        let mut output = [0.];
        s.proximity(&[[0.,0.,0.,0.,0.,0.,1.];3],0.,&mut output).unwrap();
        assert_eq!(output, [0.]);
        let mut invalid = raw;
        invalid["ignored_links"] = serde_json::json!(["typo"]);
        assert!(Scene::from_json(&serde_json::to_vec(&invalid).unwrap()).is_err());
    }
}
