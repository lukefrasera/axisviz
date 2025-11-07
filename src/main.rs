use std::path::{Path, PathBuf};

use bevy::asset::ron::de::Position;
use bevy::camera::ViewportConversionError;
use bevy::prelude::*;
use bevy_debug_grid::DebugGridPlugin;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use serde::{ Deserialize, Serialize };
use clap::Parser;
use nalgebra as na;
use na::Isometry3;
use anyhow::Result;
use std::fs::File;
use std::io::BufReader;
use serde_json;
use std::f64::consts::{PI, TAU, FRAC_PI_2, FRAC_PI_4};
use std::convert::TryFrom;
use thiserror::Error;
use std::collections::HashMap;



pub type NodeId = usize;

#[derive(Debug)]
struct TNode {
    name: String,
    parent: Option<NodeId>,
    children: Vec<NodeId>,
    local: Isometry3d,
    world: Isometry3d,
    dirty: bool,
}

#[derive(Debug, Resource)]
struct TransformTree {
    nodes: Vec<TNode>,
}

impl TransformTree {
    fn add_node(&mut self, name: &str, local: Isometry3d, parent: Option<NodeId>) -> NodeId {
        let id = self.nodes.len();
        self.nodes.push(TNode {
            name: name.to_string(),
            parent: None,
            children: vec![],
            local,
            world: Isometry3d::IDENTITY,
            dirty: true
        });
        if let Some(p) = parent && p < id {
            self.nodes[p].children.push(id);
            self.nodes[id].parent = Some(p);
            self.nodes[id].world = self.nodes[p].world * self.nodes[id].local;
            self.nodes[id].dirty = false;
        } else {
            self.nodes[id].world = self.nodes[id].local;
            self.nodes[id].dirty = false;
        }
        id
    }
    fn name_hash(&self) -> Result<HashMap<String, NodeId>, FileTransformTreeError> {
        let mut map = HashMap::with_capacity(self.nodes.len());
        for (id, node) in self.nodes.iter().enumerate() {
            let name = &node.name;
            if map.insert(name.clone(), id).is_some() {
                return Err(FileTransformTreeError::Duplicate(name.clone()));
            }
        }
        Ok(map)
    }
    fn set_parent(&mut self, id: NodeId, parent: Option<NodeId>) {
        if let Some(p) = self.nodes[id].parent.take() {
            self.nodes[p].children.retain(|&c| c != id);
        }
        if let Some(p) = parent {
            self.nodes[p].children.push(id);
            self.nodes[id].parent = Some(p);
        }
        self.mark_dirty(id);
    }
    fn mark_dirty(&mut self, id: NodeId) {
        use std::collections::VecDeque;
        let mut q = VecDeque::from([id]);
        while let Some(n) = q.pop_front() {
            if !self.nodes[n].dirty {
                self.nodes[n].dirty = true;
                q.extend(self.nodes[n].children.iter().copied());
            }
        }
    }
    fn update_world(&mut self) {
        let roots: Vec<_> = (0..self.nodes.len())
            .filter(|&i| self.nodes[i].parent.is_none())
            .collect();
        let mut stack: Vec<(NodeId, Isometry3d)> = roots.into_iter().map(|r| (r, Isometry3d::IDENTITY)).collect();
        while let Some((id, parent_world)) = stack.pop() {
            if self.nodes[id].dirty {
                self.nodes[id].world = parent_world * self.nodes[id].local;
                self.nodes[id].dirty = false;
            }
            let world = self.nodes[id].world;
            for &child in &self.nodes[id].children {
                stack.push((child, world));
            }
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct FileTransformTree {
    pub version: u32,
    pub nodes: Vec<FileNode>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct FileNode {
    pub name: String,
    pub parent: Option<String>,
    pub t: [f64; 3],
    pub r: [f64; 3],
}

impl From<&FileNode> for Isometry3d {
    fn from(node: &FileNode) -> Self {
        let [tx, ty, tz] = node.t;
        let [r, p, y] = node.r;
        let rot = Quat::from_euler(EulerRot::XYZ, r as f32, p as f32, y as f32);
        Isometry3d::new(Vec3::new(tx as f32, ty as f32, tz as f32), rot)
    }
}

impl FileTransformTree {
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        Ok(serde_json::from_reader(reader)?)
    }
    pub fn name_hash(&self) -> Result<HashMap<String, NodeId>, FileTransformTreeError> {
        let mut map = HashMap::with_capacity(self.nodes.len());
        for (id, node) in self.nodes.iter().enumerate() {
            let name = &node.name;
            if map.insert(name.clone(), id).is_some() {
                return Err(FileTransformTreeError::Duplicate(name.clone()));
            }
        }
        Ok(map)
    }
}

#[derive(Error, Debug)]
pub enum FileTransformTreeError {
    #[error("Unknown Parent")]
    ParentMissing(String),

    #[error("Duplicate Name")]
    Duplicate(String),

    #[error("Serialization Error")]
    Serialization(String),
}

impl TryFrom<FileTransformTree> for TransformTree {
    type Error = FileTransformTreeError;

    fn try_from(ftree: FileTransformTree) -> Result<Self, Self::Error> {
        // let name_map = ftree.name_hash()?;
        let mut res = TransformTree{
            nodes: vec![],
        };
        for node in ftree.nodes.iter() {
            res.add_node(node.name.as_str(), Isometry3d::from(node), None);
        }
        let name_map = res.name_hash()?;
        for node in ftree.nodes.iter() {
            if let Some(p) = node.parent.clone() {
                res.set_parent(name_map[&node.name], Some(name_map[&p]));
            }
        }
        res.update_world();
        Ok(res)
    }
}

fn load_transform_tree(path: impl AsRef<Path>) -> Result<TransformTree, FileTransformTreeError> {
    match FileTransformTree::load(path) {
        Ok(dag) => TransformTree::try_from(dag),
        Err(e) => Err(FileTransformTreeError::Serialization(e.to_string())),
    }
}

#[derive(Parser, Debug)]
struct Args {
    filename: PathBuf
}

fn main() {
    let args = Args::parse();
    let ttree = FileTransformTree {
        version: 1u32,
        nodes: vec![
            FileNode {
                name: "arm_base".to_string(),
                parent: None,
                t: [0.,0.,0.],
                r: [0.0, 0., 0.]
            },
            FileNode {
                name: "lidar".to_string(),
                parent: Some("arm_base".to_string()),
                t: [0.5, 0., 0.],
                r: [PI/2., 0., 0.]
            }
        ]
    };
    println!("Json Tree:\n{}", serde_json::to_string(&ttree).unwrap_or("Failed to serialize".to_string()));

    match load_transform_tree(args.filename) {
        Ok(dag) => {
            println!("Dag: {:?}", dag);
            App::new()
                .insert_resource(dag)
                .add_plugins((DefaultPlugins, PanOrbitCameraPlugin, MeshPickingPlugin, DebugGridPlugin::with_floor_grid()))
                .add_systems(Startup, setup)
                .add_systems(Update, draw_gizmo_axes)
                .run();
        },
        Err(e) => println!("Error: {:?}", e)
    }
}

#[derive(Component)]
struct AxisOverlayLabel {
    node: NodeId,
}

#[derive(Resource, Default)]
struct Selection {
    nodes: Vec<NodeId>,
}

fn setup(mut commands: Commands, dag: Res<TransformTree>, asset_server: Res<AssetServer>, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
    let focus = Vec3::ZERO;
    let transform = Transform::from_xyz(3.0, 2.0, 3.0).looking_at(focus, Vec3::Y);

    // Camera
    commands.spawn((
        Camera3d::default(),
        transform,
        PanOrbitCamera::default(),
    ));

    // Light (not needed for gizmos, but good for if you add meshes later)
    commands.spawn((
        PointLight::default(),
        Transform::from_xyz(2.0, 4.0, 2.0),
    ));

    let font = asset_server.load("fonts/FiraCode.ttf");
    commands.spawn((Node {
        position_type: PositionType::Absolute,
        ..default()
        },
    )).with_children(|root| {
        for (id, node) in dag.nodes.iter().enumerate() {
            root.spawn((
                AxisOverlayLabel {
                    node: id
                },
                Text::new(node.name.clone()),
                TextFont {
                    font: font.clone(),
                    font_size: 20.0,
                    ..default()
                },
                Node {
                    position_type: PositionType::Absolute,
                    ..default()
                },
                TextColor(Color::srgb(1.0, 1.0, 1.0)),
            ));
        }
    });
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            ..default()
        },
        Transform {
            ..default()
        }
    )).with_children(
        |root| {
            for (id, node) in dag.nodes.iter().enumerate() {
                root.spawn((
                    Mesh3d(meshes.add(Sphere::new(0.02))),
                    MeshMaterial3d(materials.add(StandardMaterial{
                        base_color: Color::srgb(1.0, 1.0, 1.0),
                        ..default()
                    })),
                    Transform {
                        translation: node.world.translation.to_vec3(),
                        ..default()
                    }
                )).observe(on_center_camera);
            }
        });
}

fn on_center_camera(click: On<Pointer<Click>>, mut transforms: Query<&mut Transform>, mut camera_q: Query<&mut PanOrbitCamera>) {
    let transform = transforms.get_mut(click.entity).unwrap();
    println!("on_center_camera: {:?}", transform.translation);
    if let Ok(mut camera) = camera_q.single_mut() {
        camera.target_focus = transform.translation;
    }
}

fn draw_gizmo_axes(dag: Res<TransformTree>, mut gizmos: Gizmos, camera_q: Query<(&Camera, &GlobalTransform), With<Camera3d>>, mut label_q: Query<(&mut Node, &AxisOverlayLabel, &mut Visibility)>) {
    let size = 0.2;

    for node in dag.nodes.iter() {
        let o = node.world.translation.to_vec3();
        gizmos.line(o, o + node.world.rotation * Vec3::X * size, Color::srgb(1.0, 0.0, 0.0));
        gizmos.line(o, o + node.world.rotation * Vec3::Y * size, Color::srgb(0.0, 1.0, 0.0));
        gizmos.line(o, o + node.world.rotation * Vec3::Z * size, Color::srgb(0.0, 0.0, 1.0));
        if let Some(p) = node.parent {
            gizmos.line(dag.nodes[p].world.translation.to_vec3(), o, Color::srgb(1.0, 1.0, 0.0));
        }
    }

    if let Ok((camera, cam_transform)) = camera_q.single() {
        for (mut node, label, mut visibility) in &mut label_q {
            let n = &dag.nodes[label.node];
            let world_pos = n.world.translation.to_vec3();

            match camera.world_to_viewport(cam_transform, world_pos) {
                Ok(pos) => {
                    *visibility = Visibility::Visible;
                    node.left = Val::Px(pos.x);
                    node.top = Val::Px(pos.y);
                }
                Err(ViewportConversionError::PastNearPlane | ViewportConversionError::PastFarPlane) => {
                    *visibility = Visibility::Hidden;
                }
                Err(_) => {
                    *visibility = Visibility::Hidden;
                }
            }
        }
    }
}

// fn handle_pointer_select(
//     mut selection: ResMut<Selection>,
//     dag: Res<TransformTree>,
//     mut camera_q: Query<&mut PanOrbitCamera>,
//     button: Res<ButtonInput<MouseButton>>,
//     keys: Res<ButtonInput<KeyCode>>,
//     windows: Query<&Window>,
//     camera_transform_q: Query<(&Camera, &GlobalTransform), With<MeshPickingCamera>>,
// ) {
//     if !button.just_pressed(MouseButton::Left) {
//         return;
//     }
//
//     let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
//     if let Some(hit_id) = ray
// }
