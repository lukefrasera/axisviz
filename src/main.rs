use std::path::{Path, PathBuf};

use bevy::prelude::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use serde::{ Deserialize, Serialize };
use clap::Parser;
use nalgebra as na;
use na::Isometry3;


pub type NodeId = usize;

#[derive(Debug)]
struct TNode {
    parent: Option<NodeId>,
    children: Vec<NodeId>,
    local: Isometry3<f64>,
    world: Isometry3<f64>,
    dirty: bool,
}

#[derive(Debug)]
struct TransformTree {
    nodes: Vec<TNode>,
}

impl TransformTree {
    fn add_node(&mut self, local: Isometry3<f64>, parent: Option<NodeId>) -> NodeId {
        let id = self.nodes.len();
        self.nodes.push(TNode {
            parent: None,
            children: vec![],
            local,
            world: Isometry3::identity(),
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

impl FileTransformTree {
}

#[derive(Parser, Debug)]
struct Args {
    filename: PathBuf
}

fn main() {
    let args = Args::parse();
    App::new()
        .add_plugins((DefaultPlugins, PanOrbitCameraPlugin))
        .add_systems(Startup, setup)
        .add_systems(Update, draw_gizmo_axes)
        .run();
}

fn setup(mut commands: Commands) {
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
}

fn draw_gizmo_axes(mut gizmos: Gizmos) {
    let origin = Vec3::ZERO;
    let size = 0.5;

    // X = red, Y = green, Z = blue
    gizmos.line(origin, origin + Vec3::X * size, Color::srgb(1.0, 0.0, 0.0));
    gizmos.line(origin, origin + Vec3::Y * size, Color::srgb(0.0, 1.0, 0.0));
    gizmos.line(origin, origin + Vec3::Z * size, Color::srgb(0.0, 0.0, 1.0));
}
