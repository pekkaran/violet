use kiss3d::camera::ArcBall;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::window::Window;
use nalgebra::Point3;

use crate::all::*;

pub fn run_visualize_3d(rx: mpsc::Receiver<()>) {
  let eye = Point3::new(10.0f32, 10.0, 10.0);
  let at = Point3::origin();
  let scale = 1.0;
  let mut state = State {
    arc_ball: ArcBall::new(eye, at),
    pose_trail: vec![],
    triangulation_positions: vec![],
    camera_lines: compute_camera_lines(scale),
    head_position_trail: vec![],
  };

  let mut window = Window::new("3d visualization");

  while !window.should_close() {
    render(&mut window, &mut state);
    std::thread::sleep(std::time::Duration::from_millis(1));
    if let Ok(_) = rx.try_recv() { break }
  }
  window.close();
}

type CameraLines = [[Vector3d; 2]; 8];

struct State {
  arc_ball: ArcBall,
  pose_trail: Vec<Matrix4d>,
  triangulation_positions: Vec<Vector3d>,
  camera_lines: CameraLines,
  head_position_trail: Vec<Vector3d>,
}

fn render(mut window: &mut Window, state: &mut State) {
  for event in window.events().iter() {
    match event.value {
      WindowEvent::Key(key, Action::Release, _) => {
        if key == Key::Q { window.close() }
      }
      _ => {}
    }
  }

  // Draw axes.
  for i in 0..3 {
    let mut p = Point3::new(0.0, 0.0, 0.0);
    p[i] = 1.;
    window.draw_line(&Point3::origin(), &p, &p);
  }

  // Move data to quickly release the lock.
  {
    let d = &mut DEBUG_DATA_3D.lock().unwrap();
    if !d.pose_trail.is_empty() {
      mem::swap(&mut state.pose_trail, &mut d.pose_trail);
      d.pose_trail.clear();
    }
    if !d.triangulation_positions.is_empty() {
      mem::swap(&mut state.triangulation_positions, &mut d.triangulation_positions);
      d.triangulation_positions.clear();
    }
  }

  if let Some(head_pose) = state.pose_trail.get(0) {
    let pos = position!(head_pose);
    if state.head_position_trail.is_empty()
      || (state.head_position_trail.iter().last().unwrap() - pos).norm_squared() > 1e-4
    {
      state.head_position_trail.push(pos.into());
    }
  }

  let color = Point3::new(0., 1., 0.);
  for w in state.head_position_trail.windows(2) {
    window.draw_line(
      &Point3f::from(w[0].cast::<f32>()),
      &Point3f::from(w[1].cast::<f32>()),
      &color
    );
  }

  for T in &state.pose_trail {
    draw_camera(&mut window, T, &state.camera_lines);
  }

  for p in &state.triangulation_positions {
    window.draw_point(&Point3f::from(p.cast::<f32>()), &color);
  }

  window.render_with_camera(&mut state.arc_ball);
}

fn draw_camera(window: &mut Window, T: &Matrix4d, camera_lines: &CameraLines) {
  let color = Point3::new(0., 1., 0.);
  for line in camera_lines.iter() {
    let p0 = transform_3d(T, &line[0]).cast::<f32>();
    let p1 = transform_3d(T, &line[1]).cast::<f32>();
    window.draw_line(&Point3f::from(p0), &Point3f::from(p1), &color);
  }
}

fn compute_camera_lines(w: f64) -> CameraLines {
  let h = w * 0.667;
  let z = h * 0.9;
  [
    [Vector3d::new(0., 0., 0.), Vector3d::new(w, h, z)],
    [Vector3d::new(0., 0., 0.), Vector3d::new(w, -h, z)],
    [Vector3d::new(0., 0., 0.), Vector3d::new(-w, -h, z)],
    [Vector3d::new(0., 0., 0.), Vector3d::new(-w, h, z)],
    [Vector3d::new(w, h, z), Vector3d::new(w, -h, z)],
    [Vector3d::new(-w, h, z), Vector3d::new(-w, -h, z)],
    [Vector3d::new(-w, h, z), Vector3d::new(w, h, z)],
    [Vector3d::new(-w, -h, z), Vector3d::new(w, -h, z)],
  ]
}
