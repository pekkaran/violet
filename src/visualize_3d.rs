use kiss3d::camera::ArcBall;
use kiss3d::event::{Action, WindowEvent};
use kiss3d::window::Window;
use nalgebra::Point3;

use crate::all::*;

pub fn run_visualize_3d(rx: mpsc::Receiver<()>) {
  let eye = Point3::new(10.0f32, 10.0, 10.0);
  let at = Point3::origin();
  let mut state = State {
    arc_ball: ArcBall::new(eye, at),
  };

  let mut window = Window::new("3d visualization");

  while !window.should_close() {
    render(&mut window, &mut state);
    // thread::sleep(Duration::from_millis(1));
    if let Ok(_) = rx.try_recv() { break }
  }
  window.close();
}

struct State {
  arc_ball: ArcBall,
}

fn render(window: &mut Window, state: &mut State) {
  for event in window.events().iter() {
    match event.value {
      WindowEvent::Key(key, Action::Release, _) => {
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

  window.render_with_camera(&mut state.arc_ball);
}
