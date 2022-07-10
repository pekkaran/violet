use crate::all::*;

pub struct Stationary {
  max_error: f64,
}

impl Stationary {
  pub fn new(frame_scale: f64) -> Stationary {
    let p = PARAMETER_SET.lock().unwrap();
    let max_error = (frame_scale * p.stationarity_threshold).powi(2);
    Stationary {
      max_error,
    }
  }

  pub fn check(&self, tracks: &[Track]) -> bool {
    for track in tracks {
      if track.points.len() < 2 { continue }
      let mut it = track.points.iter().rev();
      // Compare two last points from the first stereo camera.
      let p0: Vector2d = it.next().unwrap()[0];
      let p1: Vector2d = it.next().unwrap()[0];
      if (p0 - p1).norm_squared() > self.max_error { return false }
    }
    true
  }
}
