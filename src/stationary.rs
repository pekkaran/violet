/*
use crate::all::*;

pub struct Stationary {
  max_error
}

impl Stationary {
  pub fn new() -> Stationary {
    let stationarity_threshold = {
      let p = PARAMETER_SET.lock().unwrap();
      p.stationarity_threshold
    };
    let max_error =
    Stationary {
      max_error,
    }
  }
}

pub fn is_visually_stationary(tracks: &[Track]) -> bool {
  for track in tracks {
    if track.points.len() < 2 { continue }
    let mut it = track.points.iter().rev();
    let p0 = it.next().unwrap();
    let p1 = it.next().unwrap();
    if (p0 - p1).norm_squared() > max_error { return false }
  }
  true
}
*/
