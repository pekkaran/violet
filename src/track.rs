use crate::all::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackId(pub usize);

#[derive(Clone, Copy, Debug)]
pub struct Feature {
  pub point: Vector2d,
  pub id: TrackId,
}

#[derive(Clone, Debug)]
pub struct Track {
  pub points: Vec<TrackPoint>,
  pub id: TrackId,
  pub last_seen: TrackerStep,
}

#[derive(Clone, Debug)]
pub struct TrackPoint {
  // Pixels, not normalized.
  pub coordinates: [Vector2d; 2],
  pub frame_number: usize,
}

impl Track {
  pub fn new(
    feature0: Feature,
    feature1: Feature,
    last_seen: TrackerStep,
    frame_number: usize,
  ) -> Track {
    assert_eq!(feature0.id, feature1.id);
    Track {
      points: vec![TrackPoint {
        coordinates: [feature0.point, feature1.point],
        frame_number,
      }],
      id: feature0.id,
      last_seen,
    }
  }
}
