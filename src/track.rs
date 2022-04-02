use crate::all::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackId(pub usize);

#[derive(Clone, Copy, Debug)]
pub struct Feature {
  pub point: Vector2d,
  pub id: TrackId,
}

#[derive(Clone)]
pub struct Track {
  pub points: Vec<[Vector2d; 2]>,
  pub id: TrackId,
  pub last_seen: TrackerStep,
}

impl Track {
  pub fn new(
    feature0: Feature,
    feature1: Feature,
    last_seen: TrackerStep,
  ) -> Track {
    assert_eq!(feature0.id, feature1.id);
    Track {
      points: vec![[feature0.point, feature1.point]],
      id: feature0.id,
      last_seen,
    }
  }
}
