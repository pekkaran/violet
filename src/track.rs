use crate::all::*;

#[derive(Clone, Copy)]
pub struct TrackId(pub usize);

#[derive(Clone, Copy)]
pub struct Feature {
  pub point: Vector2d,
  pub id: TrackId,
}

// pub struct Track {
//   pub features:
// }

