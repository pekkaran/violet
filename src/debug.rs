use crate::all::*;

lazy_static! {
  pub static ref DEBUG_DATA: Mutex<DebugData> = Mutex::new(DebugData::default());
}

#[derive(Default)]
pub struct DebugData {
  pub detections: Vec<Feature>,
  pub detection_mask: Vec<bool>,
  pub flow0: Vec<Feature>,
  pub flow1: Vec<Feature>,
  pub tracks: Vec<Track>,
  pub epipolar: Vec<(Feature, Feature, Vec<Vector2d>)>
}
