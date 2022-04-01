use crate::all::*;

lazy_static! {
  pub static ref DEBUG_DATA: Mutex<DebugData> = Mutex::new(DebugData::default());
}

#[derive(Default)]
pub struct DebugData {
  pub detections: Vec<Vector2d>,
  pub detection_mask: Vec<bool>,
  pub flow: Vec<(Vector2d, Vector2d)>,
}
