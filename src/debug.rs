use crate::all::*;

lazy_static! {
  pub static ref DEBUG_DATA: Mutex<DebugData> = Mutex::new(DebugData::default());
}

#[derive(Default)]
pub struct DebugData {
  pub detections: Vec<Vector2i>,
  pub detection_mask: Vec<bool>,
}
