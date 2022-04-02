use crate::all::*;

lazy_static! {
  pub static ref DEBUG_DATA: Mutex<DebugData> = Mutex::new(DebugData::default());
}

#[derive(Default)]
pub struct DebugData {
  pub detections: Vec<Feature>,
  pub detection_mask: Vec<bool>,
  pub flow: Vec<Feature>,
}
