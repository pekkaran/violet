use crate::all::*;

pub struct Tracker {
  detector: Detector,
  detections: Vec<Pixel>,
}

impl Tracker {
  pub fn new() -> Tracker {
    Tracker {
      detector: Detector::new(),
      detections: vec![],
    }
  }

  pub fn process(&mut self, frame: &InputFrame) {
    self.detector.process(&frame.video, &mut self.detections);
    // dbg!(&self.detections);
  }
}

