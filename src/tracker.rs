use crate::all::*;

pub struct Tracker {
  detector: Detector,
}

impl Tracker {
  pub fn new() -> Tracker {
    Tracker {
      detector: Detector::new(),
    }
  }

  pub fn process(&mut self, frame: &InputFrame) {
    self.detector.process(frame);
  }
}

