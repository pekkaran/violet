use crate::all::*;

pub struct Tracker {
  detector: Detector,
  optical_flow: OpticalFlow,
  detections: Vec<Vector2i>,
  features0: Vec<Vector2d>,
  features1: Vec<Vector2d>,
  statuses: Vec<bool>,
}

impl Tracker {
  pub fn new() -> Result<Tracker> {
    Ok(Tracker {
      detector: Detector::new(),
      optical_flow: OpticalFlow::new()?,
      detections: vec![],
      features0: vec![],
      features1: vec![],
      statuses: vec![],
    })
  }

  pub fn process(
    &mut self,
    frame0: Option<&Frame>,
    frame1: &Frame,
  ) {
    if let Some(frame0) = frame0 {
      self.features0.clear();
      self.features0.extend(self.detections.iter()
        .map(|p| Vector2d::new(p[0] as f64, p[1] as f64)));

      self.optical_flow.process(
        &frame0.cameras[0],
        &frame1.cameras[0],
        &self.features0,
        &mut self.features1,
        &mut self.statuses,
      );
    }

    self.detector.process(&frame1.cameras[0], &mut self.detections);
  }
}
