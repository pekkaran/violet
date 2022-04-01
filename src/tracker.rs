use crate::all::*;

pub struct Tracker {
  detector: Detector,
  optical_flow: OpticalFlow,
  detections: Vec<Vector2d>,
  features0: Vec<Vector2d>,
  features1: Vec<Vector2d>,
  statuses: Vec<bool>,
  max_tracks: usize,
}

impl Tracker {
  pub fn new() -> Result<Tracker> {
    let max_tracks = {
      let p = PARAMETER_SET.lock().unwrap();
      p.max_tracks
    };
    Ok(Tracker {
      detector: Detector::new(),
      optical_flow: OpticalFlow::new()?,
      detections: vec![],
      features0: vec![],
      features1: vec![],
      statuses: vec![],
      max_tracks,
    })
  }

  pub fn process(
    &mut self,
    frame0: Option<&Frame>,
    frame1: &Frame,
  ) {
    if let Some(frame0) = frame0 {
      self.optical_flow.process(
        OpticalFlowKind::LeftPreviousToCurrent,
        &frame0.cameras[0],
        &frame1.cameras[0],
        &self.features0,
        &mut self.features1,
        &mut self.statuses,
      );
      remove_failed_tracks(&mut self.features1, &mut self.statuses);

      self.optical_flow.process(
        OpticalFlowKind::LeftCurrentToRightCurrent,
        &frame1.cameras[0],
        &frame1.cameras[1],
        &self.features1,
        &mut self.features0,
        &mut self.statuses,
      );
      remove_failed_tracks(&mut self.features0, &mut self.statuses);
    }

    assert!(self.features0.len() <= self.max_tracks);
    let needed_features_count = self.max_tracks - self.features0.len();
    self.detector.process(&frame1.cameras[0], &mut self.detections, needed_features_count);
    self.optical_flow.process(
      OpticalFlowKind::LeftCurrentToRightCurrentDetection,
      &frame1.cameras[0],
      &frame1.cameras[1],
      &self.detections,
      &mut self.features0,
      &mut self.statuses,
    );
    remove_failed_tracks(&mut self.features0, &mut self.statuses);
  }
}

fn remove_failed_tracks(features: &mut Vec<Vector2d>, statuses: &mut Vec<bool>) {
  let mut i = 0;
  while i < features.len() {
    if statuses[i] {
      i += 1;
      continue;
    }
    features.swap_remove(i);
    statuses.swap_remove(i);
  }
}
