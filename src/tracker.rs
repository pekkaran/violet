use crate::all::*;

pub struct Tracker {
  detector: Detector,
  optical_flow: OpticalFlow,
  features0: Vec<Feature>,
  features1: Vec<Feature>,
  features2: Vec<Feature>,
  max_tracks: usize,
  next_id: TrackId,
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
      features0: vec![],
      features1: vec![],
      features2: vec![],
      max_tracks,
      next_id: TrackId(0),
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
      );

      self.optical_flow.process(
        OpticalFlowKind::LeftCurrentToRightCurrent,
        &frame1.cameras[0],
        &frame1.cameras[1],
        &self.features1,
        &mut self.features0,
      );
    }

    assert!(self.features0.len() <= self.max_tracks);
    let needed_features_count = self.max_tracks - self.features0.len();

    self.detector.process(
      &frame1.cameras[0].image,
      &mut self.features1,
      needed_features_count,
      &mut self.next_id
    );
    self.optical_flow.process(
      OpticalFlowKind::LeftCurrentToRightCurrentDetection,
      &frame1.cameras[0],
      &frame1.cameras[1],
      &self.features1,
      &mut self.features2,
    );
    self.features0.extend(self.features2.iter());
  }
}
