use crate::all::*;

pub struct Tracker {
  detector: Detector,
  optical_flow: OpticalFlow,
  features0: Vec<Feature>,
  features1: Vec<Feature>,
  features2: Vec<Feature>,
  tracks: Vec<Track>,
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
      tracks: vec![],
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
      update_tracks(&mut self.tracks, &self.features0, &self.features1, false);
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
    update_tracks(&mut self.tracks, &self.features1, &self.features2, true);
  }
}

fn update_tracks(
  tracks: &mut Vec<Track>,
  features0: &[Feature],
  features1: &[Feature],
  new_track: bool,
) {
  for feature0 in features0 {
    for feature1 in features1 {
      if feature1.id == feature0.id {
        if new_track {
          tracks.push(Track::new(*feature0, *feature1));
        }
        else {
          for track in tracks.iter_mut() {
            if track.id == feature0.id {
              track.points.push([feature0.point, feature1.point]);
            }
          }
        }
        break;
      }
    }
  }
  // TODO Remove non-current tracks.
}
