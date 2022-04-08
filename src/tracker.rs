use crate::all::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackerStep(pub usize);

pub struct Tracker {
  detector: Detector,
  optical_flow: OpticalFlow,
  features0: Vec<Feature>,
  features1: Vec<Feature>,
  features2: Vec<Feature>,
  tracks: Vec<Track>,
  max_tracks: usize,
  next_id: TrackId,
  step: TrackerStep,
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
      step: TrackerStep(0),
    })
  }

  pub fn process(
    &mut self,
    frame0: Option<&Frame>,
    frame1: &Frame,
    cameras: &[Camera],
  ) {
    if let Some(frame0) = frame0 {
      self.optical_flow.process(
        OpticalFlowKind::LeftPreviousToCurrent,
        &frame0.cameras[0],
        &frame1.cameras[0],
        &[&cameras[0], &cameras[0]],
        &self.features1,
        &mut self.features2,
        &mut self.features0,
      );

      self.optical_flow.process(
        OpticalFlowKind::LeftCurrentToRightCurrent,
        &frame1.cameras[0],
        &frame1.cameras[1],
        &[&cameras[0], &cameras[1]],
        &self.features0,
        &mut self.features1,
        &mut self.features2,
      );
      update_tracks(&mut self.tracks, &self.features1, &self.features2, false, self.step);
    }

    // TODO Make this adaptive.
    let min_distance = 5.0;
    sparsify_tracks(&mut self.tracks, min_distance);

    assert!(self.features2.len() <= self.max_tracks);
    let needed_features_count = self.max_tracks - self.features2.len();

    self.detector.process(
      &frame1.cameras[0].image,
      &mut self.features0,
      needed_features_count,
      &mut self.next_id
    );
    self.optical_flow.process(
      OpticalFlowKind::LeftCurrentToRightCurrentDetection,
      &frame1.cameras[0],
      &frame1.cameras[1],
      &[&cameras[0], &cameras[1]],
      &self.features0,
      &mut self.features1,
      &mut self.features2,
    );
    update_tracks(&mut self.tracks, &self.features1, &self.features2, true, self.step);

    self.step.0 += 1
  }
}

fn update_tracks(
  tracks: &mut Vec<Track>,
  features0: &[Feature],
  features1: &[Feature],
  new_tracks: bool,
  step: TrackerStep,
) {
  assert_eq!(features0.len(), features1.len());
  for (feature0, feature1) in features0.iter().zip(features1.iter()) {
    assert_eq!(feature0.id, feature1.id);
    if new_tracks {
      tracks.push(Track::new(*feature0, *feature1, step));
    }
    else {
      for track in tracks.iter_mut() {
        if track.id == feature0.id {
          track.points.push([feature0.point, feature1.point]);
          track.last_seen = step;
        }
      }
    }
  }

  let mut i = 0;
  while i < tracks.len() {
    if tracks[i].last_seen.0 == step.0 {
      i += 1;
      continue;
    }
    tracks.swap_remove(i);
  }
  let d = &mut DEBUG_DATA.lock().unwrap();
  let p = PARAMETER_SET.lock().unwrap();
  if p.show_tracks {
    d.tracks.clear();
    d.tracks.extend(tracks.iter().cloned());
  }
}

fn sparsify_tracks(
  tracks: &mut Vec<Track>,
  min_distance: f64,
) {
  let d2 = min_distance.powi(2);
  for i0 in 0..tracks.len() {
    for i1 in (i0 + 1)..tracks.len() {
      if tracks[i0].points.is_empty() { continue }
      if tracks[i1].points.is_empty() { continue }
      let p0 = tracks[i0].points.iter().last().unwrap();
      let p1 = tracks[i1].points.iter().last().unwrap();
      for k in 0..2 {
        if (p0[k] - p1[k]).norm_squared() < d2 {
          assert_eq!(tracks[i0].last_seen, tracks[i1].last_seen);
          if tracks[i0].points.len() < tracks[i1].points.len() {
            tracks[i0].points.clear();
          }
          else {
            tracks[i1].points.clear();
          }
          break;
        }
      }
    }
  }
  let mut i = 0;
  while i < tracks.len() {
    if !tracks[i].points.is_empty() {
      i += 1;
      continue;
    }
    tracks.swap_remove(i);
  }
}
