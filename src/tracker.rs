use crate::all::*;

// Could this be replaced with frame numbers?
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackerStep(pub usize);

pub struct Tracker {
  detector: Detector,
  optical_flow: OpticalFlow,
  tracks: Vec<Track>,
  max_tracks: usize,
  next_id: TrackId,
  step: TrackerStep,
  // Workspace.
  features0: Vec<Feature>,
  features1: Vec<Feature>,
  features2: Vec<Feature>,
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
      tracks: vec![],
      max_tracks,
      next_id: TrackId(0),
      step: TrackerStep(0),
      features0: vec![],
      features1: vec![],
      features2: vec![],
    })
  }

  pub fn get_tracks(&self) -> &Vec<Track> {
    &self.tracks
  }

  pub fn process(
    &mut self,
    frame0: Option<&Frame>,
    frame1: &Frame,
    cameras: &[Camera],
    frame_number: usize,
  ) {
    if let Some(frame0) = frame0 {
      self.features1.clear();
      for track in &self.tracks {
        if track.last_seen.0 + 1 == self.step.0 {
          self.features1.push(Feature {
            point: track.points.iter().last().unwrap().coordinates[0],
            id: track.id,
          });
        }
      }

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
      update_tracks(
        &mut self.tracks,
        [&self.features1, &self.features2],
        [&cameras[0], &cameras[1]],
        false,
        self.step,
        frame_number,
      );
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
    update_tracks(
      &mut self.tracks,
      [&self.features1, &self.features2],
      [&cameras[0], &cameras[1]],
      true,
      self.step,
      frame_number,
    );

    self.step.0 += 1
  }
}

fn update_tracks(
  tracks: &mut Vec<Track>,
  features: [&[Feature]; 2],
  cameras: [&Camera; 2],
  new_tracks: bool,
  step: TrackerStep,
  frame_number: usize,
) {
  let mut update_track = |features: [Feature; 2]| {
    // Skip tracks for which normalized coordinates cannot be computed, eg those
    // with viewing angle 90 degrees or larger. Later parts of the VIO pipeline
    // may require existence of normalized coordinates so it's easiest to
    // guarantee them already here.
    let normalized_coordinates = [
      hnormalize(cameras[0].model.pixel_to_ray(features[0].point)?)?,
      hnormalize(cameras[1].model.pixel_to_ray(features[1].point)?)?,
    ];
    assert_eq!(features[0].id, features[1].id);
    if new_tracks {
      tracks.push(Track::new(features, normalized_coordinates, step, frame_number));
    }
    else {
      for track in tracks.iter_mut() {
        if track.id == features[0].id {
          track.points.push(TrackPoint {
            coordinates: [features[0].point, features[1].point],
            normalized_coordinates,
            frame_number,
          });
          track.last_seen = step;
          break;
        }
      }
    }
    Some(())
  };

  assert_eq!(features[0].len(), features[1].len());
  for (feature0, feature1) in features[0].iter().zip(features[1].iter()) {
    update_track([*feature0, *feature1]);
  }

  // Remove tracks that could not be tracked, even if some may not have been
  // used for visual updates yet. Fresh tracks are always needed to reliably
  // estimate the current pose.
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
      let p0 = &tracks[i0].points.iter().last().unwrap().coordinates;
      let p1 = &tracks[i1].points.iter().last().unwrap().coordinates;
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
