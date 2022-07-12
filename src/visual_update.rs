use crate::all::*;

pub struct VisualUpdate {
  pose_trail_len: usize,
  tmp: Tmp,
}

struct Tmp {
  camera_to_worlds: Vec<Matrix4d>,
  indices: Vec<usize>,
  // features: Vec<>,
}

impl VisualUpdate {
  pub fn new() -> VisualUpdate {
    let pose_trail_len = {
      let p = PARAMETER_SET.lock().unwrap();
      p.pose_trail_len
    };
    VisualUpdate {
      pose_trail_len,
      tmp: Tmp {
        camera_to_worlds: vec![],
        indices: vec![],
      },
    }
  }

  pub fn process(
    &mut self,
    kalman_filter: &mut KalmanFilter,
    tracks: &[Track],
    _cameras: &[Camera],
    pose_trail_frame_numbers: &VecDeque<usize>,
  ) {
    for track in tracks {
      self.tmp.indices.clear();
      let mut i = 0;
      for point in &track.points {
        if point.frame_number < pose_trail_frame_numbers[i] { continue }
        while i < pose_trail_frame_numbers.len() && pose_trail_frame_numbers[i] < point.frame_number {
          i += 1;
        }
        if pose_trail_frame_numbers[i] == point.frame_number {
          // In the Kalman Filter state the newest pose is first: reverse indices.
          self.tmp.indices.push(self.pose_trail_len - i - 1);
          // TODO Normalize and push the stereo 2d coordinates into a tmp vector.
        }
        if i >= pose_trail_frame_numbers.len() { break }
      }

      kalman_filter.get_pose_trail(&self.tmp.indices, &mut self.tmp.camera_to_worlds);

      // TODO Do triangulation.

      // TODO Do the visual update.
    }
  }
}
