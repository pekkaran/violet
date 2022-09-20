use crate::all::*;

pub struct VisualUpdate {
  pose_trail_len: usize,
  tmp: Tmp,
}

struct Tmp {
  kalman_filter_poses: Vec<[KalmanFilterPose; 2]>,
  indices: Vec<usize>,
  normalized_coordinates: Vec<[Vector2d; 2]>,
  triangulate_output: TriangulateOutput,
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
        kalman_filter_poses: vec![],
        indices: vec![],
        normalized_coordinates: vec![],
        triangulate_output: TriangulateOutput {
          a: Vector3d::zeros(),
          da_dp: vec![],
          da_dq: vec![],
        },
      },
    }
  }

  pub fn process(
    &mut self,
    kalman_filter: &mut KalmanFilter,
    tracks: &[Track],
    cameras: [&Camera; 2],
    pose_trail_frame_numbers: &VecDeque<usize>,
  ) {
    for track in tracks {
      self.tmp.indices.clear();
      self.tmp.normalized_coordinates.clear();
      let mut i = 0;
      for point in &track.points {
        if point.frame_number < pose_trail_frame_numbers[i] { continue }
        while i < pose_trail_frame_numbers.len() && pose_trail_frame_numbers[i] < point.frame_number {
          i += 1;
        }
        if pose_trail_frame_numbers[i] == point.frame_number {
          // In the Kalman Filter state the newest pose is first: reverse indices.
          self.tmp.indices.push(self.pose_trail_len - i - 1);
          self.tmp.normalized_coordinates.push(point.normalized_coordinates);
        }
        if i >= pose_trail_frame_numbers.len() { break }
      }

      // TODO Crashes.
      // let success = kalman_filter.get_camera_pose_trail(
      //   &self.tmp.indices,
      //   cameras,
      //   &mut self.tmp.kalman_filter_poses,
      // );
      let success = false;

      // TODO The assert shouldn't fail, the above logic probably doesn't
      // correctly check if KF poses are usable.
      if !success { continue }
      assert!(success);

      if triangulate(
        &self.tmp.normalized_coordinates,
        &self.tmp.kalman_filter_poses,
        &mut self.tmp.triangulate_output,
      ).is_none() {
        continue;
      }

      // TODO Do the visual update.
    }
  }
}

struct TriangulateOutput {
  // Triangulated position in world coordinates.
  a: Vector3d,
  // Triangulated position differentiated wrt camera positions.
  da_dp: Vec<Matrix3d>,
  // Triangulated position differentiated wrt camera orientations.
  da_dq: Vec<Matrix34d>,
}

// Algorithm from the book Computer Vision: Algorithms and Applications
// by Richard Szeliski. Chapter 7.1 Triangulation, page 345.
//
// There are many algorithms for triangulation using N cameras. This one is
// probably the simplest to differentiate wrt to all the pose variables.
// However, it has a particular weakness in that it ignores the fact that the
// fixed transformation between the stereo cameras is known, and instead treats
// all the camera rays as equal. Using this triangulation function may degrade
// quality of the visual updates considerably.
//
// NOTE This function is heavily based on the HybVIO implementation here:
//   <https://github.com/SpectacularAI/HybVIO/blob/main/src/odometry/triangulation.cpp>
//   (see `triangulateLinear()`)
fn triangulate(
  normalized_coordinates: &[[Vector2d; 2]],
  kalman_filter_poses: &[[KalmanFilterPose; 2]],
  output: &mut TriangulateOutput,
) -> Option<()> {
  // TODO This function has not been tested at all.
  output.a = Vector3d::zeros();
  output.da_dp.clear();
  output.da_dq.clear();

  // Triangulation function.
  assert_eq!(normalized_coordinates.len(), kalman_filter_poses.len());
  let mut S = Matrix3d::zeros();
  let mut t = Vector3d::zeros();
  for i in 0..normalized_coordinates.len() {
    for j in 0..2 {
      let pose = &kalman_filter_poses[i][j];
      let ip = &normalized_coordinates[i][j];
      let ip = Vector3d::new(ip[0], ip[1], 0.);
      let vn = pose.R.transpose() * ip.normalize();
      let A = Matrix3d::identity() - vn * vn.transpose();
      S += A;
      t += A * pose.p;
    }
  }
  let inv_S = S.try_inverse()?;
  output.a = inv_S * t;

  // Derivatives of the triangulation function.
  for i in 0..normalized_coordinates.len() {
    for j in 0..2 {
      let pose = &kalman_filter_poses[i][j];
      let ip = &normalized_coordinates[i][j];
      let ip = Vector3d::new(ip[0], ip[1], 0.);
      let v = pose.R.transpose() * ip;
      let vn = v.normalize();
      let A = Matrix3d::identity() - vn * vn.transpose();
      output.da_dp.push(inv_S * A);

      // Derivative of v wrt q.
      let mut dv_dq = Matrix34d::zeros();
      for k in 0..4 {
        dv_dq.column_mut(k).copy_from(&(pose.dR_dq[k].transpose() * ip));
      }

      // Derivative of normalized v wrt v.
      let n = v.norm();
      let dvn_dv = A / n;

      // Derivative of p wrt normalized v.
      let mut da_dvn = Matrix3d::zeros();
      for k in 0..3 {
        // Derivatives of v*v' wrt to v. Since first is 3x3 and second 3x1,
        // differentiate wrt to individual components of v.
        let mut ek = Vector3d::zeros();
        ek[k] = 1.;
        let Q = ek * vn.transpose() + vn * ek.transpose();
        da_dvn.column_mut(k).copy_from(&(inv_S * Q * inv_S * t - inv_S * Q * pose.p));
      }

      output.da_dq.push(da_dvn * dvn_dv * dv_dq);
    }
  }

  Some(())
}
