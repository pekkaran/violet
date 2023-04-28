use crate::all::*;

use rand::seq::SliceRandom;
use rand::SeedableRng;

pub struct VisualUpdate {
  kf_noise_visual: f64,
  rng: Xoshiro256PlusPlus,
  tmp: Tmp,
}

struct Tmp {
  kalman_filter_poses: Vec<[KalmanFilterPose; 2]>,
  indices: Vec<usize>,
  normalized_coordinates: Vec<[Vector2d; 2]>,
  triangulate_output: TriangulateOutput,
  // EKF measurement function Jacobian.
  H: Matrixd,
  // Stacked measured features minus stacked triangulated and reprojected features.
  // `z - h(x)` in EKF update.
  y: Vectord,
}

impl VisualUpdate {
  pub fn new() -> VisualUpdate {
    let p = PARAMETER_SET.lock().unwrap();
    VisualUpdate {
      kf_noise_visual: p.kf_noise_visual,
      rng: Xoshiro256PlusPlus::seed_from_u64(0),
      tmp: Tmp {
        kalman_filter_poses: vec![],
        indices: vec![],
        normalized_coordinates: vec![],
        triangulate_output: TriangulateOutput {
          a: Vector3d::zeros(),
          da_dp: vec![],
          da_dq: vec![],
        },
        H: Matrixd::zeros(0, 0),
        y: Vectord::zeros(0),
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
    {
      let d = &mut DEBUG_DATA_3D.lock().unwrap();
      d.triangulation_positions.clear();
    }

    let mut successful_update_count = 0;

    'track:
    for track in tracks.choose_multiple(&mut self.rng, 50) {
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
          self.tmp.indices.push(pose_trail_frame_numbers.len() - i - 1);
          self.tmp.normalized_coordinates.push(point.normalized_coordinates);
        }
        if i >= pose_trail_frame_numbers.len() { break }
      }

      let success = kalman_filter.get_camera_pose_trail(
        &self.tmp.indices,
        cameras,
        &mut self.tmp.kalman_filter_poses,
      );
      assert!(success);

      if triangulate(
        &self.tmp.normalized_coordinates,
        &self.tmp.kalman_filter_poses,
        &mut self.tmp.triangulate_output,
      ).is_none() {
        continue;
      }

      // The visual update is defined by the measurement function `h()`
      // operating on the EKF state `x` as:
      //   h_i(x) = hnormalize(pose_i.R * (aw - pose_i.p)),
      // where
      //   aw = triangulate(x)
      // is given in world coordinates.
      //
      // As part of the triangulation we have computed all derivatives of `aw`
      // and it remains to differentiate `h_i(x)` for all poses k. The result
      // depends on if i == k.
      //
      // As an example, to compute the position derivatives:
      // d_{k_p}h_i(x) = d_hnormalize * [
      //   d_{k_p}(pose_i.R) * (aw - pose_i.p)
      //   + pose_i.R * d_{k_p}(aw - pose_i.p)
      // ]
      // = d_hnormalized * pose_i.R * d_{k_p}(aw - pose_i.p)
      let n = self.tmp.kalman_filter_poses.len();
      self.tmp.H.resize_mut(4 * n, kalman_filter.get_state_len(), 0.);
      self.tmp.y.resize_vertically_mut(4 * n, 0.);
      let aw = self.tmp.triangulate_output.a;

      {
        let d = &mut DEBUG_DATA_3D.lock().unwrap();
        d.triangulation_positions.push(aw);
      }

      for i in 0..n {
        for j in 0..2 {
          let row = 2 * (2 * i + j);
          let pose = &self.tmp.kalman_filter_poses[i][j];
          // let wcp = position!(world_to_camera); // TODO This is not needed, right?
          // We decompose this for clarity with the derivatives but it's the same as:
          //   let ac = affine_transform(world_to_camera, aw); // TODO Verify.
          let ac = pose.R * (aw - pose.p);

          // Check the triangulated point is in front of all cameras.
          if ac[2] <= 0. { continue 'track; }

          // Compute normalized coordinates ("project" the triangulated point).
          let normalized_ac = hnormalize(ac).unwrap();
          let mut d_normalized_ac = Matrix23d::zeros();
          d_normalized_ac[(0, 0)] = 1. / ac[2];
          d_normalized_ac[(1, 1)] = 1. / ac[2];
          d_normalized_ac[(0, 2)] = -ac[0] / ac[2];
          d_normalized_ac[(1, 2)] = -ac[1] / ac[2];

          self.tmp.y[row + 0] = self.tmp.normalized_coordinates[i][j][0] - normalized_ac[0];
          self.tmp.y[row + 1] = self.tmp.normalized_coordinates[i][j][1] - normalized_ac[1];

          // This is the contribution to the derivatives in the case
          // i == k. Using again the position as example and ignoring the `aw` term:
          //   d_{i_p}h_i(x) = d_hnormalized * pose_i.R * d_{i_p}(aw - pose_i.p)
          //   -> d_hormalized * pose_i.R * (-I)
          let col_pos = kalman_filter.get_camera_pos_ind(i);
          let col_ori = kalman_filter.get_camera_ori_ind(i);
          self.tmp.H.fixed_slice_mut::<2, 3>(row, col_pos).copy_from(&(-d_normalized_ac * pose.R));
          for m in 0..4 {
            self.tmp.H.fixed_slice_mut::<2, 1>(row, col_ori + m).copy_from(&(
              d_normalized_ac * pose.dR_dq[m] * (aw - pose.p)
            ));
          }

          for k in 0..n {
            let col_pos_k = kalman_filter.get_camera_pos_ind(k);
            let col_ori_k = kalman_filter.get_camera_ori_ind(k);
            let da_dp = &self.tmp.triangulate_output.da_dp;
            let da_dq = &self.tmp.triangulate_output.da_dq;
            let mut pos = self.tmp.H.fixed_slice_mut::<2, 3>(row, col_pos_k);
            pos += d_normalized_ac * pose.R * da_dp[k];
            let mut ori = self.tmp.H.fixed_slice_mut::<2, 4>(row, col_ori_k);
            ori += d_normalized_ac * pose.R * da_dq[k];
          }
        } // for j in 0..2
      } // for i in 0..n

      kalman_filter.update_visual(
        &self.tmp.H,
        &self.tmp.y,
        self.kf_noise_visual,
      );

      successful_update_count += 1;
      if successful_update_count >= 5 { break }
    } // process()
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
      let ip = Vector3d::new(ip[0], ip[1], 1.);
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
      let ip = Vector3d::new(ip[0], ip[1], 1.);
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
