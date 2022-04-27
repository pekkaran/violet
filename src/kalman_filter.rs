// Heavily based on:
//   <https://github.com/SpectacularAI/HybVIO/blob/main/src/odometry/ekf.cpp>
//
// Some differences:
// * Changed some notation to better match <https://en.wikipedia.org/wiki/Extended_Kalman_filter>
//   * Specifically, the predict that operates only on a part of the state: P = F*P*F' + L*Q*L'
// * Moved the first pose next to rest of the trail in the KF state.
// * No multiplicative accelerometer bias.

use crate::all::*;

// Repeated poses forming the pose trail, starting from `CAM0`.
const CAM_POS: usize = 0; // Position [x, y, z].
const CAM_ORI: usize = 3; // Orientation [w, x, y, z].
const CAM_SIZE: usize = 7;

// The part of the state used in prediction, includes first pose of the trail.
const F_VEL: usize = 0; // Velocity.
const F_BGA: usize = 3; // [B]ias [G]yroscope [A]dditive.
const F_BAA: usize = 6; // [B]ias [A]ccelerometer [A]dditive.
const CAM0: usize = 9; // Start of camera poses.
const F_POS: usize = CAM0;
const F_ORI: usize = CAM0 + CAM_ORI;
const F_SIZE: usize = CAM0 + CAM_SIZE;

// Prediction noise.
const Q_A: usize = 0; // Accelerometer.
const Q_G: usize = 3; // Gyroscope.
const Q_BGA: usize = 6; // BGA drift.
#[allow(dead_code)] // TODO Implement.
const Q_BAA: usize = 9; // BAA drift.
const Q_SIZE: usize = 12;

// Note that these create references. To clone use eg:
//   let x: VectorN = ori!().into();
macro_rules! vel { ($m: expr) => { $m.fixed_slice::<3, 1>(F_VEL, 0) } }
macro_rules! bga { ($m: expr) => { $m.fixed_slice::<3, 1>(F_BGA, 0) } }
macro_rules! baa { ($m: expr) => { $m.fixed_slice::<3, 1>(F_BAA, 0) } }
macro_rules! pos { ($m: expr, $ind: expr) => {
  $m.fixed_slice::<3, 1>(CAM0 + CAM_POS + $ind * CAM_SIZE, 0)
} }
macro_rules! ori { ($m: expr, $ind: expr) => {
  $m.fixed_slice::<4, 1>(CAM0 + CAM_ORI + $ind * CAM_SIZE, 0)
} }

#[allow(non_snake_case)]
pub struct KalmanFilter {
  last_time: Option<f64>,
  pose_trail_len: usize,
  state_len: usize,
  gravity: Vector3d,

  // The notation is mostly based on <https://en.wikipedia.org/wiki/Extended_Kalman_filter>
  // State mean. `x` in the Wikipedia article. TODO Change to `x`.
  m: Vectord,
  // State covariance.
  P: Matrixd,
  // Process noise covariance.
  Q: Matrixd,
  // State transition Jacobian.
  F: Matrixd,
  // Process noise Jacobian.
  L: Matrixd,
  // Pose augmentation.
  aug_F: Matrixd,
  aug_Q: Matrixd,
  aug_R: Matrixd,
  aug_H: Matrixd,
  aug_S: Matrixd,
  aug_S_inv: Matrixd,
  // Temporary variables to avoid allocations. It's a bit unclear if these work
  // in nalgebra like in Eigen.
  tmp_m: Vectord,
  tmp_P: Matrixd,
}

#[allow(non_snake_case)]
impl KalmanFilter {
  pub fn new() -> KalmanFilter {
    let p = PARAMETER_SET.lock().unwrap();
    let pose_trail_len = p.pose_trail_len;
    let state_len = CAM0 + CAM_SIZE * pose_trail_len;

    let mut F = DMatrix::zeros(F_SIZE, F_SIZE);
    F.fixed_slice_mut::<3, 3>(F_POS, F_POS).copy_from(&Matrix3d::identity());
    F.fixed_slice_mut::<3, 3>(F_VEL, F_VEL).copy_from(&Matrix3d::identity());
    F.fixed_slice_mut::<3, 3>(F_BGA, F_BGA).copy_from(&Matrix3d::identity());
    F.fixed_slice_mut::<3, 3>(F_BAA, F_BAA).copy_from(&Matrix3d::identity());

    let mut L = DMatrix::zeros(F_SIZE, Q_SIZE);
    L.fixed_slice_mut::<3, 3>(F_BGA, Q_BGA).copy_from(&Matrix3d::identity());
    L.fixed_slice_mut::<3, 3>(F_BAA, Q_BGA).copy_from(&Matrix3d::identity());

    let mut Q = DMatrix::zeros(Q_SIZE, Q_SIZE);
    Q.fixed_slice_mut::<3, 3>(Q_A, Q_A).copy_from(&(p.kf_noise_a.powi(2) * Matrix3d::identity()));
    Q.fixed_slice_mut::<3, 3>(Q_G, Q_G).copy_from(&(p.kf_noise_g.powi(2) * Matrix3d::identity()));

    let mut aug_F = DMatrix::zeros(state_len, state_len);
    aug_F.fixed_slice_mut::<F_SIZE, F_SIZE>(0, 0).copy_from(&(DMatrix::identity(F_SIZE, F_SIZE)));

    let mut aug_Q = DMatrix::zeros(state_len, state_len);
    // VIO should not be sensitive to these, hard-coded.
    let aug_process_noise_position = 100.;
    let aug_process_noise_orientation = 3.;
    let CAM1 = CAM0 + CAM_SIZE;
    for i in 0..3 {
      aug_Q[(CAM1 + i, CAM1 + i)] = aug_process_noise_position;
    }
    for i in 0..4 {
      aug_Q[(CAM1 + 3 + i, CAM1 + 3 + i)] = aug_process_noise_orientation;
    }

    let aug_update_noise = 1e-9; // VIO should not be sensitive to this, hard-coded.
    let aug_R = aug_update_noise * DMatrix::identity(CAM_SIZE, CAM_SIZE);

    let mut aug_H = DMatrix::zeros(CAM_SIZE, state_len);
    for i in 0..CAM_SIZE {
      aug_H[(i, CAM0 + i)] = 1.;
      aug_H[(i, CAM0 + CAM_SIZE + i)] = -1.;
    }

    let aug_S = DMatrix::zeros(CAM_SIZE, CAM_SIZE);
    let aug_S_inv = DMatrix::zeros(CAM_SIZE, CAM_SIZE);

    KalmanFilter {
      last_time: None,
      pose_trail_len,
      state_len,
      gravity: Vector3d::new(0., 0., -p.gravity),
      m: DVector::zeros(state_len),
      P: DMatrix::zeros(state_len, state_len),
      Q,
      F,
      L,
      aug_F,
      aug_Q,
      aug_R,
      aug_H,
      aug_S,
      aug_S_inv,
      tmp_m: DVector::zeros(state_len),
      tmp_P: DMatrix::zeros(state_len, state_len),
    }
  }

  pub fn predict(
    &mut self,
    time: f64,
    gyroscope: Vector3d,
    accelerometer: Vector3d,
  ) {
    let dt = if let Some(last_time) = self.last_time { time - last_time } else { 0. };
    self.last_time = Some(time);
    if dt <= 0. { return }

    // TODO Bias random walk.

    let g = gyroscope - bga!(self.m); // Unbiased gyroscope.
    let S = -0.5 * dt * Matrix4d::new(
      0., -g[0], -g[1], -g[2],
      g[0], 0., -g[2], g[1],
      g[1], g[2], 0., -g[0],
      g[2], -g[1], g[0], 0.,
    );

    let A = S.exp();
    let last_q: Vector4d = ori!(self.m, 0).into(); // Clone.
    let (R, dR) = to_rotation_matrix_d(last_q);

    let pos_new = pos!(self.m, 0) + vel!(self.m) * dt;
    self.m.fixed_slice_mut::<3, 1>(F_POS, 0).copy_from(&pos_new);

    let a = accelerometer - baa!(self.m); // Unbiased accelerometer.
    let vel_new = vel!(self.m) + (R.transpose() * a + self.gravity) * dt;
    self.m.fixed_slice_mut::<3, 1>(F_VEL, 0).copy_from(&vel_new);

    let ori_new = A * ori!(self.m, 0);
    self.m.fixed_slice_mut::<4, 1>(F_ORI, 0).copy_from(&ori_new);

    self.F.fixed_slice_mut::<3, 3>(F_POS, F_VEL).copy_from(&(dt * Matrix3d::identity()));

    let mut Y = Matrix34d::zeros();
    for i in 0..4 {
      Y.fixed_slice_mut::<3, 1>(0, i).copy_from(&(dt * dR[i].transpose() * a));
    }
    self.F.fixed_slice_mut::<3, 4>(F_VEL, F_ORI).copy_from(&(Y * A));

    self.F.fixed_slice_mut::<4, 4>(F_ORI, F_ORI).copy_from(&A);

    self.L.fixed_slice_mut::<3, 3>(F_VEL, Q_A).copy_from(&(dt * R.transpose()));

    // Might be cleaner to make a constant matrix that is then multiplied by `dt` on each call.
    let dS = [
      Matrix4d::new(
        0., dt / 2., 0., 0.,
        -dt / 2., 0., 0., 0.,
        0., 0., 0., dt / 2.,
        0., 0., -dt / 2., 0.,
      ),
      Matrix4d::new(
        0., 0., dt / 2., 0.,
        0., 0., 0., -dt / 2.,
        -dt / 2., 0., 0., 0.,
        0., dt / 2., 0., 0.,
      ),
      Matrix4d::new(
        0., 0., 0., dt / 2.,
        0., 0., dt / 2., 0.,
        0., -dt / 2., 0., 0.,
        -dt / 2., 0., 0., 0.,
      ),
    ];

    for i in 0..3 {
      self.L.fixed_slice_mut::<4, 1>(F_ORI, Q_G + i).copy_from(&(A * dS[i] * last_q));
    }

    let Z = self.F.fixed_slice::<3, 4>(F_VEL, F_ORI) * self.L.fixed_slice::<4, 3>(F_ORI, Q_G);
    self.L.fixed_slice_mut::<3, 3>(F_VEL, Q_G).copy_from(&Z);

    let Z = -self.L.fixed_slice::<3, 3>(F_VEL, Q_G);
    self.F.fixed_slice_mut::<3, 3>(F_VEL, F_BGA).copy_from(&Z);

    let Z = -self.L.fixed_slice::<4, 3>(F_ORI, Q_G);
    self.F.fixed_slice_mut::<4, 3>(F_ORI, F_BGA).copy_from(&Z);

    self.F.fixed_slice_mut::<3, 3>(F_VEL, F_BAA).copy_from(&(-dt * R.transpose()));

    // P = F*P*F' + L*Q*L'
    self.tmp_P.fixed_slice_mut::<F_SIZE, F_SIZE>(0, 0)
      .copy_from(&(
          &self.F * self.P.fixed_slice::<F_SIZE, F_SIZE>(0, 0) * &self.F.transpose()
          + &self.L * &self.Q * &self.L.transpose()
      ));
    let n = self.state_len;
    self.tmp_P.slice_mut((F_SIZE, 0), (n - F_SIZE, F_SIZE))
      .copy_from(&(self.P.slice_mut((F_SIZE, 0), (n - F_SIZE, F_SIZE)) * &self.F.transpose()));
    self.tmp_P.slice_mut((0, F_SIZE), (F_SIZE, n - F_SIZE))
      .copy_from(&(&self.F * self.P.slice_mut((0, F_SIZE), (F_SIZE, n - F_SIZE))));
    mem::swap(&mut self.P, &mut self.tmp_P);

    self.normalize_quaternions();
  }

  pub fn augment_pose(&mut self) {
    self.tmp_m.copy_from(&(&self.aug_F * &self.m));
    mem::swap(&mut self.m, &mut self.tmp_m);

    self.tmp_P.copy_from(&(&self.aug_F * &self.P * &self.aug_F.transpose() + &self.aug_Q));
    mem::swap(&mut self.P, &mut self.tmp_P);

    // `aug_H * P` will be computed twice.
    self.aug_S.copy_from(&(&self.aug_R + &self.aug_H * &self.P * &self.aug_H.transpose()));
    self.aug_S_inv.copy_from(&(&self.aug_S.clone().try_inverse().unwrap()));

    // TODO Implement.
    /*
    K = invS.solve(HP).transpose();
    Eigen::MatrixXd &v = tmp_P0;
    v.noalias() = -visAugH * m;
    m.noalias() += K * v; // m += K * (-visAugH * m)

    updateCommonJosephForm(P, visAugH, R, K, tmp_P0, tmp_P1);
    */

    // TODO Check "maintainPositiveSemiDefinite()" is not needed.
    self.normalize_quaternions();
  }

  fn normalize_quaternions(&mut self) {
    for i in 0..self.pose_trail_len {
      let ori_new = ori!(self.m, i).normalize();
      self.m.fixed_slice_mut::<4, 1>(CAM0 + CAM_ORI + i * CAM_SIZE, 0).copy_from(&ori_new);
    }
  }
}
