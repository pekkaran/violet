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
  m: Vectord,
  P: Matrixd,
  Q: Matrixd,
  tmpP: Matrixd,
  F: Matrixd,
  L: Matrixd,
  gravity: Vector3d,
}

#[allow(non_snake_case)]
impl KalmanFilter {
  pub fn new() -> KalmanFilter {
    let p = PARAMETER_SET.lock().unwrap();
    let pose_trail_len = p.pose_trail_len;
    let state_len = CAM0 + CAM_SIZE * pose_trail_len;
    let m = DVector::zeros(state_len);

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

    KalmanFilter {
      last_time: None,
      pose_trail_len,
      m,
      P: DMatrix::zeros(state_len, state_len),
      tmpP: DMatrix::zeros(state_len, state_len),
      Q,
      F,
      L,
      gravity: Vector3d::new(0., 0., -p.gravity),
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

    self.tmpP.fixed_slice_mut::<F_SIZE, F_SIZE>(0, 0)
      .copy_from(&(
          &self.F * self.P.fixed_slice::<F_SIZE, F_SIZE>(0, 0) * &self.F.transpose()
          + &self.L * &self.Q * &self.L.transpose()
      ));
    // TODO Missing the two side blocks.

    mem::swap(&mut self.P, &mut self.tmpP);

    // TODO Normalize the quaternions somewhere.
  }
}
