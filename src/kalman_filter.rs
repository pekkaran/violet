// Heavily based on:
//   <https://github.com/SpectacularAI/HybVIO/blob/main/src/odometry/ekf.cpp>

use crate::all::*;

const IND_VEL: usize = 0;
const IND_BGA: usize = 3;
const IND_BAA: usize = 6;
const IND_CAM: usize = 9;

const CAM_SIZE: usize = 7;
const CAM_POS: usize = 0;
const CAM_ORI: usize = 3;

const IND_POS0: usize = IND_CAM;
const IND_ORI0: usize = IND_CAM + CAM_ORI;

macro_rules! vel { ($m: expr) => { $m.fixed_slice::<3, 1>(IND_VEL, 0) } }
macro_rules! bga { ($m: expr) => { $m.fixed_slice::<3, 1>(IND_BGA, 0) } }
macro_rules! baa { ($m: expr) => { $m.fixed_slice::<3, 1>(IND_BAA, 0) } }
macro_rules! pos { ($m: expr, $ind: expr) => {
  $m.fixed_slice::<3, 1>(IND_CAM + CAM_POS + $ind * CAM_SIZE, 0)
} }
macro_rules! ori { ($m: expr, $ind: expr) => {
  $m.fixed_slice::<4, 1>(IND_CAM + CAM_ORI + $ind * CAM_SIZE, 0)
} }

#[allow(non_snake_case)]
pub struct KalmanFilter {
  last_time: Option<f64>,
  pose_trail_len: usize,
  m: Vectord,
  P: Matrixd,
  F: Matrixd,
  gravity: Vector3d,
}

#[allow(non_snake_case)]
impl KalmanFilter {
  pub fn new() -> KalmanFilter {
    let p = PARAMETER_SET.lock().unwrap();
    let pose_trail_len = p.pose_trail_len;
    let state_len = IND_CAM + CAM_SIZE * pose_trail_len;
    let m = DVector::zeros(state_len);

    let mut F = DMatrix::zeros(IND_CAM, IND_CAM);
    F.fixed_slice_mut::<3, 3>(IND_POS0, IND_POS0).copy_from(&Matrix3d::identity());
    F.fixed_slice_mut::<3, 3>(IND_VEL, IND_VEL).copy_from(&Matrix3d::identity());
    F.fixed_slice_mut::<3, 3>(IND_BGA, IND_BGA).copy_from(&Matrix3d::identity());
    F.fixed_slice_mut::<3, 3>(IND_BAA, IND_BAA).copy_from(&Matrix3d::identity());

    KalmanFilter {
      last_time: None,
      pose_trail_len,
      m,
      P: DMatrix::zeros(state_len, state_len),
      F,
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

    let w = gyroscope - bga!(self.m);
    let S = - 0.5 * dt * Matrix4d::new(
      0., -w[0], -w[1], -w[2],
      w[0], 0., -w[2], w[1],
      w[1], w[2], 0., -w[0],
      w[2], -w[1], w[0], 0.,
    );

    let A = S.exp();
    let last_q = ori!(self.m, 0);
    let (R, dR) = to_rotation_matrix_d(last_q.into());

    let pos_new = pos!(self.m, 0) + vel!(self.m) * dt;
    self.m.fixed_slice_mut::<3, 1>(IND_POS0, 0).copy_from(&pos_new);

    let a = accelerometer - baa!(self.m);
    let vel_new = vel!(self.m) + (R.transpose() * a + self.gravity) * dt;
    self.m.fixed_slice_mut::<3, 1>(IND_VEL, 0).copy_from(&vel_new);

    let ori_new = A * ori!(self.m, 0);
    self.m.fixed_slice_mut::<4, 1>(IND_ORI0, 0).copy_from(&ori_new);

    self.F.fixed_slice_mut::<3, 3>(IND_POS0, IND_VEL).copy_from(&(dt * Matrix3d::identity()));

    // TODO Many more equations.
  }
}
