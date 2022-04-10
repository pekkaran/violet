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
}

#[allow(non_snake_case)]
impl KalmanFilter {
  pub fn new() -> KalmanFilter {
    let p = PARAMETER_SET.lock().unwrap();
    let pose_trail_len = p.pose_trail_len;
    let m = DVector::zeros(IND_CAM + CAM_SIZE * pose_trail_len);
    KalmanFilter {
      last_time: None,
      pose_trail_len,
      m,
      P: dmatrix!(),
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

    let w = gyroscope - self.m.fixed_slice::<3, 1>(IND_BGA, 0);
    let S = - 0.5 * dt * Matrix4d::new(
      0., -w[0], -w[1], -w[2],
      w[0], 0., -w[2], w[1],
      w[1], w[2], 0., -w[0],
      w[2], -w[1], w[0], 0.,
    );

    let A = S.exp();
    let (R, dR) = to_rotation_matrix_d(ori!(self.m, 0).into());
  }
}
