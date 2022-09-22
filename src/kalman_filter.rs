// Extentend Kalman Filter (EKF).
//
// The notation is mostly based on <https://en.wikipedia.org/wiki/Extended_Kalman_filter>
//
// The implementation is heavily based on:
//   <https://github.com/SpectacularAI/HybVIO/blob/main/src/odometry/ekf.cpp>
//
// Some differences to HybVIO:
// * Moved the first pose next to rest of the trail in the KF state.
// * No multiplicative accelerometer bias.
//
// NOTE I'm finding `nalgebra` quite unergonomic for the computations in this
// source file. I think the issues are mostly rooted on Rust's strictness and
// thus unavoidable in any library. One option to improve the situation would
// be to make extensive use of macros, whose implementations should be readable.

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
macro_rules! vel { ($x: expr) => { $x.fixed_slice::<3, 1>(F_VEL, 0) } }
macro_rules! bga { ($x: expr) => { $x.fixed_slice::<3, 1>(F_BGA, 0) } }
macro_rules! baa { ($x: expr) => { $x.fixed_slice::<3, 1>(F_BAA, 0) } }
macro_rules! pos { ($x: expr, $ind: expr) => {
  $x.fixed_slice::<3, 1>(CAM0 + CAM_POS + $ind * CAM_SIZE, 0)
} }
macro_rules! ori { ($x: expr, $ind: expr) => {
  $x.fixed_slice::<4, 1>(CAM0 + CAM_ORI + $ind * CAM_SIZE, 0)
} }

fn imu_to_world(pos: Vector3d, ori: Vector4d) -> Matrix4d {
  let mut T = Matrix4d::identity();
  // NOTE The Kalman Filter internally stores rotations as world-to-IMU,
  // even though the positions in world coordinates would make IMU-to-world
  // representation more natural.
  T.fixed_slice_mut::<3, 3>(0, 0).copy_from(&to_rotation_matrix(ori).transpose());
  T.fixed_slice_mut::<3, 1>(0, 3).copy_from(&pos);
  T
}

const D_OMEGA: [Matrix4d; 3] = [
  Matrix4d::new(
    0., 0.5, 0., 0.,
    -0.5, 0., 0., 0.,
    0., 0., 0., 0.5,
    0., 0., -0.5, 0.,
  ),
  Matrix4d::new(
    0., 0., 0.5, 0.,
    0., 0., 0., -0.5,
    -0.5, 0., 0., 0.,
    0., 0.5, 0., 0.,
  ),
  Matrix4d::new(
    0., 0., 0., 0.5,
    0., 0., 0.5, 0.,
    0., -0.5, 0., 0.,
    -0.5, 0., 0., 0.,
  ),
];

// Internal pose representation needed for manipulation of the derivatives for EKF
// updates. Inherits the mixed representation.
pub struct KalmanFilterPose {
  // Device-to-world.
  pub p: Vector3d,
  // World-to-device.
  pub R: Matrix3d,
  pub dR_dq: [Matrix3d; 4],
  // For convenience.
  pub camera_to_world: Matrix4d,
}

pub struct KalmanFilter {
  last_time: Option<f64>,
  pose_trail_len: usize,
  state_len: usize,
  gravity: Vector3d,

  predict_count: usize,
  augment_count: usize,

  // State mean.
  x: Vectord,
  // State covariance.
  P: Matrixd,

  // Pose augmentation (constant).
  aug_F: Matrixd,
  // Process noise covariance (constant).
  Q: Matrixd,

  tmp: Tmp,
}

// Reused local variables. Helps avoid heap allocations.
// It's a bit unclear if this works in nalgebra like in Eigen.
struct Tmp {
  // State transition `f` Jacobian.
  F: Matrixd,
  // Observation model `h` Jacobian.
  H: Matrixd,
  // Innovation covariance.
  S: Matrixd,
  // (Optimal) Kalman gain.
  K: Matrixd,
  // Observation noise covariance.
  R: Matrixd,
  y: Vectord,
  // Process noise Jacobian.
  L: Matrixd,

  x: Vectord,
  P: Matrixd,
  HP: Matrixd,
  IKH: Matrixd,
}

impl KalmanFilter {
  pub fn new() -> KalmanFilter {
    let p = PARAMETER_SET.lock().unwrap();
    let pose_trail_len = p.pose_trail_len;
    let state_len = CAM0 + CAM_SIZE * pose_trail_len;

    let set_diagonal = |x: &mut Matrixd, start, len, value: f64| {
      for i in start..(start + len) {
        x[(i, i)] = value.powi(2);
      }
    };

    let mut F = DMatrix::zeros(F_SIZE, F_SIZE);
    F.fixed_slice_mut::<3, 3>(F_POS, F_POS).copy_from(&Matrix3d::identity());
    F.fixed_slice_mut::<3, 3>(F_VEL, F_VEL).copy_from(&Matrix3d::identity());
    F.fixed_slice_mut::<3, 3>(F_BGA, F_BGA).copy_from(&Matrix3d::identity());
    F.fixed_slice_mut::<3, 3>(F_BAA, F_BAA).copy_from(&Matrix3d::identity());

    let mut L = DMatrix::zeros(F_SIZE, Q_SIZE);
    L.fixed_slice_mut::<3, 3>(F_BGA, Q_BGA).copy_from(&Matrix3d::identity());
    L.fixed_slice_mut::<3, 3>(F_BAA, Q_BAA).copy_from(&Matrix3d::identity());

    let mut Q = DMatrix::zeros(Q_SIZE, Q_SIZE);
    set_diagonal(&mut Q, Q_A, 3, p.kf_noise_a);
    set_diagonal(&mut Q, Q_G, 3, p.kf_noise_g);

    let mut aug_F = DMatrix::zeros(state_len, state_len);
    // Do not change state before the second pose in the trail.
    aug_F.fixed_slice_mut::<F_SIZE, F_SIZE>(0, 0).copy_from(&(DMatrix::identity(F_SIZE, F_SIZE)));
    // Shift poses so that first is copied to the second and the last one dropped.
    for i in 0..(CAM_SIZE * (p.pose_trail_len - 1)) {
      aug_F[(F_SIZE + i, CAM0 + i)] = 1.;
    }

    let mut P = DMatrix::zeros(state_len, state_len);
    set_diagonal(&mut P, F_VEL, 3, p.kf_noise_vel);
    set_diagonal(&mut P, F_BGA, 3, p.kf_noise_bga);
    set_diagonal(&mut P, F_BAA, 3, p.kf_noise_baa);
    for i in 0..p.pose_trail_len {
      set_diagonal(&mut P, CAM0 + i * CAM_SIZE + CAM_POS, 3, p.kf_noise_pos);
      set_diagonal(&mut P, CAM0 + i * CAM_SIZE + CAM_ORI, 4, p.kf_noise_ori);
    }

    KalmanFilter {
      last_time: None,
      pose_trail_len,
      state_len,
      gravity: Vector3d::new(0., 0., -p.gravity),
      predict_count: 0,
      augment_count: 0,
      x: DVector::zeros(state_len),
      P,
      Q,
      aug_F,
      tmp: Tmp {
        F,
        H: DMatrix::zeros(0, state_len),
        S: DMatrix::zeros(0, 0),
        K: DMatrix::zeros(0, 0),
        R: DMatrix::zeros(0, 0),
        y: DVector::zeros(0),
        L,
        x: DVector::zeros(state_len),
        P: DMatrix::zeros(state_len, state_len),
        HP: DMatrix::zeros(0, state_len),
        IKH: DMatrix::zeros(state_len, state_len),
      },
    }
  }

  // Prediction step that uses a dynamic model derived from physics with a
  // control model based on the IMU measurements.
  pub fn predict(
    &mut self,
    time: f64,
    gyroscope: Vector3d,
    accelerometer: Vector3d,
  ) {
    let x = &mut self.x;
    if ori!(x, 0) == Vector4d::zeros() {
      // Initialize orientation based on the first accelometer sample.
      // Based on <https://math.stackexchange.com/a/2313401>.
      let u = -self.gravity;
      let v = accelerometer;
      let un = u.norm();
      let vn = v.norm();
      let n = 1. / (u * vn + un * v).norm();
      x[F_ORI] = n * (un * vn + (u.transpose() * v)[(0, 0)]); // w component
      x.fixed_slice_mut::<3, 1>(F_ORI + 1, 0).copy_from(&(n * u.cross(&v))); // xyz components
    }

    let dt = if let Some(last_time) = self.last_time { time - last_time } else { 0. };
    self.last_time = Some(time);
    if dt <= 0. { return }

    // TODO Bias random walk.

    let g = gyroscope - bga!(x); // Unbiased gyroscope.

    let Omega = (-0.5 * dt * Matrix4d::new(
      0., -g[0], -g[1], -g[2],
      g[0], 0., -g[2], g[1],
      g[1], g[2], 0., -g[0],
      g[2], -g[1], g[0], 0.,
    )).exp();
    let last_q: Vector4d = ori!(x, 0).into(); // Clone.

    let q_as_R = to_rotation_matrix_d(last_q);
    let R = q_as_R.R;
    let dR = q_as_R.dR_dq;

    let pos_new = pos!(x, 0) + vel!(x) * dt;
    x.fixed_slice_mut::<3, 1>(F_POS, 0).copy_from(&pos_new);

    let a = accelerometer - baa!(x); // Unbiased accelerometer.
    let vel_new = vel!(x) + (R.transpose() * a + self.gravity) * dt;
    x.fixed_slice_mut::<3, 1>(F_VEL, 0).copy_from(&vel_new);

    let ori_new = Omega * ori!(x, 0);
    x.fixed_slice_mut::<4, 1>(F_ORI, 0).copy_from(&ori_new);

    let F = &mut self.tmp.F;
    F.fixed_slice_mut::<3, 3>(F_POS, F_VEL).copy_from(&(dt * Matrix3d::identity()));

    let mut Y = Matrix34d::zeros();
    for i in 0..4 {
      Y.fixed_slice_mut::<3, 1>(0, i).copy_from(&(dt * dR[i].transpose() * a));
    }
    F.fixed_slice_mut::<3, 4>(F_VEL, F_ORI).copy_from(&(Y * Omega));

    F.fixed_slice_mut::<4, 4>(F_ORI, F_ORI).copy_from(&Omega);

    let L = &mut self.tmp.L;
    L.fixed_slice_mut::<3, 3>(F_VEL, Q_A).copy_from(&(dt * R.transpose()));

    for i in 0..3 {
      L.fixed_slice_mut::<4, 1>(F_ORI, Q_G + i).copy_from(&(dt * Omega * D_OMEGA[i] * last_q));
    }

    let Z = F.fixed_slice::<3, 4>(F_VEL, F_ORI) * L.fixed_slice::<4, 3>(F_ORI, Q_G);
    L.fixed_slice_mut::<3, 3>(F_VEL, Q_G).copy_from(&Z);

    let Z = -L.fixed_slice::<3, 3>(F_VEL, Q_G);
    F.fixed_slice_mut::<3, 3>(F_VEL, F_BGA).copy_from(&Z);

    let Z = -L.fixed_slice::<4, 3>(F_ORI, Q_G);
    F.fixed_slice_mut::<4, 3>(F_ORI, F_BGA).copy_from(&Z);

    F.fixed_slice_mut::<3, 3>(F_VEL, F_BAA).copy_from(&(-dt * R.transpose()));

    // P = F*P*F' + L*Q*L'
    let P = &mut self.P;
    self.tmp.P.fixed_slice_mut::<F_SIZE, F_SIZE>(0, 0)
      .copy_from(&(
          &*F * P.fixed_slice::<F_SIZE, F_SIZE>(0, 0) * &F.transpose()
          + &*L * &self.Q * &L.transpose()
      ));
    let n = self.state_len;
    self.tmp.P.slice_mut((F_SIZE, 0), (n - F_SIZE, F_SIZE))
      .copy_from(&(P.slice_mut((F_SIZE, 0), (n - F_SIZE, F_SIZE)) * &F.transpose()));
    self.tmp.P.slice_mut((0, F_SIZE), (F_SIZE, n - F_SIZE))
      .copy_from(&(&*F * P.slice_mut((0, F_SIZE), (F_SIZE, n - F_SIZE))));
    mem::swap(&mut *P, &mut self.tmp.P);

    self.predict_count += 1;
  }

  fn update(&mut self) {
    update_sub(&mut self.x, &mut self.P, &mut self.tmp);
    self.normalize_quaternions();
    // To help spot errors, not needed otherwise.
    self.tmp.H.resize_mut(0, 0, 0.);
    self.tmp.R.resize_mut(0, 0, 0.);
    self.tmp.y.resize_vertically_mut(0, 0.);
  }

  // Prediction step that shifts the pose trail.
  pub fn augment_pose(&mut self) {
    self.check_nan(); // Periodic check for development use.

    self.tmp.x.copy_from(&(&self.aug_F * &self.x));
    mem::swap(&mut self.x, &mut self.tmp.x);

    self.tmp.P.copy_from(&(&self.aug_F * &self.P * &self.aug_F.transpose()));
    mem::swap(&mut self.P, &mut self.tmp.P);

    // May not be necessary.
    for i in CAM0..(CAM0 + CAM_SIZE) {
      self.P[(i, i)] += 1e-10;
    }

    self.augment_count += 1;
  }

  pub fn update_zero_velocity(&mut self, r: f64) {
    self.tmp.H.resize_mut(3, F_VEL + 3, 0.);
    self.tmp.H.fixed_slice_mut::<3, 3>(0, F_VEL).copy_from(&Matrix3d::identity());
    // TODO Use macro for the resize_mut + fixed_slice_mut calls.
    self.tmp.y.resize_vertically_mut(3, 0.);
    self.tmp.y.fixed_slice_mut::<3, 1>(0, 0).copy_from(&(-vel!(self.x)));
    self.tmp.R.resize_mut(3, 3, 0.);
    self.tmp.R.fixed_slice_mut::<3, 3>(0, 0).copy_from(&(r * Matrix3d::identity()));
    self.update();
  }

  fn normalize_quaternions(&mut self) {
    for i in 0..self.pose_trail_len {
      if ori!(self.x, i) == Vector4d::zeros() { continue }
      let ori_new = ori!(self.x, i).normalize();
      self.x.fixed_slice_mut::<4, 1>(CAM0 + CAM_ORI + i * CAM_SIZE, 0).copy_from(&ori_new);
    }
  }

  pub fn get_imu_to_worlds(&self, indices: &[usize], imu_to_worlds: &mut Vec<Matrix4d>) {
    imu_to_worlds.clear();
    for i in indices {
      let ori = ori!(self.x, i);
      let pos = pos!(self.x, i);
      if ori == Vector4d::zeros() { continue }
      imu_to_worlds.push(imu_to_world(pos.into(), ori.into()));
    }
  }

  pub fn get_camera_pose_trail(
    &self,
    indices: &[usize],
    cameras: [&Camera; 2],
    poses: &mut Vec<[KalmanFilterPose; 2]>,
  ) -> bool {
    let transform = |A: &KalmanFilterPose, q, imu_to_camera| {
      let imu_to_world = imu_to_world(A.p, q);
      let camera_to_world = imu_to_world * affine_inverse(imu_to_camera);
      let world_to_camera = affine_inverse(camera_to_world);
      KalmanFilterPose {
        camera_to_world: camera_to_world,
        p: position!(camera_to_world).into(),
        R: rotation!(world_to_camera).into(),
        dR_dq: A.dR_dq.map(|dR_dqi| rotation!(world_to_camera) * dR_dqi),
      }
    };

    poses.clear();
    for i in indices {
      let q = ori!(self.x, i).into();
      let p = pos!(self.x, i).into();
      if q == Vector4d::zeros() { return false }
      let q_as_R = to_rotation_matrix_d(q);
      let imu_pose = KalmanFilterPose {
        camera_to_world: Matrix4d::zeros(),
        p,
        R: q_as_R.R,
        dR_dq: q_as_R.dR_dq,
      };
      poses.push([
        transform(&imu_pose, q, cameras[0].imu_to_camera),
        transform(&imu_pose, q, cameras[1].imu_to_camera),
      ]);
    }
    true
  }

  #[allow(dead_code)]
  fn check_nan(&self) {
    for i in 0..self.state_len {
      if self.x[i].is_nan() {
        info!("{}: {}", i, self.x[i]);
      }
      assert!(!self.x[i].is_nan());
    }
  }
}

// EKF update. Before calling this, awkwardly, you need to set the matrices H, y, R.
//   H: Jacobian of the observation model `h`.
//   y = z - h(x), where `z` is the observation measurement
//   R: Covariance of the observation noise.
fn update_sub(
  x: &mut Vectord,
  P: &mut Matrixd,
  tmp: &mut Tmp,
) {
  // TODO Take aliases to `tmp`.
  let ny = tmp.y.nrows(); // Measurement size.
  let nh = tmp.H.ncols(); // (Truncated) state size.
  let nx = tmp.P.ncols(); // Full state size.
  assert_eq!(tmp.H.nrows(), ny);
  assert_eq!(tmp.R.shape(), (ny, ny));
  tmp.HP.resize_mut(ny, nx, 0.);
  tmp.HP.copy_from(&(&tmp.H * P.rows(0, nh)));
  tmp.S.resize_mut(ny, ny, 0.);
  tmp.S.copy_from(&(tmp.HP.columns(0, nh) * tmp.H.transpose() + &tmp.R));

  // TODO If this works, make more compact using `new_unchecked()`.
  let inv_S = if let Some(inv_S) = nalgebra::linalg::Cholesky::new(tmp.S.clone()) {
    inv_S
  }
  else {
    // Can `S` be just semi positive definite, in which case we need to use
    // LDLT decomposition instead of LLT?
    warn!("Cholesky decomposition failed.");
    return;
  };

  tmp.K.resize_mut(nx, ny, 0.);
  tmp.K.copy_from(&(inv_S.solve(&tmp.HP).transpose()));

  *x += &tmp.K * &tmp.y;

  // (I - KH)P(I - KH)' + KRK'
  tmp.IKH = Matrixd::zeros(nx, nx);
  tmp.IKH.slice_mut((0, 0), (nx, nh)).copy_from(&(-&tmp.K * &tmp.H));
  for i in 0..nx {
    tmp.IKH[(i, i)] += 1.;
  }
  // TODO needs to be copy_from()?
  tmp.P = &tmp.IKH * &*P * &tmp.IKH.transpose() + &tmp.K * &tmp.R * &tmp.K.transpose();
  mem::swap(&mut *P, &mut tmp.P);
}
