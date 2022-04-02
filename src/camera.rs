use crate::all::*;

// Intrinsic and extrinsic camera parameters for a single camera.
pub struct Camera {
  pub imu_to_camera: Matrix4d,
  pub kind: CameraKind,
  pub model: Box<dyn CameraModel>,
}

#[derive(Debug)]
pub enum CameraKind {
  Pinhole,
  KannalaBrandt4,
}

pub trait CameraModel {
  fn pixel_to_ray(&self, pixel: Vector2d) -> Option<Vector3d>;
  fn ray_to_pixel(&self, ray: Vector3d) -> Option<Vector2d>;
}

pub struct PinholeModel {
  pub camera_matrix: Matrix3d,
  pub distortion_coefficients: Vec<f64>,
}

impl PinholeModel {
  pub fn new(
    camera_matrix: Matrix3d,
    distortion_coefficients: Vec<f64>,
  ) -> PinholeModel {
    PinholeModel {
      camera_matrix,
      distortion_coefficients,
    }
  }
}

impl CameraModel for PinholeModel {
  fn pixel_to_ray(&self, _pixel: Vector2d) -> Option<Vector3d> {
    unimplemented!();
  }

  fn ray_to_pixel(&self, _ray: Vector3d) -> Option<Vector2d> {
    unimplemented!();
  }
}
