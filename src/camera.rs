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

  fn ray_to_pixel_d(&self, ray: Vector3d, compute_derivative: bool)
    -> (Option<Vector2d>, Option<Matrix23d>);

  fn ray_to_pixel(&self, ray: Vector3d) -> Option<Vector2d> {
    self.ray_to_pixel_d(ray, false).0
  }
}
