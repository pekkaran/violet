// Pinhole mode with OpenCV distortion model using only the radial components.
//
// NOTE This code is heavily based on the HybVIO implementation here:
//   <https://github.com/SpectacularAI/HybVIO/blob/main/src/tracker/camera.cpp>

use crate::all::*;

const UNDISTORT_MAX_ITERATIONS: usize = 20;
const UNDISTORT_CONVERGENCE_THRESHOLD: f64 = 1e-5;

pub struct PinholeModel {
  pub camera_matrix: Matrix3d,
  pub camera_matrix_inv: Matrix3d,
  pub distortion_coefficients: Vec<f64>,
}

impl PinholeModel {
  pub fn new(
    camera_matrix: Matrix3d,
    distortion_coefficients: Vec<f64>,
  ) -> PinholeModel {
    PinholeModel {
      camera_matrix,
      camera_matrix_inv: camera_matrix.try_inverse().unwrap(),
      distortion_coefficients,
    }
  }

  fn distort(&self, p: Vector2d, compute_derivative: bool) -> (Vector2d, Option<Matrix2d>) {
    if self.distortion_coefficients.is_empty() {
      return (p, if compute_derivative { Some(Matrix2d::identity()) } else { None });
    }
    let c = &self.distortion_coefficients;
    assert_eq!(c.len(), 3);
    let x = p[0];
    let y = p[1];
    let r2 = x * x + y * y;
    let theta = 1. + r2 * (c[0] + r2 * (c[1] + r2 * c[2]));
    let dtheta = c[0] + r2 * (c[1] * 2. + r2 * c[2] * 3.);
    let dp = if compute_derivative {
      Some(Matrix2d::new(
        theta + x * dtheta * 2. * x, x * dtheta * 2. * y,
        y * dtheta * 2. * x, theta + y * dtheta * 2. * y
      ))
    }
    else {
      None
    };
    (Vector2d::new(x * theta, y * theta), dp)
  }

  fn undistort(&self, dist: Vector2d) -> Vector2d {
    if self.distortion_coefficients.is_empty() { return dist }
    let mut point = dist;
    for _ in 0..UNDISTORT_MAX_ITERATIONS {
      let (p, dp) = self.distort(point, true);
      let delta = dp.unwrap().try_inverse().unwrap() * (dist - p);
      point += delta;
      if delta.norm() < UNDISTORT_CONVERGENCE_THRESHOLD { break }
    }
    point
  }
}

impl CameraModel for PinholeModel {
  fn pixel_to_ray(&self, pixel: Vector2d) -> Option<Vector3d> {
    let dist = Vector2d::new(
      (pixel[0] - self.camera_matrix[(0, 2)]) / self.camera_matrix[(0, 0)],
      (pixel[1] - self.camera_matrix[(1, 2)]) / self.camera_matrix[(1, 1)],
    );
    let p = self.undistort(dist);
    Some(Vector3d::new(p[0], p[1], 1.).normalize())
  }

  fn ray_to_pixel_d(&self, ray: Vector3d, compute_derivative: bool)
    -> (Option<Vector2d>, Option<Matrix23d>)
  {
    if ray[2] <= 0. { return (None, None) }
    let iz = 1. / ray[2];
    let (dist, ddist) = self.distort(iz * Vector2d::new(ray[0], ray[1]), compute_derivative);
    let p = Vector3d::new(dist[0], dist[1], 1.);
    let pixel = self.camera_matrix * p;

    let dpixel = if compute_derivative {
      let dhomog = matrix!(
        iz, 0., -ray[0] * iz * iz;
        0., iz, -ray[1] * iz * iz;
      );
      Some(self.camera_matrix.fixed_slice::<2, 2>(0, 0) * ddist.unwrap() * dhomog)
    }
    else {
      None
    };
    (Some(Vector2d::new(pixel[0], pixel[1])), dpixel)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_pinhole() {
    let K = Matrix3d::new(
      1000., 0., 360.,
      0., 1000., 640.,
      0., 0., 1.,
    );
    let camera = PinholeModel::new(K, vec![]);
    let ray0 = Vector3d::new(-0.25, 0.11, 2.).normalize();
    let pixel = camera.ray_to_pixel(ray0).unwrap();
    assert!((pixel - Vector2d::new(235., 695.)).norm() < 1e-6);
    let ray = camera.pixel_to_ray(pixel).unwrap();
    assert!((ray - ray0).norm() < 1e-10);

    let K = Matrix3d::new(
      458., 0., 367.215,
      0., 458., 248.375,
      0., 0., 1.,
    );
    let camera = PinholeModel::new(K, vec![-0.28340811, 0.07395907, 0.00019359]);
    let pixel = camera.ray_to_pixel(ray0).unwrap();
    assert!((pixel - Vector2d::new(310.26612557476517, 273.4325047471033)).norm() < 1e-6);
    let ray = camera.pixel_to_ray(pixel).unwrap();
    assert!((ray - ray0).norm() < 1e-10);
  }
}
