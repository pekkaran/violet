use crate::all::*;

// The order of quaternion entries in a Vector4d is: w, x, y, z.

// Affine transformation matrices.
macro_rules! rotation { ($x: expr) => { $x.fixed_slice::<3, 3>(0, 0) } }
macro_rules! position { ($x: expr) => { $x.fixed_slice::<3, 1>(0, 3) } }

#[allow(dead_code)]
pub fn affine_transform(M: Matrix4d, p: Vector3d) -> Vector3d {
  rotation!(M) * p + position!(M)
}

pub fn affine_inverse(M: Matrix4d) -> Matrix4d {
  // TODO Use the faster way to compute with transpose.
  M.try_inverse().unwrap()
}

pub fn hnormalize(p: Vector3d) -> Option<Vector2d> {
  if p[2] < 0. { return None }
  Some(Vector2d::new(p[0] / p[2], p[1] / p[2]))
}

pub fn to_rotation_matrix(q: Vector4d) -> Matrix3d {
  Matrix3d::new(
    q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3], 2.*q[1]*q[2] - 2.*q[0]*q[3], 2.*q[1]*q[3] + 2.*q[0]*q[2],
    2.*q[1]*q[2] + 2.*q[0]*q[3], q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3], 2.*q[2]*q[3] - 2.*q[0]*q[1],
    2.*q[1]*q[3] - 2.*q[0]*q[2], 2.*q[2]*q[3] + 2.*q[0]*q[1], q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3],
    )
}

// Differentiation of the quaternion->rotation matrix conversion.
pub struct QuaternionAsRotation {
  pub R: Matrix3d,
  pub dR_dq: [Matrix3d; 4],
}

pub fn to_rotation_matrix_d(q: Vector4d) -> QuaternionAsRotation {
  QuaternionAsRotation {
    R: Matrix3d::new(
      q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3], 2.*q[1]*q[2] - 2.*q[0]*q[3], 2.*q[1]*q[3] + 2.*q[0]*q[2],
      2.*q[1]*q[2] + 2.*q[0]*q[3], q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3], 2.*q[2]*q[3] - 2.*q[0]*q[1],
      2.*q[1]*q[3] - 2.*q[0]*q[2], 2.*q[2]*q[3] + 2.*q[0]*q[1], q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3],
    ),
    dR_dq: [
      Matrix3d::new(
        2. * q[0], -2. * q[3],  2. * q[2],
        2. * q[3],  2. * q[0], -2. * q[1],
        -2. * q[2],  2. * q[1],  2. * q[0],
      ),
      Matrix3d::new(
        2. * q[1],  2. * q[2],  2. * q[3],
        2. * q[2], -2. * q[1], -2. * q[0],
        2. * q[3],  2. * q[0], -2. * q[1],
      ),
      Matrix3d::new(
        -2. * q[2],  2. * q[1],  2. * q[0],
        2. * q[1],  2. * q[2],  2. * q[3],
        -2. * q[0],  2. * q[3], -2. * q[2],
      ),
      Matrix3d::new(
        -2. * q[3], -2. * q[0],  2. * q[1],
        2. * q[0], -2. * q[3],  2. * q[2],
        2. * q[1],  2. * q[2],  2. * q[3],
      ),
    ],
  }
}

pub fn transform_3d(T: &Matrix4d, x: &Vector3d) -> Vector3d {
  T.fixed_slice::<3, 3>(0, 0) * x + T.fixed_slice::<3, 1>(0, 3)
}
