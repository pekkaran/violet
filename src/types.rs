// Eigen-like aliases.
pub type Vector2d = nalgebra::Vector2::<f64>;
pub type Vector3d = nalgebra::Vector3::<f64>;
pub type Vector4d = nalgebra::Vector4::<f64>;
pub type Vector2i = nalgebra::Vector2::<i32>;
pub type Vector2usize = nalgebra::Vector2::<usize>;
pub type Vectord = nalgebra::DVector::<f64>;
pub type Matrixd = nalgebra::DMatrix::<f64>;
pub type Matrix2d = nalgebra::Matrix2::<f64>;
pub type Matrix3d = nalgebra::Matrix3::<f64>;
pub type Matrix4d = nalgebra::Matrix4::<f64>;
pub type Matrix23d = nalgebra::Matrix2x3::<f64>;

pub fn from_usize(p: Vector2usize) -> Vector2i {
  Vector2i::new(p[0] as i32, p[1] as i32)
}

pub fn from_f64(p: Vector2d) -> Vector2i {
  Vector2i::new(p[0].round() as i32, p[1].round() as i32)
}
