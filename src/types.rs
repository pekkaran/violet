// use crate::all::*;

// Eigen-like aliases.
pub type Vector3d = nalgebra::Vector3::<f64>;
pub type Vector2i = nalgebra::Vector2::<i32>;
pub type Vector2usize = nalgebra::Vector2::<usize>;

pub fn from_usize(p: &Vector2usize) -> Vector2i {
  Vector2i::new(p[0] as i32, p[1] as i32)
}
