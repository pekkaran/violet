// use crate::all::*;

// Eigen-like aliases.
pub type Vector3d = nalgebra::Vector3::<f64>;
pub type Pixel = nalgebra::Vector2::<i32>;
pub type PixelUsize = nalgebra::Vector2::<usize>;

pub fn from_usize(p: &PixelUsize) -> Pixel {
  Pixel::new(p[0] as i32, p[1] as i32)
}
