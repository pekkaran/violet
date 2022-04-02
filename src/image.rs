use crate::all::*;

// Row-major grayscale image storage.
// Could also have used nalebgra::DMatrix, but the added complexity
// only seems to help with algorithms like the optical flow.
#[derive(Clone)]
pub struct Image {
  pub data: Vec<u8>,
  pub width: usize,
  pub height: usize,
}

impl Image {
  pub fn empty() -> Image {
    Image {
      data: vec![],
      width: 0,
      height: 0,
    }
  }

  pub fn clear(&mut self) {
    self.data.clear();
    self.width = 0;
    self.height = 0;
  }

  pub fn size(&self, dim: usize) -> usize {
    if dim == 0 { self.width } else { self.height }
  }

  #[inline(always)]
  #[allow(dead_code)]
  pub fn value(&self, x: usize, y: usize) -> u8 {
    self.data[y * self.width + x]
  }

  #[inline(always)]
  pub fn value_i32(&self, x: i32, y: i32) -> u8 {
    self.data[y as usize * self.width + x as usize]
  }
}

// Element access in the manner of `image[y][x]`.
impl Index<usize> for Image {
  type Output = [u8];
  fn index(&self, y: usize) -> &Self::Output {
    &self.data[y * self.width .. (y + 1) * self.width]
  }
}
