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

  #[inline(always)]
  pub fn set_value(&mut self, x: usize, y: usize, value: u8) {
    self.data[y * self.width + x] = value;
  }

  pub fn set_sub_image(&mut self, ax: usize, ay: usize, image: &Image) {
    for y in 0..image.height {
      for x in 0..image.width {
        self.set_value(ax + x, ay + y, image.value(x, y));
      }
    }
  }

  pub fn set_sub_image_i32(&mut self, ax: i32, ay: i32, image: &Image) {
    self.set_sub_image(ax as usize, ay as usize, image)
  }

  pub fn get_sub_image(
    &self,
    ax: usize,
    ay: usize,
    width: usize,
    height: usize,
  ) -> Image {
    let mut image = Image {
      width,
      height,
      data: vec![0; width * height],
    };
    for y in 0..height {
      for x in 0..width {
        image.set_value(x, y, self.value(ax + x, ay + y));
      }
    }
    image
  }
}

// Element access in the manner of `image[y][x]`.
impl Index<usize> for Image {
  type Output = [u8];
  fn index(&self, y: usize) -> &Self::Output {
    &self.data[y * self.width .. (y + 1) * self.width]
  }
}

impl fmt::Display for Image {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let mut s = String::new();
    for y in 0..self.height {
      for x in 0..self.width {
        s += &format!("{:>3},", self.value(x, y));
      }
      s += "\n";
    }
    write!(f, "{}", s)
  }
}
