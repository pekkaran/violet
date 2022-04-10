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
  #[cfg(test)]
  pub fn set_value(&mut self, x: usize, y: usize, value: u8) {
    self.data[y * self.width + x] = value;
  }

  #[cfg(test)]
  pub fn set_sub_image(&mut self, ax: usize, ay: usize, image: &Image) {
    for y in 0..image.height {
      for x in 0..image.width {
        self.set_value(ax + x, ay + y, image.value(x, y));
      }
    }
  }

  #[cfg(test)]
  pub fn set_sub_image_i32(&mut self, ax: i32, ay: i32, image: &Image) {
    self.set_sub_image(ax as usize, ay as usize, image)
  }

  #[cfg(test)]
  #[allow(dead_code)]
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

#[inline(always)]
pub fn bilinear(image: &Image, u: Vector2d) -> f64 {
  // dbg!(u);
  assert!(u[0] >= 0.0 && u[0] <= image.width as f64 - 1.);
  assert!(u[1] >= 0.0 && u[1] <= image.height as f64 - 1.);
  let x0 = u[0] as usize;
  let y0 = u[1] as usize;
  let x1 = x0 + 1;
  let y1 = y0 + 1;
  let xa = u[0].fract();
  let ya = u[1].fract();
  // Besides improving computation speed, these allow to work one pixel
  // closer to the right and bottom edges when coordinates are integers.
  let eps = 1e-5;
  if xa < eps && ya < eps {
    image.data[y0 * image.width + x0] as f64
  }
  else if xa < eps {
    (1. - ya) * image.data[y0 * image.width + x0] as f64
      + ya * image.data[y1 * image.width + x0] as f64
  }
  else if ya < eps {
    (1. - xa) * image.data[y0 * image.width + x0] as f64
      + xa * image.data[y0 * image.width + x1] as f64
  }
  else {
    (1. - xa) * (1. - ya) * image.data[y0 * image.width + x0] as f64
      + xa * (1. - ya) * image.data[y0 * image.width + x1] as f64
      + (1. - xa) * ya * image.data[y1 * image.width + x0] as f64
      + xa * ya * image.data[y1 * image.width + x1] as f64
  }
}
