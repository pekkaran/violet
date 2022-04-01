use crate::all::*;

const NAIVE_DOWNSCALE: bool = false;

pub struct Pyramid {
  pub levels: Vec<Vec<u8>>,
  pub size: [usize; 2],
}

impl Pyramid {
  pub fn new(
    video_frame: &VideoFrame,
    unused_pyramid: Option<Pyramid>,
    level_count: usize,
  ) -> Result<Pyramid> {
    Ok(Pyramid {
      levels: compute_levels(
        video_frame,
        unused_pyramid.map(|x| x.levels),
        level_count,
      )?,
      size: [video_frame.width, video_frame.height],
    })
  }

  // Argument 0 gives size of the original non-down scaled image.
  pub fn size(&self, level: usize) -> [usize; 2] {
    let n = usize::pow(2, level as u32);
    let w = self.size[0] / n;
    let h = self.size[1] / n;
    if w * n != self.size[0] || h * n != self.size[1] {
      warn!("Pyramid level size is not exact.");
    }
    [w, h]
  }
}

fn compute_levels(
  video_frame: &VideoFrame,
  unused_levels: Option<Vec<Vec<u8>>>,
  level_count: usize,
) -> Result<Vec<Vec<u8>>> {
  let mut levels = unused_levels.unwrap_or(vec![vec![]; level_count]);
  if level_count == 0 { return Ok(levels) }
  let mut width = video_frame.width;
  let mut height = video_frame.height;
  downscale(&video_frame.data, &mut levels[0], width, height)?;
  width /= 2;
  height /= 2;
  for i in 0..(level_count - 1) {
    let rest = &mut levels[i..];
    // Need to use a split function to get a mutable and non-mutable reference
    // to different elements of the vector.
    if let Some((parent, rest)) = rest.split_first_mut() {
      downscale(&parent, &mut rest[0], width, height)?;
      width /= 2;
      height /= 2;
    }
  }
  Ok(levels)
}

fn downscale(
  parent: &[u8],
  child: &mut Vec<u8>,
  w: usize,
  h: usize,
) -> Result<()> {
  assert_eq!(parent.len(), w * h);
  if w % 2 != 0 || h % 2 != 0 {
    bail!("Cannot downscale image with dimensions {}x{}", w, h);
  }
  let w = w as i32;
  let h = h as i32;
  let w2 = w / 2;
  let h2 = h / 2;
  child.clear();

  let v = |mut x: i32, mut y: i32| -> u16 {
    if x < 0 { x = 0 }
    if y < 0 { y = 0 }
    if x >= w { x = w }
    if y >= h { y = h }
    parent[((y * w) + x) as usize] as u16
  };

  for y in 0..h2 {
    let y2 = 2 * y;
    for x in 0..w2 {
      let x2 = 2 * x;
      let value = if NAIVE_DOWNSCALE {
        (v(x2, y2) + v(x2 + 1, y2) + v(x2, y2 + 1) + v(x2 + 1, y2 + 1) + 2) / 4
      }
      else {
        // Low-pass filter and possibly more accurate coordinate indexing(?).
        v(x2, y2) / 4
          + (v(x2 + 1, y2) + v(x2 - 1, y2) + v(x2, y2 + 1) + v(x2, y2 - 1)) / 8
          + (v(x2 + 1, y2 + 1) + v(x2 - 1, y2 - 1) + v(x2 - 1, y2 + 1) + v(x2 + 1, y2 - 1)) / 16
      };
      child.push(value as u8);
    }
  }
  Ok(())
}
