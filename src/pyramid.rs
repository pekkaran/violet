use crate::all::*;

const NAIVE_DOWNSCALE: bool = false;

pub struct Pyramid {
  pub levels: Vec<Image>,
  // Size of the parent Image. Needed?
  pub size: [usize; 2],
}

impl Pyramid {
  pub fn empty() -> Pyramid {
    Pyramid {
      levels: vec![],
      size: [0, 0],
    }
  }

  pub fn compute(
    // Unused pyramid or a new one from `Pyramid::empty()`.
    pyramid: &mut Pyramid,
    video_frame: &Image,
    level_count: usize,
  ) -> Result<()> {
    pyramid.levels = compute_levels(
      video_frame,
      mem::take(&mut pyramid.levels),
      level_count,
    )?;
    pyramid.size = [video_frame.width, video_frame.height];
    Ok(())
  }
}

fn compute_levels(
  video_frame: &Image,
  mut levels: Vec<Image>,
  level_count: usize,
) -> Result<Vec<Image>> {
  while levels.len() < level_count {
    levels.push(Image::empty());
  }
  if level_count == 0 { return Ok(levels) }
  downscale(&video_frame, &mut levels[0])?;
  for i in 0..(level_count - 1) {
    let rest = &mut levels[i..];
    // Need to use a split function to get a mutable and non-mutable reference
    // to different elements of the vector.
    if let Some((parent, rest)) = rest.split_first_mut() {
      downscale(&parent, &mut rest[0])?;
    }
  }
  Ok(levels)
}

fn downscale(
  parent: &Image,
  child: &mut Image
) -> Result<()> {
  let w = parent.width as i32;
  let h = parent.height as i32;
  if w % 2 != 0 || h % 2 != 0 {
    bail!("Cannot downscale image with dimensions {}x{}", w, h);
  }
  let w2 = w / 2;
  let h2 = h / 2;
  child.data.clear();
  child.width = w2 as usize;
  child.height = h2 as usize;

  let v = |mut x: i32, mut y: i32| -> u16 {
    if x < 0 { x = 0 }
    if y < 0 { y = 0 }
    if x >= w { x = w }
    if y >= h { y = h }
    parent.data[((y * w) + x) as usize] as u16
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
      child.data.push(value as u8);
    }
  }
  Ok(())
}
