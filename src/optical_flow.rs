// Pyramidal Lucas-Kanade tracker based on:
// <http://robots.stanford.edu/cs223b04/algo_tracking.pdf>
// “Pyramidal Implementation of the Lucas Kanade Feature Tracker
//   Description of the algorithm” by Jean-Yves Bouguet

use crate::all::*;

type Range = [[i16; 2]; 2];

#[allow(non_snake_case)]
pub struct OpticalFlow {
  lk_iters: usize,
  lk_levels: usize,
  lk_win_size: usize,
  Ix: Matrixd,
  Iy: Matrixd,
  // Workspace.
  grid: Matrixd,
}

impl OpticalFlow {
  pub fn new() -> Result<OpticalFlow> {
    let p = &*PARAMETER_SET.lock().unwrap();
    if p.lk_win_size % 2 != 1 {
      bail!("Lucas-Kanade window size must be odd number.");
    }
    if p.lk_win_size < 3 {
      bail!("Lucas-Kanade window size must be at least 3.");
    }
    Ok(OpticalFlow {
      lk_iters: p.lk_iters,
      lk_levels: p.lk_levels,
      lk_win_size: p.lk_win_size,
      Ix: DMatrix::zeros(p.lk_win_size, p.lk_win_size),
      Iy: DMatrix::zeros(p.lk_win_size, p.lk_win_size),
      grid: DMatrix::zeros(p.lk_win_size, p.lk_win_size),
    })
  }

  pub fn process(
    &mut self,
    frame0: &Frame,
    frame1: &Frame,
    features0: &[Vector2d],
    features1: &mut Vec<Vector2d>,
    statuses: &mut Vec<bool>,
  ) {
    features1.clear();
    for feature0 in features0 {
      let (feature1, status) = self.process_feature(frame0, frame1, *feature0);
      features1.push(feature1);
      statuses.push(status);
    }
  }

  #[allow(non_snake_case)]
  fn process_feature(
    &mut self,
    frame0: &Frame,
    _frame1: &Frame,
    feature0: Vector2d,
  ) -> (Vector2d, bool) {
    let r = (self.lk_win_size - 1) / 2;
    let mut g = Vector2d::zeros();
    let mut d = Vector2d::zeros();
    for L in (0..self.lk_levels + 1).rev() {
      let u = feature0 / u32::pow(2, L as u32) as f64;
      let range = integration_range(frame0, u, r, 1);
      scharr(frame0, u, range, &mut self.Ix, &mut self.Iy, &mut self.grid);
      let mut G = spatial_gradient(range, &self.Ix, &self.Iy);
      let mut nu = Vector2d::zeros();
      for _ in 0..self.lk_iters {
        // Compute new range based on g and nu. If the range has become smaller,
        //   recompute G.
        // let It = image_difference(range, grid, frame1);
        // let b = image_mismatch();
        // let eta = G.inverse() * b;
        let eta = Vector2d::zeros(); // TODO Remove.
        nu += eta;
        // if nu < self.lk_term { break }
      }
      d = nu;
      if L > 0 { g = 2. * (g + d) }
    }
    (feature0 + g + d, true)
  }
}

fn spatial_gradient(
  // For now assuming range is the same as around the source feature.
  _range: Range,
  Ix: &Matrixd,
  Iy: &Matrixd,
) -> Matrix2d {
  assert_eq!(Ix.nrows(), Iy.nrows());
  assert_eq!(Ix.ncols(), Iy.ncols());
  let mut x2 = 0.;
  let mut y2 = 0.;
  let mut xy = 0.;
  for y in 0..Ix.nrows() {
    for x in 0..Ix.ncols() {
      x2 += Ix[(y, x)] * Ix[(y, x)];
      y2 += Iy[(y, x)] * Iy[(y, x)];
      xy += Ix[(y, x)] * Iy[(y, x)];
    }
  }
  Matrix2d::new(x2, xy, xy, y2)
}

// Returns closed range of integer steps that can be takes without going outside
// the image borders.
fn integration_range(
  frame: &Frame,
  u: Vector2d,
  r: usize,
  padding: i16,
) -> Range {
  let r = r as i16;
  let mut range = [[0, 0], [0, 0]];
  for i in 0..2 {
    let n = u[i] as i16;
    let s = if i == 0 { frame.width } else { frame.height };
    range[i] = [i16::max(-r, -n + padding), i16::min(r, s as i16 - n - padding - 2)];
  }
  range
}

fn scharr(
  frame: &Frame,
  u: Vector2d,
  range: Range,
  out_x: &mut Matrixd,
  out_y: &mut Matrixd,
  // Workspace.
  grid: &mut Matrixd,
) {
  let grange = [[range[0][0] - 1, range[0][1] + 1], [range[1][0] - 1, range[1][1] + 1]];
  // TODO Unclear if these kind of statements cause allocations.
  *grid = DMatrix::zeros((grange[1][1] - grange[1][0] + 1) as usize, (grange[0][1] - grange[0][0] + 1) as usize);
  for (y_ind, y) in (grange[1][0]..=grange[1][1]).enumerate() {
    for (x_ind, x) in (grange[0][0]..=grange[0][1]).enumerate() {
      grid[(y_ind, x_ind)] = bilinear(frame, u + Vector2d::new(x as f64, y as f64));
    }
  }
  *out_x = Matrixd::zeros(grid.nrows() - 2, grid.ncols() - 2);
  *out_y = Matrixd::zeros(grid.nrows() - 2, grid.ncols() - 2);
  for y in 1..(grid.nrows() - 1) {
    for x in 1..(grid.ncols() - 1) {
      out_x[(y - 1, x - 1)] = (10. * grid[(y, x + 1)]
        + 3. * grid[(y + 1, x + 1)]
        + 3. * grid[(y - 1, x + 1)]
        - 10. * grid[(y, x - 1)]
        - 3. * grid[(y + 1, x - 1)]
        - 3. * grid[(y - 1, x - 1)]
      ) / 32.;
      out_y[(y - 1, x - 1)] = (10. * grid[(y + 1, x)]
        + 3. * grid[(y + 1, x + 1)]
        + 3. * grid[(y + 1, x - 1)]
        - 10. * grid[(y - 1, x)]
        - 3. * grid[(y - 1, x + 1)]
        - 3. * grid[(y - 1, x - 1)]
      ) / 32.;
    }
  }
}

#[inline(always)]
fn bilinear(frame: &Frame, u: Vector2d) -> f64 {
  assert!(u[0] >= 0.0 && u[0] <= frame.width as f64 - 1.);
  assert!(u[1] >= 0.0 && u[1] <= frame.height as f64 - 1.);
  let x0 = u[0] as usize;
  let y0 = u[1] as usize;
  let x1 = x0 + 1;
  let y1 = y0 + 1;
  let xa = u[0].fract();
  let ya = u[1].fract();
  (1. - xa) * (1. - ya) * frame.data[y0 * frame.width + x0] as f64
    + xa * (1. - ya) * frame.data[y0 * frame.width + x1] as f64
    + (1. - xa) * ya * frame.data[y1 * frame.width + x0] as f64
    + xa * ya * frame.data[y1 * frame.width + x1] as f64
}


#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_integration_range() {
    // Width and height are pixels. Coordinate (0, 0) means center of top-left
    // pixel. Thus (9, 9) is the center of the bottom-right pixel for 10x10
    // image.
    let frame = Frame {
      data: vec![],
      width: 10,
      height: 10,
      pyramid: Pyramid {
        levels: vec![],
        size: [10, 10],
      },
    };
    assert_eq!(integration_range(&frame, Vector2d::new(4.5, 4.5), 3, 0), [[-3, 3], [-3, 3]]);
    assert_eq!(integration_range(&frame, Vector2d::new(1.5, 2.5), 3, 0), [[-1, 3], [-2, 3]]);
    assert_eq!(integration_range(&frame, Vector2d::new(1.0, 2.0), 3, 0), [[-1, 3], [-2, 3]]);
    assert_eq!(integration_range(&frame, Vector2d::new(0.9, 1.9), 3, 0), [[0, 3], [-1, 3]]);
    assert_eq!(integration_range(&frame, Vector2d::new(0.9, 1.9), 3, 1), [[1, 3], [0, 3]]);
    assert_eq!(integration_range(&frame, Vector2d::new(8.5, 2.0), 3, 0), [[-3, 0], [-2, 3]]);
    assert_eq!(integration_range(&frame, Vector2d::new(9.5, 2.0), 3, 0), [[-3, -1], [-2, 3]]);
    assert_eq!(integration_range(&frame, Vector2d::new(9.5, 8.5), 3, 0), [[-3, -1], [-3, 0]]);
    assert_eq!(integration_range(&frame, Vector2d::new(9.5, 8.5), 3, 1), [[-3, -2], [-3, -1]]);
  }
}
