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
  It: Matrixd,
  // Workspace.
  grid0: Matrixd,
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
      It: DMatrix::zeros(p.lk_win_size, p.lk_win_size),
      grid0: DMatrix::zeros(p.lk_win_size, p.lk_win_size),
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
      if let Some(feature1) = self.process_feature(frame0, frame1, *feature0) {
        features1.push(feature1);
        statuses.push(true);
      }
      else {
        features1.push(*feature0);
        statuses.push(false);
      }
    }
  }

  #[allow(non_snake_case)]
  fn process_feature(
    &mut self,
    frame0: &Frame,
    frame1: &Frame,
    feature0: Vector2d,
  ) -> Option<Vector2d> {
    let r = (self.lk_win_size - 1) / 2;
    let mut g = Vector2d::zeros();
    let mut d = Vector2d::zeros();
    for L in (0..self.lk_levels + 1).rev() {
      let level0 = frame0.get_level(L);
      let level1 = frame1.get_level(L);
      let u = feature0 / u32::pow(2, L as u32) as f64;
      let range = integration_range(&level0, u, r, 1)?;
      scharr(&level0, u, range, &mut self.Ix, &mut self.Iy, &mut self.grid0);
      let G = spatial_gradient(range, &self.Ix, &self.Iy);
      let mut nu = Vector2d::zeros();
      for _ in 0..self.lk_iters {
        image_difference(range, r, &self.grid0, &mut self.It, &level1, u + g + nu)?;
        nu += flow_vector(&G, &self.Ix, &self.Iy, &self.It)?;
        // if nu < self.lk_term { break }
      }
      d = nu;
      if L > 0 { g = 2. * (g + d) }
    }
    Some(feature0 + g + d)
  }
}

#[allow(non_snake_case)]
fn image_difference(
  prev_range: Range,
  r: usize,
  I0: &Matrixd,
  mut It: &mut Matrixd,
  level: &Level,
  center: Vector2d,
) -> Option<()> {
  let range = integration_range(level, center, r, 0)?;
  // TODO The new range can be larger, should reduce it.
  // TODO If the new range is smaller, could recompute G (refer to the PDF).
  if range != prev_range {
    return None;
  }
  fill_grid(level, range, center, &mut It);
  *It *= -1.;
  *It += I0.slice((1, 1), (It.nrows(), It.ncols()));
  Some(())
}

#[allow(non_snake_case)]
fn flow_vector(
  G: &Matrix2d,
  Ix: &Matrixd,
  Iy: &Matrixd,
  It: &Matrixd,
) -> Option<Vector2d> {
  let mut b = Vector2d::zeros();
  for y in 0..Ix.nrows() {
    for x in 0..Ix.ncols() {
      b[0] += It[(y, x)] * Ix[(y, x)];
      b[1] += It[(y, x)] * Iy[(y, x)];
    }
  }
  // Could instead solve the linear equation?
  G.try_inverse().map(|invG| invG * b)
}

#[allow(non_snake_case)]
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
// the image borders. Returns None if the center point is outside the level
// boundaries.
fn integration_range(
  level: &Level,
  center: Vector2d,
  r: usize,
  padding: i16,
) -> Option<Range> {
  let r = r as i16;
  let mut range = [[0, 0], [0, 0]];
  for i in 0..2 {
    let s = if i == 0 { level.width } else { level.height };
    if center[i] < 0. || center[i] > (s - 1) as f64 { return None; }
    let n = center[i] as i16;
    // Should check if the -2 is really correct.
    range[i] = [i16::max(-r, -n + padding), i16::min(r, s as i16 - n - padding - 2)];
  }
  Some(range)
}

fn fill_grid(
  level: &Level,
  range: Range,
  center: Vector2d,
  grid: &mut Matrixd,
) {
  *grid = DMatrix::zeros((range[1][1] - range[1][0] + 1) as usize, (range[0][1] - range[0][0] + 1) as usize);
  for (y_ind, y) in (range[1][0]..=range[1][1]).enumerate() {
    for (x_ind, x) in (range[0][0]..=range[0][1]).enumerate() {
      grid[(y_ind, x_ind)] = bilinear(level, center + Vector2d::new(x as f64, y as f64));
    }
  }
}

fn scharr(
  level: &Level,
  center: Vector2d,
  range: Range,
  out_x: &mut Matrixd,
  out_y: &mut Matrixd,
  // Workspace.
  mut grid: &mut Matrixd,
) {
  let grange = [[range[0][0] - 1, range[0][1] + 1], [range[1][0] - 1, range[1][1] + 1]];
  fill_grid(level, grange, center, &mut grid);
  // TODO Unclear if these kind of statements cause allocations.
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
fn bilinear(frame: &Level, u: Vector2d) -> f64 {
  assert!(u[0] >= 0.0 && u[0] <= frame.width as f64 - 1.);
  assert!(u[1] >= 0.0 && u[1] <= frame.height as f64 - 1.);
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
    frame.data[y0 * frame.width + x0] as f64
  }
  else if xa < eps {
    (1. - ya) * frame.data[y0 * frame.width + x0] as f64
      + ya * frame.data[y1 * frame.width + x0] as f64
  }
  else if ya < eps {
    (1. - xa) * frame.data[y0 * frame.width + x0] as f64
      + xa * frame.data[y0 * frame.width + x1] as f64
  }
  else {
    (1. - xa) * (1. - ya) * frame.data[y0 * frame.width + x0] as f64
      + xa * (1. - ya) * frame.data[y0 * frame.width + x1] as f64
      + (1. - xa) * ya * frame.data[y1 * frame.width + x0] as f64
      + xa * ya * frame.data[y1 * frame.width + x1] as f64
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_scharr() {
    let mut frame = Frame {
      data: vec![
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
      ],
      width: 5,
      height: 5,
      pyramid: Pyramid {
        levels: vec![],
        size: [5, 5],
      },
    };
    let level = frame.get_level(0);

    let mut out_x = dmatrix!();
    let mut out_y = dmatrix!();
    let mut grid = dmatrix!();
    let center = Vector2d::new(2.0, 2.0);
    let range = integration_range(&level, center, 1, 1).unwrap();
    scharr(&level, center, range, &mut out_x, &mut out_y, &mut grid);
    assert_eq!(out_x, DMatrix::zeros(3, 3));
    assert_eq!(out_y, DMatrix::zeros(3, 3));

    frame.data = vec![
      0, 1, 2, 3, 4,
      0, 1, 2, 3, 4,
      0, 1, 2, 3, 4,
      0, 1, 2, 3, 4,
      0, 1, 2, 3, 4,
    ];
    let level = frame.get_level(0);
    scharr(&level, center, range, &mut out_x, &mut out_y, &mut grid);
    assert_eq!(out_x, DMatrix::repeat(3, 3, 1.));
    assert_eq!(out_y, DMatrix::zeros(3, 3));

    frame.data = vec![
      0, 1, 2, 3, 4,
      1, 2, 3, 4, 5,
      2, 3, 4, 5, 6,
      3, 4, 5, 6, 7,
      4, 5, 6, 7, 8,
    ];
    let level = frame.get_level(0);
    scharr(&level, center, range, &mut out_x, &mut out_y, &mut grid);
    assert_eq!(out_x, DMatrix::repeat(3, 3, 1.));
    assert_eq!(out_y, DMatrix::repeat(3, 3, 1.));

    frame.data = vec![
      0, 0, 5, 0, 0,
      0, 0, 5, 0, 0,
      0, 0, 5, 0, 0,
      0, 0, 5, 0, 0,
      0, 0, 5, 0, 0,
    ];
    let level = frame.get_level(0);
    scharr(&level, center, range, &mut out_x, &mut out_y, &mut grid);
    let answer_x = dmatrix!(
      2.5, 0., -2.5;
      2.5, 0., -2.5;
      2.5, 0., -2.5;
    );
    assert_eq!(out_x, answer_x);
    assert_eq!(out_y, DMatrix::zeros(3, 3));
  }

  // #[test]
  // fn quick() {
  //   assert_eq!(5.4 as i32, 5);
  //   assert_eq!(-2.4_f32.floor(), 5.0);
  // }

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
    let level = frame.get_level(0);
    assert_eq!(integration_range(&level, Vector2d::new(4.5, 4.5), 3, 0).unwrap(), [[-3, 3], [-3, 3]]);
    assert_eq!(integration_range(&level, Vector2d::new(1.5, 2.5), 3, 0).unwrap(), [[-1, 3], [-2, 3]]);
    assert_eq!(integration_range(&level, Vector2d::new(1.0, 2.0), 3, 0).unwrap(), [[-1, 3], [-2, 3]]);
    assert_eq!(integration_range(&level, Vector2d::new(0.9, 1.9), 3, 0).unwrap(), [[0, 3], [-1, 3]]);
    assert_eq!(integration_range(&level, Vector2d::new(0.9, 1.9), 3, 1).unwrap(), [[1, 3], [0, 3]]);
    assert_eq!(integration_range(&level, Vector2d::new(8.5, 2.0), 3, 0).unwrap(), [[-3, 0], [-2, 3]]);
    assert_eq!(integration_range(&level, Vector2d::new(9.5, 2.0), 3, 0).unwrap(), [[-3, -1], [-2, 3]]);
    assert_eq!(integration_range(&level, Vector2d::new(9.5, 8.5), 3, 0).unwrap(), [[-3, -1], [-3, 0]]);
    assert_eq!(integration_range(&level, Vector2d::new(9.5, 8.5), 3, 1).unwrap(), [[-3, -2], [-3, -1]]);
  }
}
