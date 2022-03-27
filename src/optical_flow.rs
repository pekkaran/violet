// Pyramidal Lucas-Kanade tracker based on:
// <http://robots.stanford.edu/cs223b04/algo_tracking.pdf>
// “Pyramidal Implementation of the Lucas Kanade Feature Tracker
//   Description of the algorithm” by Jean-Yves Bouguet

use crate::all::*;

pub struct OpticalFlow {
  lk_iters: usize,
  lk_levels: usize,
  lk_win_size: usize,
  Ix: Vectord,
  Iy: Vectord,
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
      Ix: dvector![],
      Iy: dvector![],
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
    let mut _g = Vector2d::zeros();
    for L in (0..self.lk_levels + 1).rev() {
      let u = feature0 / u32::pow(2, L as u32) as f64;
      scharr(frame0, u, r, 0, &mut self.Ix, &mut self.grid);
      scharr(frame0, u, r, 1, &mut self.Iy, &mut self.grid);
    }

    (feature0, true) // TODO
  }
}

fn scharr(
  frame: &Frame,
  u: Vector2d,
  r: usize,
  dim: usize,
  out: &mut Vectord,
  grid: &mut Matrixd,
) {
  // TODO Should r be one larger for the kernel?
  for y in 0..(2 * r + 1) {
    for x in 0..(2 * r + 1) {
      let r = r as f64;
      grid[(y, x)] = bilinear(frame, u + Vector2d::new(u[0] - r + x as f64, u[1] - r + y as f64));
    }
  }
  // TODO Compute Scharr on the grid. Remember normalization (1/32?).
}

#[inline(always)]
fn bilinear(frame: &Frame, u: Vector2d) -> f64 {
  // TODO cap coordinates outside image boundaries? The PDF maybe suggested to
  // instead omit values from the sums, but make sure such sums are compatible.
  0. // TODO
}
