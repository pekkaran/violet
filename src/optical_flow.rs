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
      let range = integration_range(frame0, u, r, 1);
      scharr(frame0, u, range, 0, &mut self.Ix, &mut self.grid);
      scharr(frame0, u, range, 1, &mut self.Iy, &mut self.grid);
    }

    (feature0, true) // TODO
  }
}

// Returns closed range of integer steps that can be takes without going outside
// the image borders.
fn integration_range(
  frame: &Frame,
  u: Vector2d,
  r: usize,
  padding: i16,
) -> [[i16; 2]; 2] {
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
  range: [[i16; 2]; 2],
  dim: usize,
  out: &mut Vectord,
  grid: &mut Matrixd,
) {
  *grid = DMatrix::zeros((range[1][1] - range[1][0] + 1) as usize, (range[0][1] - range[0][0] + 1) as usize);
  for (y_ind, y) in (range[1][0]..=range[1][1]).enumerate() {
    for (x_ind, x) in (range[0][0]..=range[0][1]).enumerate() {
      grid[(y_ind, x_ind)] = bilinear(frame, u + Vector2d::new(u[0] + x as f64, u[1] + y as f64));
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
