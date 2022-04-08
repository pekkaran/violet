// Pyramidal Lucas-Kanade tracker based on:
// <http://robots.stanford.edu/cs223b04/algo_tracking.pdf>
// “Pyramidal Implementation of the Lucas Kanade Feature Tracker
//   Description of the algorithm” by Jean-Yves Bouguet

use crate::all::*;

type Range = [[i16; 2]; 2];

const AVERAGE_DISTANCE_METERS: f64 = 5.;

#[allow(non_snake_case)]
pub struct OpticalFlow {
  lk_iters: usize,
  lk_levels: usize,
  lk_win_size: usize,
  lk_term: f64,
  lk_min_eig: f64,
  Ix: Matrixd,
  Iy: Matrixd,
  It: Matrixd,
  // Workspace.
  grid0: Matrixd,
}

#[derive(Clone, Copy, PartialEq)]
pub enum OpticalFlowKind {
  LeftPreviousToCurrent,
  LeftCurrentToRightCurrent,
  LeftCurrentToRightCurrentDetection,
}

impl OpticalFlow {
  pub fn new() -> Result<OpticalFlow> {
    let (lk_iters, lk_levels, lk_win_size, lk_term, lk_min_eig) = {
      let p = PARAMETER_SET.lock().unwrap();
      (p.lk_iters, p.lk_levels, p.lk_win_size, p.lk_term, p.lk_min_eig)
    };
    Self::new_custom(lk_iters, lk_levels, lk_win_size, lk_term, lk_min_eig)
  }

  pub fn new_custom(
    lk_iters: usize,
    lk_levels: usize,
    lk_win_size: usize,
    lk_term: f64,
    lk_min_eig: f64,
  ) -> Result<OpticalFlow> {
    if lk_win_size % 2 != 1 {
      bail!("Lucas-Kanade window size must be odd number.");
    }
    if lk_win_size < 3 {
      bail!("Lucas-Kanade window size must be at least 3.");
    }
    Ok(OpticalFlow {
      lk_iters,
      lk_levels,
      lk_win_size,
      lk_term,
      lk_min_eig,
      Ix: DMatrix::zeros(lk_win_size, lk_win_size),
      Iy: DMatrix::zeros(lk_win_size, lk_win_size),
      It: DMatrix::zeros(lk_win_size, lk_win_size),
      grid0: DMatrix::zeros(lk_win_size, lk_win_size),
    })
  }

  pub fn process(
    &mut self,
    kind: OpticalFlowKind,
    frame_camera0: &FrameCamera,
    frame_camera1: &FrameCamera,
    cameras: &[&Camera],
    features0_in: &[Feature],
    // `feature0_in` with failed features removed.
    features0: &mut Vec<Feature>,
    // Successfully tracked features, same size as `features0`.
    features1: &mut Vec<Feature>,
  ) {
    features0.clear();
    features1.clear();
    let cam0_to_cam1 = cameras[1].imu_to_camera * cameras[0].imu_to_camera.try_inverse().unwrap();
    for feature0 in features0_in {
      let point1_in = compute_initial_guess(feature0.point, cameras, &cam0_to_cam1);
      if let Some(feature1) = self.process_feature(
        frame_camera0,
        frame_camera1,
        *feature0,
        point1_in,
      ) {
        features1.push(feature1);
        features0.push(*feature0);
      }
    }

    // Epipolar check. Heavily based on:
    //   <https://github.com/SpectacularAI/HybVIO/blob/main/src/tracker/tracker.cpp>
    use OpticalFlowKind::*;
    if kind == LeftCurrentToRightCurrent || kind == LeftCurrentToRightCurrentDetection {
      let d = &mut DEBUG_DATA.lock().unwrap();
      let p = PARAMETER_SET.lock().unwrap();
      if p.show_epipolar { d.epipolar.clear() }
      let curve_point_count = 8;
      let mut curve = vec![];
      for (feature0, feature1) in features0.iter().zip(features1.iter()) {
        curve.clear();
        if let Some(ray) = cameras[0].model.pixel_to_ray(feature0.point) {
          let mut s = 0.5;
          for _ in 0..curve_point_count {
            let r0 = s * ray;
            let r1 = transform_vector3d(&cam0_to_cam1, &r0);
            let r1 = r1.normalize(); // TODO needed?
            if let Some(pixel) = cameras[1].model.ray_to_pixel(r1) {
              curve.push(pixel);
              s *= 2.;
            }
          }
        }
        // TODO Remove tracked features that do not lie on the curve.

        if p.show_epipolar {
          let p1_initial = compute_initial_guess(feature0.point, cameras, &cam0_to_cam1);
          d.epipolar.push(DebugEpipolar {
            p0: feature0.point,
            p1: feature1.point,
            p1_initial,
            curve1: curve.clone(),
          });
        }
      }
    }

    let d = &mut DEBUG_DATA.lock().unwrap();
    let p = PARAMETER_SET.lock().unwrap();
    for x in [
      (p.show_flow0, LeftPreviousToCurrent),
      (p.show_flow1, LeftCurrentToRightCurrent),
      (p.show_flow2, LeftCurrentToRightCurrentDetection),
    ] {
      if !x.0 || kind != x.1 { continue }
      d.flow0.clear();
      d.flow0.extend(features0.iter());
      d.flow1.clear();
      d.flow1.extend(features1.iter());
    }
  }

  #[allow(non_snake_case)]
  fn process_feature(
    &mut self,
    frame_camera0: &FrameCamera,
    frame_camera1: &FrameCamera,
    feature0: Feature,
    point1_in: Option<Vector2d>,
  ) -> Option<Feature> {
    let term2 = self.lk_term.powi(2);
    let r = (self.lk_win_size - 1) / 2;
    let mut g = point1_in.map(|p| p - feature0.point).unwrap_or(Vector2d::zeros())
      / u32::pow(2, self.lk_levels as u32) as f64;
    let mut d = Vector2d::zeros();
    for L in (0..self.lk_levels + 1).rev() {
      let level0 = frame_camera0.get_level(L);
      let level1 = frame_camera1.get_level(L);
      let u = feature0.point / u32::pow(2, L as u32) as f64;
      let range = integration_range(&level0, u, r, 1)?;
      scharr(&level0, u, range, &mut self.Ix, &mut self.Iy, &mut self.grid0);
      let G = spatial_gradient(range, &self.Ix, &self.Iy);
      if G.eigenvalues()?.min() < self.lk_min_eig { return None }
      let mut converged = false;
      let mut nu = Vector2d::zeros();
      for _ in 0..self.lk_iters {
        image_difference(range, r, &self.grid0, &mut self.It, &level1, u + g + nu)?;
        let eta = flow_vector(&G, &self.Ix, &self.Iy, &self.It)?;
        nu += eta;
        if eta.norm_squared() < term2 {
          converged = true;
          break;
        }
      }
      d = nu;
      if !converged { return None }
      if L > 0 { g = 2. * (g + d) }
    }
    // Verify match in the one-camera tracking where distortions are expected to
    // be smaller.
    // if kind == OpticalFlowKind::LeftPreviousToCurrent {
    //   if self.It.component_mul(&self.It).sum() / (self.It.nrows() * self.It.ncols()) as f64 > 1000. {
    //     return None;
    //   }
    // }

    // if let Some(point1_in) = point1_in {
    //   let feature1 = Feature {
    //     point: feature0.point + g + d,
    //     id: feature0.id,
    //   };
    //   let good = (point1_in - feature1.point).norm();
    //   let bad = (feature0.point - feature1.point).norm();
    //   if bad < good {
    //     info!("{} {}", bad, good);
    //   }
    // }
    Some(Feature {
      point: feature0.point + g + d,
      id: feature0.id,
    })
  }
}

#[allow(non_snake_case)]
fn image_difference(
  prev_range: Range,
  r: usize,
  I0: &Matrixd,
  mut It: &mut Matrixd,
  level: &Image,
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
  level: &Image,
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
    let fract = if center[i].fract() > 0. { 1 } else { 0 };
    range[i] = [i16::max(-r, -n + padding), i16::min(r, s as i16 - n - padding - 1 - fract)];
  }
  Some(range)
}

fn fill_grid(
  level: &Image,
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
  level: &Image,
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
fn bilinear(frame: &Image, u: Vector2d) -> f64 {
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

fn transform_vector3d(m: &Matrix4d, v: &Vector3d) -> Vector3d {
  m.fixed_slice::<3, 3>(0, 0) * v + m.fixed_slice::<3, 1>(0, 3)
}

fn compute_initial_guess(
  p0: Vector2d,
  cameras: &[&Camera],
  cam0_to_cam1: &Matrix4d,
) -> Option<Vector2d> {
  if cameras[0].imu_to_camera == cameras[1].imu_to_camera { return None }
  if let Some(ray) = cameras[0].model.pixel_to_ray(p0) {
    let r0 = AVERAGE_DISTANCE_METERS * ray;
    let r1 = transform_vector3d(&cam0_to_cam1, &r0);
    let r1 = r1.normalize(); // TODO needed?
    // TODO Check these are correct by a visualization?
    cameras[1].model.ray_to_pixel(r1)
  }
  else {
    None
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn make_camera(image: Image, lk_levels: usize) -> FrameCamera {
    let mut pyramid = Pyramid::empty();
    Pyramid::compute(&mut pyramid, &image, lk_levels).unwrap();
    FrameCamera {
      image,
      pyramid,
    }
  }

  #[test]
  fn test_flow() {
    // Use a quite large image to avoid issues at the borders.
    let mut image0 = Image {
      data: vec![0; 128 * 128],
      width: 128,
      height: 128,
    };
    let mut image1 = image0.clone();

    let patch = Image {
      data: vec![
        44, 44, 44, 44, 44, 44, 44, 44, 44,
        44, 55, 55, 55, 55, 55, 55, 55, 44,
        44, 55, 77, 77, 77, 77, 77, 55, 44,
        44, 55, 77, 88, 88, 88, 77, 55, 44,
        44, 55, 77, 88, 99, 88, 77, 55, 44,
        44, 55, 77, 88, 88, 88, 77, 55, 44,
        44, 55, 77, 77, 77, 77, 77, 55, 44,
        44, 55, 55, 55, 55, 55, 55, 55, 44,
        44, 44, 44, 44, 44, 44, 44, 44, 44,
      ],
      width: 9,
      height: 9,
    };

    // Place the patch at different positions in the two images.
    let x: i32 = 60;
    let y: i32 = 60;
    let dx: i32 = -14;
    let dy: i32 = 7;
    image0.set_sub_image_i32(x, y, &patch);
    image1.set_sub_image_i32(x + dx, y + dy, &patch);

    let lk_levels = 3;
    let lk_iters = 5;
    let lk_win_size = 7;
    let lk_term = 0.1;
    let lk_min_eig = 1e-4;

    let camera0 = make_camera(image0, lk_levels);
    let camera1 = make_camera(image1, lk_levels);

    // Place feature at center of the first patch.
    let r = (patch.width - 1) as i32 / 2;
    let feature0 = Feature {
      point: Vector2d::new((x + r) as f64, (y + r) as f64),
      id: TrackId(0),
    };
    let mut flow = OpticalFlow::new_custom(lk_iters, lk_levels, lk_win_size, lk_term, lk_min_eig).unwrap();
    // let guess = feature0.point + Vector2d::new(dx as f64, dy as f64);
    // if let Some(feature1) = flow.process_feature(&camera0, &camera1, feature0, Some(guess)) {
    if let Some(feature1) = flow.process_feature(&camera0, &camera1, feature0, None) {
      // The found feature should be near center of the second patch.
      let err = (feature1.point - feature0.point) - Vector2d::new(dx as f64, dy as f64);
      dbg!(err.norm());
      // dbg!(feature1, err);
    }
    else {
      assert!(false);
    }
  }

  #[test]
  fn test_scharr() {
    let mut image = Image {
      data: vec![
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
      ],
      width: 5,
      height: 5,
    };
    let mut out_x = dmatrix!();
    let mut out_y = dmatrix!();
    let mut grid = dmatrix!();
    let center = Vector2d::new(2.0, 2.0);
    let range = integration_range(&image, center, 1, 1).unwrap();
    scharr(&image, center, range, &mut out_x, &mut out_y, &mut grid);
    assert_eq!(out_x, DMatrix::zeros(3, 3));
    assert_eq!(out_y, DMatrix::zeros(3, 3));

    image.data = vec![
      0, 1, 2, 3, 4,
      0, 1, 2, 3, 4,
      0, 1, 2, 3, 4,
      0, 1, 2, 3, 4,
      0, 1, 2, 3, 4,
    ];
    scharr(&image, center, range, &mut out_x, &mut out_y, &mut grid);
    assert_eq!(out_x, DMatrix::repeat(3, 3, 1.));
    assert_eq!(out_y, DMatrix::zeros(3, 3));

    image.data = vec![
      0, 1, 2, 3, 4,
      1, 2, 3, 4, 5,
      2, 3, 4, 5, 6,
      3, 4, 5, 6, 7,
      4, 5, 6, 7, 8,
    ];
    scharr(&image, center, range, &mut out_x, &mut out_y, &mut grid);
    assert_eq!(out_x, DMatrix::repeat(3, 3, 1.));
    assert_eq!(out_y, DMatrix::repeat(3, 3, 1.));

    image.data = vec![
      0, 0, 5, 0, 0,
      0, 0, 5, 0, 0,
      0, 0, 5, 0, 0,
      0, 0, 5, 0, 0,
      0, 0, 5, 0, 0,
    ];
    scharr(&image, center, range, &mut out_x, &mut out_y, &mut grid);
    let answer_x = dmatrix!(
      2.5, 0., -2.5;
      2.5, 0., -2.5;
      2.5, 0., -2.5;
    );
    assert_eq!(out_x, answer_x);
    assert_eq!(out_y, DMatrix::zeros(3, 3));
  }

  #[test]
  fn test_integration_range() {
    // Width and height are pixels. Coordinate (0, 0) means center of top-left
    // pixel. Thus (9, 9) is the center of the bottom-right pixel for 10x10
    // image.
    let image = Image {
      data: vec![],
      width: 10,
      height: 10,
    };
    assert_eq!(integration_range(&image, Vector2d::new(4.5, 4.5), 3, 0).unwrap(), [[-3, 3], [-3, 3]]);
    assert_eq!(integration_range(&image, Vector2d::new(1.5, 2.5), 3, 0).unwrap(), [[-1, 3], [-2, 3]]);
    assert_eq!(integration_range(&image, Vector2d::new(1.0, 2.0), 3, 0).unwrap(), [[-1, 3], [-2, 3]]);
    assert_eq!(integration_range(&image, Vector2d::new(0.9, 1.9), 3, 0).unwrap(), [[0, 3], [-1, 3]]);
    assert_eq!(integration_range(&image, Vector2d::new(0.9, 1.9), 3, 1).unwrap(), [[1, 3], [0, 3]]);
    assert_eq!(integration_range(&image, Vector2d::new(8.5, 2.0), 3, 0).unwrap(), [[-3, 0], [-2, 3]]);
    assert_eq!(integration_range(&image, Vector2d::new(9.5, 2.0), 3, 0), None);
  }
}
