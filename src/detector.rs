use crate::all::*;

// Values 9 and 12 are popular, allowing quick rejection logic.
const FAST_VARIANT_N: usize = 9;

// A Bresenham circle.
const CIRCLE_RADIUS: usize = 3;
const CIRCLE: [[i32; 2]; 16] = [
  [ 0, -3], [ 1, -3], [ 2, -2], [ 3, -1], [ 3,  0], [ 3,  1], [ 2,  2], [ 1,  3],
  [ 0,  3], [-1,  3], [-2,  2], [-3,  1], [-3,  0], [-3, -1], [-2, -2], [-1, -3],
];

pub struct Detector {
  // TODO move to parameters
  pub start_threshold: i16,
  mask: Vec<bool>,
}

impl Detector {
  pub fn new() -> Detector {
    Detector {
      start_threshold: 64,
      mask: vec![],
    }
  }

  pub fn process(
    &mut self,
    frame_camera: &FrameCamera,
    detections: &mut Vec<Vector2d>,
  ) {
    detections.clear();
    self.mask.clear();
    for _ in 0 .. (frame_camera.width * frame_camera.height) {
      self.mask.push(false);
    }
    let mut threshold = self.start_threshold;
    let mask_radius = ((frame_camera.width.max(frame_camera.height) as f32) / 100.0).round() as i32;
    for _ in 0..4 {
      for x in CIRCLE_RADIUS .. (frame_camera.width - CIRCLE_RADIUS) {
        for y in CIRCLE_RADIUS .. (frame_camera.height - CIRCLE_RADIUS) {
          if self.mask[y * frame_camera.width + x] { continue }
          if !self.detect_at_pixel(x as i32, y as i32, frame_camera, threshold) { continue }
          detections.push(Vector2d::new(x as f64, y as f64));
          add_mask(&mut self.mask, x as i32, y as i32, frame_camera.width, frame_camera.height, mask_radius);
        }
      }
      threshold /= 2;
    }

    let d = &mut DEBUG_DATA.lock().unwrap();
    let p = PARAMETER_SET.lock().unwrap();
    if p.show_features {
      d.detections.clear();
      d.detections.extend(detections.iter());
    }
    if p.show_mask {
      d.detection_mask.clear();
      d.detection_mask.extend(self.mask.iter());
    }
  }

  fn detect_at_pixel(
    &mut self,
    x: i32,
    y: i32,
    frame_camera: &FrameCamera,
    threshold: i16,
  ) -> bool {
    let center_value = value(x, y, frame_camera);
    if continuous(x, y, frame_camera, |v| v < center_value - threshold) { return true }
    if continuous(x, y, frame_camera, |v| v > center_value + threshold) { return true }
    false
  }
}

fn continuous<F: Fn(i16) -> bool>(x: i32, y: i32, frame_camera: &FrameCamera, f: F) -> bool {
  // Quick rejection for 9 and 12 variants.
  if !f(value(x + 3, y, frame_camera)) && !f(value(x - 3, y, frame_camera)) { return false }

  let it = CircleIterator::new(x, y);
  let mut n = 0;
  for p in it {
    let v = value(p[0], p[1], frame_camera);
    if f(v) {
      n += 1;
      if n >= FAST_VARIANT_N { return true }
    }
    // Could quit early if n + remaining is too small.
    else {
      n = 0;
    }
  }
  false
}

struct CircleIterator {
  center: [i32; 2],
  ind: usize,
}

impl CircleIterator {
  pub fn new(x: i32, y:i32) -> CircleIterator {
    CircleIterator {
      center: [x, y],
      ind: 0,
    }
  }
}

impl Iterator for CircleIterator {
  type Item = [i32; 2];

  fn next(&mut self) -> Option<Self::Item> {
    if self.ind >= CIRCLE.len() { return None }
    self.ind += 1;
    Some([
      self.center[0] + CIRCLE[self.ind - 1][0],
      self.center[1] + CIRCLE[self.ind - 1][1],
    ])
  }
}

#[inline(always)]
fn value(x: i32, y: i32, frame_camera: &FrameCamera) -> i16 {
  frame_camera.data[y as usize * frame_camera.width + x as usize] as i16
}

fn add_mask(
  mask: &mut Vec<bool>,
  cx: i32,
  cy: i32,
  width: usize,
  height: usize,
  mask_radius: i32,
) {
  for x in (cx - mask_radius) .. (cx + mask_radius + 1) {
    if x < 0 || x >= width as i32 { continue }
    for y in (cy - mask_radius) .. (cy + mask_radius + 1) {
      if y < 0 || y >= height as i32 { continue }
      mask[y as usize * width + x as usize] = true;
    }
  }
}
