use crate::all::*;

// Values 9 and 12 are popular, allowing quick rejection logic.
// const FAST_VARIANT_N: usize = 9;
const FAST_VARIANT_N: usize = 12;

// A Bresenham circle.
const CIRCLE_RADIUS: usize = 3;
const CIRCLE: [[i32; 2]; 16] = [
  [ 0, -3], [ 1, -3], [ 2, -2], [ 3, -1], [ 3,  0], [ 3,  1], [ 2,  2], [ 1,  3],
  [ 0,  3], [-1,  3], [-2,  2], [-3,  1], [-3,  0], [-3, -1], [-2, -2], [-1, -3],
];

pub struct Detector {
  start_threshold: i16,
  mask: Vec<bool>,
}

impl Detector {
  pub fn new() -> Detector {
    Detector {
      start_threshold: 128,
      mask: vec![],
    }
  }

  pub fn process(
    &mut self,
    image: &Image,
    detections: &mut Vec<Feature>,
    needed_features_count: usize,
    next_id: &mut TrackId,
  ) {
    assert!(image.width > 1 + 2 * CIRCLE_RADIUS);
    assert!(image.height > 1 + 2 * CIRCLE_RADIUS);
    detections.clear();
    if needed_features_count == 0 { return }
    self.mask.clear();
    for _ in 0 .. (image.width * image.height) {
      self.mask.push(false);
    }
    let mut threshold = self.start_threshold;
    let mask_radius = ((image.width.max(image.height) as f32) / 100.0).round() as i32;
    let threshold_halving_iterations = 4;

    'detection:
    for _ in 0..threshold_halving_iterations {
      for x in CIRCLE_RADIUS .. (image.width - CIRCLE_RADIUS) {
        for y in CIRCLE_RADIUS .. (image.height - CIRCLE_RADIUS) {
          if self.mask[y * image.width + x] { continue }
          if !self.detect_at_pixel(x as i32, y as i32, image, threshold) { continue }
          detections.push(Feature {
            point: Vector2d::new(x as f64, y as f64),
            id: *next_id,
          });
          next_id.0 += 1;
          add_mask(&mut self.mask, x as i32, y as i32, image.width, image.height, mask_radius);
          if detections.len() >= needed_features_count { break 'detection }
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
    image: &Image,
    threshold: i16,
  ) -> bool {
    let center_value = image.value_i32(x, y) as i16;
    if continuous(x, y, image, |v| (v as i16) < center_value - threshold) { return true }
    if continuous(x, y, image, |v| (v as i16) > center_value + threshold) { return true }
    false
  }
}

fn continuous<F: Fn(u8) -> bool>(x: i32, y: i32, image: &Image, f: F) -> bool {
  // There are also other quick rejection schemes depending on the threshold.
  if FAST_VARIANT_N >= 9
    && !f(image.value_i32(x + 3, y)) && !f(image.value_i32(x - 3, y)
  ) {
      return false
  }

  let it = CircleIterator::new(x, y);
  let mut n = 0;
  for p in it {
    if f(image.value_i32(p[0], p[1])) {
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
