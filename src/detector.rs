use crate::all::*;

// Values 9 and 12 are popular, allowing quick rejection logic.
const FAST_VARIANT_N: usize = 12;

// A Bresenham circle.
const CIRCLE_RADIUS: usize = 3;
const CIRCLE: [[i32; 2]; 16] = [
  [ 0, -3], [ 1, -3], [ 2, -2], [ 3, -1], [ 3,  0], [ 3,  1], [ 2,  2], [ 1,  3],
  [ 0,  3], [-1,  3], [-2,  2], [-3,  1], [-3,  0], [-3, -1], [-2, -2], [-1, -3],
];

pub struct Detector {
  // TODO move to parameters
  pub threshold: i16,
}

impl Detector {
  pub fn new() -> Detector {
    Detector {
      threshold: 10,
    }
  }

  pub fn process(&mut self, frame: &VideoFrame, detections: &mut Vec<Pixel>) {
    detections.clear();
    for x in CIRCLE_RADIUS .. (frame.width - CIRCLE_RADIUS) {
      for y in CIRCLE_RADIUS .. (frame.height - CIRCLE_RADIUS) {
        if !self.detect_at_pixel(x as i32, y as i32, frame) { continue }
        detections.push(Pixel::new(x as i32, y as i32));
      }
    }
  }

  fn detect_at_pixel(&mut self, x: i32, y: i32, frame: &VideoFrame) -> bool {
    let center_value = value(x, y, frame);
    if continuous(x, y, frame, |v| v < center_value - self.threshold) { return true }
    if continuous(x, y, frame, |v| v > center_value + self.threshold) { return true }
    false
  }
}

fn continuous<F: Fn(i16) -> bool>(x: i32, y: i32, frame: &VideoFrame, f: F) -> bool {
  // Quick rejection for 9 and 12 variants.
  if !f(value(x + 3, y, frame)) && !f(value(x - 3, y, frame)) { return false }

  let it = CircleIterator::new(x, y);
  let mut n = 0;
  for p in it {
    let v = value(p[0], p[1], frame);
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

fn value(x: i32, y: i32, frame: &VideoFrame) -> i16 {
  frame.data[y as usize * frame.width + x as usize] as i16
}
