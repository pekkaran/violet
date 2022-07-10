// Handles lazy initialization of `Vio` so that it has access to information
// from the first video frames.

use crate::all::*;

pub struct VioInit {
  cameras: Vec<Camera>,
}

impl VioInit {
  pub fn new(cameras: Vec<Camera>) -> VioInit {
    VioInit {
      cameras,
    }
  }

  pub fn try_init(&mut self, input_data: &InputData) -> Option<Result<Vio>> {
    if let InputDataSensor::Frame(ref frame) = input_data.sensor {
      let frame_scale = compute_frame_scale(&frame.images);
      let mut cameras = vec![];
      mem::swap(&mut self.cameras, &mut cameras);
      Some(Vio::new(cameras, frame_scale))
    }
    else {
      // Ignore data before the first frame. Could also save in buffer and replay.
      None
    }
  }
}

fn compute_frame_scale(images: &[&Image]) -> f64 {
  assert!(!images.is_empty());
  let i = images[0];
  // Normalize so that the value is roughly 1.
  1e-3 * ((i.width * i.width + i.height * i.height) as f64).sqrt()
}
