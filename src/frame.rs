use crate::all::*;

// Data derived from single frame input to `Vio::process()`.
pub struct Frame {
  pub data: Vec<u8>,
  pub width: usize,
  pub height: usize,
  // Pyramids etc.
}

impl Frame {
  pub fn new(input_frame: &InputFrame) -> Frame {
    Frame {
      data: input_frame.video.data.clone(),
      width: input_frame.video.width,
      height: input_frame.video.height,
    }
  }
}
