use crate::all::*;

// Data derived from single frame input to `Vio::process()`.
pub struct Frame {
  pub data: Vec<u8>,
  pub width: usize,
  pub height: usize,
  // Pyramids etc.
}

impl Frame {
  pub fn new(input_frame: &InputFrame, unused_frame: Option<Frame>) -> Frame {
    let data = if let Some(mut unused_frame) = unused_frame {
      // Move data buffer from old unused frame to the new frame to avoid allocation.
      unused_frame.data.clear();
      unused_frame.data.extend(input_frame.video.data.iter());
      unused_frame.data
    }
    else {
      input_frame.video.data.clone()
    };

    Frame {
      data,
      width: input_frame.video.width,
      height: input_frame.video.height,
    }
  }
}
