use crate::all::*;

pub const PYRAMID_LEVEL_COUNT: usize = 3;

// Data derived from single frame input to `Vio::process()`.
pub struct Frame {
  pub data: Vec<u8>,
  pub width: usize,
  pub height: usize,
  pub pyramid: Pyramid,
}

impl Frame {
  pub fn new(
    input_frame: &InputFrame,
    unused_frame: Option<Frame>,
  ) -> Result<Frame> {
    let (data, unused_pyramid) = if let Some(mut unused_frame) = unused_frame {
      // Move data buffer from old unused frame to the new frame to avoid allocation.
      unused_frame.data.clear();
      unused_frame.data.extend(input_frame.video.data.iter());
      (unused_frame.data, Some(unused_frame.pyramid))
    }
    else {
      (input_frame.video.data.clone(), None)
    };

    Ok(Frame {
      pyramid: Pyramid::new(&input_frame.video, unused_pyramid, PYRAMID_LEVEL_COUNT)?,
      data,
      width: input_frame.video.width,
      height: input_frame.video.height,
    })
  }
}
