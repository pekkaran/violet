use crate::all::*;

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

    let p = &*PARAMETER_SET.lock().unwrap();
    Ok(Frame {
      pyramid: Pyramid::new(&input_frame.video, unused_pyramid, p.lk_levels)?,
      data,
      width: input_frame.video.width,
      height: input_frame.video.height,
    })
  }
}
