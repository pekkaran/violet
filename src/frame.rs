use crate::all::*;

// Data derived from single frame input to `Vio::process()`.
pub struct Frame {
  pub cameras: Vec<FrameCamera>
}

pub struct FrameCamera {
  pub data: Vec<u8>,
  pub width: usize,
  pub height: usize,
  pub pyramid: Pyramid,
}

pub struct Level<'a> {
  pub data: &'a[u8],
  pub width: usize,
  pub height: usize,
}

impl Frame {
  pub fn new(
    input_frame: &InputFrame,
    unused_frame: Option<Frame>,
  ) -> Result<Frame> {
    let mut frame = if let Some(mut unused_frame) = unused_frame {
      // Move data buffer from old unused frame to the new frame to avoid allocation.
      for i in 0..unused_frame.cameras.len() {
        unused_frame.cameras[i].data.clear();
      }
      unused_frame
    }
    else {
      let mut cameras = vec![];
      for video in &input_frame.videos {
        cameras.push(FrameCamera {
          pyramid: Pyramid::empty(),
          data: video.data.clone(),
          width: video.width,
          height: video.height,
        });
      }
      Frame { cameras }
    };

    let lk_levels = {
      let p = PARAMETER_SET.lock().unwrap();
      p.lk_levels
    };
    for (i, camera) in frame.cameras.iter_mut().enumerate() {
      camera.data.extend(input_frame.videos[i].data.iter());
      Pyramid::compute(&mut camera.pyramid, &input_frame.videos[0], lk_levels)?;
    }
    Ok(frame)
  }
}

impl FrameCamera {
  pub fn get_level(&self, level: usize) -> Level {
    if level == 0 {
      Level {
        data: &self.data,
        width: self.width,
        height: self.height,
      }
    }
    else {
      let size = self.pyramid.size(level);
      Level {
        data: &self.pyramid.levels[level - 1],
        width: size[0],
        height: size[1],
      }
    }
  }
}
