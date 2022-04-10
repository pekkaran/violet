use crate::all::*;

// Data derived from single frame input to `Vio::process()`.
pub struct Frame {
  pub cameras: Vec<FrameCamera>
}

pub struct FrameCamera {
  pub image: Image,
  pub pyramid: Pyramid,
}

impl Frame {
  pub fn new(
    input_frame: &InputFrame,
    unused_frame: Option<Frame>,
  ) -> Result<Frame> {
    let mut frame = if let Some(mut unused_frame) = unused_frame {
      // Move data buffer from old unused frame to the new frame to avoid allocation.
      for i in 0..unused_frame.cameras.len() {
        unused_frame.cameras[i].image.clear();
      }
      unused_frame
    }
    else {
      let mut cameras = vec![];
      for image in &input_frame.images {
        cameras.push(FrameCamera {
          image: (*image).clone(),
          pyramid: Pyramid::empty(),
        });
      }
      Frame { cameras }
    };

    let lk_levels = {
      let p = PARAMETER_SET.lock().unwrap();
      p.lk_levels
    };
    for (i, camera) in frame.cameras.iter_mut().enumerate() {
      camera.image.data.extend(input_frame.images[i].data.iter());
      camera.image.width = input_frame.images[i].width;
      camera.image.height = input_frame.images[i].height;
      Pyramid::compute(&mut camera.pyramid, &input_frame.images[i], lk_levels)?;
    }
    Ok(frame)
  }
}

impl FrameCamera {
  pub fn get_level(&self, level: usize) -> &Image {
    if level == 0 {
      &self.image
    }
    else {
      &self.pyramid.levels[level - 1]
    }
  }
}
