use crate::all::*;

const MAX_FRAMES_IN_MEMORY: usize = 2;

pub struct Vio {
  // Use private fields to clarify this struct would form the main API.
  tracker: Tracker,
  frames: Vec<Frame>,
  frame_number: usize,
  frame_sub: usize,
}

impl Vio {
  pub fn new() -> Result<Vio> {
    let frame_sub = {
      let p = PARAMETER_SET.lock().unwrap();
      p.frame_sub
    };
    Ok(Vio {
      tracker: Tracker::new()?,
      frames: vec![],
      frame_number: 0,
      frame_sub,
    })
  }

  pub fn get_frames(&self) -> &[Frame] {
    &self.frames
  }

  // Returns true if processed a frame.
  pub fn process(&mut self, input_data: &InputData) -> Result<bool> {
    match input_data.sensor {
      InputDataSensor::Frame(ref frame) => {
        assert!(frame.images[0].width > 0 && frame.images[0].height > 0);
        assert!(frame.images[1].width > 0 && frame.images[1].height > 0);
        self.frame_number += 1;
        if (self.frame_number - 1) % self.frame_sub == 0 {
          self.process_frame(frame)?;
          return Ok(true);
        }
      },
      InputDataSensor::Gyroscope(_) => {},
      InputDataSensor::Accelerometer(_) => {}
    }
    Ok(false)
  }

  fn process_frame(&mut self, frame: &InputFrame) -> Result<()> {
    assert!(MAX_FRAMES_IN_MEMORY >= 1);
    let mut unused_frame = None;
    if self.frames.len() >= MAX_FRAMES_IN_MEMORY {
      unused_frame = Some(self.frames.remove(0));
    };

    self.frames.push(Frame::new(frame, unused_frame)?);

    let frame0 = self.frames.iter().rev().nth(1);
    let frame1 = self.frames.iter().last().unwrap();
    self.tracker.process(frame0, frame1);
    Ok(())
  }
}
