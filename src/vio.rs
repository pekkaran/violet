use crate::all::*;

const MAX_FRAMES_IN_MEMORY: usize = 2;

pub struct Vio {
  // Use private fields to clarify this struct would form the main API.
  tracker: Tracker,
  frames: Vec<Frame>,
}

impl Vio {
  pub fn new() -> Result<Vio> {
    Ok(Vio {
      tracker: Tracker::new()?,
      frames: vec![],
    })
  }

  pub fn get_frames(&self) -> &[Frame] {
    &self.frames
  }

  pub fn process(&mut self, input_data: &InputData) -> Result<()> {
    match input_data.sensor {
      InputDataSensor::Frame(ref frame) => {
        self.process_frame(frame)?;
      },
      InputDataSensor::Gyroscope(_) => {},
      InputDataSensor::Accelerometer(_) => {}
    }
    Ok(())
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
