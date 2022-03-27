use crate::all::*;

const MAX_FRAMES_IN_MEMORY: usize = 2;

pub struct Vio {
  tracker: Tracker,
  frames: Vec<Frame>,
}

impl Vio {
  pub fn new() -> Vio {
    Vio {
      tracker: Tracker::new(),
      frames: vec![],
    }
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
    let frame = self.frames.iter().last().unwrap();

    self.tracker.process(frame);
    Ok(())
  }
}
