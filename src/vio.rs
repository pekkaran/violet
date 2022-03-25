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

  pub fn process(&mut self, input_data: &InputData) {
    match input_data.sensor {
      InputDataSensor::Frame(ref frame) => {
        self.process_frame(frame);
      },
      InputDataSensor::Gyroscope(_) => {},
      InputDataSensor::Accelerometer(_) => {}
    }
  }

  fn process_frame(&mut self, frame: &InputFrame) {
    self.frames.push(Frame::new(frame));
    assert!(MAX_FRAMES_IN_MEMORY >= 1);
    if self.frames.len() > MAX_FRAMES_IN_MEMORY {
      self.frames.remove(0);
    }
    let frame = self.frames.iter().last().unwrap();

    self.tracker.process(frame);
  }
}
