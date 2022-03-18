use crate::all::*;

pub struct Vio {
  tracker: Tracker,

}

impl Vio {
  pub fn new() -> Vio {
    Vio {
      tracker: Tracker::new(),
    }
  }

  pub fn process(&mut self, input_data: &InputData) {
    match input_data {
      InputData::Gyroscope { time: _, v: _ } => {},
      InputData::Accelerometer { time: _, v: _ } => {}
      InputData::Frame(ref frame) => {},
    }
  }
}
