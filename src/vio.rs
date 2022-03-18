use crate::all::*;

pub struct Vio {
  tracker: Tracker,
  last_time: Option<f64>,
}

impl Vio {
  pub fn new() -> Vio {
    Vio {
      tracker: Tracker::new(),
      last_time: None,
    }
  }

  pub fn process(&mut self, input_data: &InputData) {
    if let Some(last_time) = self.last_time {
      if input_data.time <= last_time {
        warn!("Ignoring unordered/duplicated sensor sample.");
        return;
      }
    }
    self.last_time = Some(input_data.time);

    match input_data.sensor {
      InputDataSensor::Frame(ref frame) => {
        self.tracker.process(frame);
      },
      InputDataSensor::Gyroscope(_) => {},
      InputDataSensor::Accelerometer(_) => {}
    }
  }
}
