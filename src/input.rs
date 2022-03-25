use crate::all::*;

pub struct Input {
  reader: BufReader<File>,
  line: String,
  video_input: VideoInput,
}

pub struct InputFrame<'a> {
  // Maybe a Vec<> here for stereo input?
  pub video: &'a VideoFrame,
}

pub struct InputData<'a> {
  pub time: f64,
  pub sensor: InputDataSensor<'a>,
}

pub enum InputDataSensor<'a> {
  Gyroscope(Vector3d),
  Accelerometer(Vector3d),
  Frame(InputFrame<'a>),
}

impl Input {
  pub fn new(path: &Path) -> Result<Input> {
    let file = File::open(path.join("data.jsonl"))?;
    let video_input = VideoInput::new(&path.join("data.mp4"))
      .context("Failed to create video input")?;
    Ok(Input {
      reader: BufReader::new(file),
      line: String::new(),
      video_input,
    })
  }

  // Not using the Iterator trait here because "streaming iterators" are not
  // supported by the trait. Specifically, the yielded items are not allowed
  // to borrow from the `Input` struct, but the InputData::Frame variant does
  // that to avoid allocating a new buffers for the images.
  pub fn next(&mut self) -> Result<Option<InputData>> {
    loop {
      self.line.clear();
      match self.reader.read_line(&mut self.line) {
        Ok(0) => return Ok(None),
        Err(err) => bail!("Failed to read line. {}", err),
        _ => {},
      }
      let value: serde_json::Value = serde_json::from_str(&self.line)
        .context(format!("Input::next JSON deserialization failed for line: {}", self.line))?;
      let value = value.as_object()
        .ok_or(anyhow!("JSONL line is not a map."))?;
      let time = value["time"].as_f64()
        .ok_or(anyhow!("Time is not a number."))?;

      if let Some(sensor) = value.get("sensor") {
        let v = &sensor["values"].as_array()
          .ok_or(anyhow!("Sensor values field is not an array."))?;
        let v: Vec<f64> = v.iter().map(|x| x.as_f64().unwrap()).collect();
        assert!(v.len() >= 3);
        let v = Vector3d::new(v[0], v[1], v[2]);
        let sensor_type = sensor["type"].as_str()
          .ok_or(anyhow!("Sensor type is not a string."))?;

        match sensor_type {
          "gyroscope" => return Ok(Some(InputData {
            time,
            sensor: InputDataSensor::Gyroscope(v),
          })),
          "accelerometer" => return Ok(Some(InputData {
            time,
            sensor: InputDataSensor::Accelerometer(v),
          })),
          _ => {
            warn!("Unknown sensor type {}", sensor_type);
            continue;
          },
        }
      }
      else if let Some(_frames) = value.get("frames") {
        let input_frame = InputFrame {
          video: self.video_input.read()?,
        };
        return Ok(Some(InputData {
          time,
          sensor: InputDataSensor::Frame(input_frame),
        }));
      }
      else if let Some(_) = value.get("groundTruth") {
        // Pass.
      }
      else {
        warn!("Unrecognized data: {}", self.line);
        continue;
      }
    }
  }
}
