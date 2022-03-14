use crate::all::*;

pub struct Input {
  reader: BufReader<File>,
  line: String,
  // TODO Read video data.
  // Outer Vec for mono, stereo, etc., inner for frame data bytes size of `height*width`.
  // video: Vec<Vec<u8>>,
}

pub enum InputData {
  Gyroscope { time: f64, v: Vector3d },
  Accelerometer { time: f64, v: Vector3d },
  // Frame
}

impl Input {
  pub fn new(path: &Path) -> Result<Input> {
    let file = File::open(path)?;
    Ok(Input {
      reader: BufReader::new(file),
      line: String::new(),
    })
  }

  // Not using `impl Iterator` to allow returning `Result`.
  // End of data is signaled by `Result::Ok(Option::None)`.
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
          .ok_or(anyhow!("Sensor values field is not an array"))?;
        let v: Vec<f64> = v.iter().map(|x| x.as_f64().unwrap()).collect();
        assert!(v.len() >= 3);
        let v = Vector3d::new(v[0], v[1], v[2]);
        let sensor_type = sensor["type"].as_str()
          .ok_or(anyhow!("Sensor type is not a string."))?;

        match sensor_type {
          "gyroscope" => return Ok(Some(InputData::Gyroscope { time, v })),
          "accelerometer" => return Ok(Some(InputData::Accelerometer { time, v })),
          _ => {
            warn!("Unknown sensor type {}", sensor_type);
            continue;
          },
        }
      }
      else if let Some(_frames) = value.get("frames") {
        // dbg!(frames);
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
