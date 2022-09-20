// The main algorithm class. Takes input data and produces pose estimates.

use crate::all::*;

const MAX_FRAMES_IN_MEMORY: usize = 2;

#[allow(dead_code)]
pub struct Vio {
  // Use private fields to clarify this struct would form the main API.
  tracker: Tracker,
  kalman_filter: KalmanFilter,
  stationary: Stationary,
  visual_update: VisualUpdate,
  cameras: Vec<Camera>,
  frames: Vec<Frame>,
  // Incremented just before processing a new frame. 0 before the first frame.
  frame_number: usize,
  // Last element is the current pose. Augmentation duplicates the last element
  // and shortens the trail is at maximum length. New frame replaces the
  // previous last element. This logic allows to skip the augmentation easily.
  pose_trail_frame_numbers: VecDeque<usize>,
  frame_sub: usize,
  last_gyroscope: Option<(f64, Vector3d)>,
  last_accelerometer: Option<(f64, Vector3d)>,
  last_time: Option<f64>,
  kf_noise_zero_velocity: f64,
  pose_trail_len: usize,
}

impl Vio {
  pub fn new(cameras: Vec<Camera>, frame_scale: f64) -> Result<Vio> {
    let (frame_sub, kf_noise_zero_velocity, pose_trail_len) = {
      let p = PARAMETER_SET.lock().unwrap();
      (p.frame_sub, p.kf_noise_zero_velocity, p.pose_trail_len)
    };
    let mut pose_trail_frame_numbers = VecDeque::new();
    pose_trail_frame_numbers.push_back(0);

    Ok(Vio {
      tracker: Tracker::new()?,
      kalman_filter: KalmanFilter::new(),
      stationary: Stationary::new(frame_scale),
      visual_update: VisualUpdate::new(),
      cameras,
      frames: vec![],
      pose_trail_frame_numbers,
      frame_number: 0,
      frame_sub,
      last_gyroscope: None,
      last_accelerometer: None,
      last_time: None,
      kf_noise_zero_velocity,
      pose_trail_len,
    })
  }

  pub fn get_frames(&self) -> &[Frame] {
    &self.frames
  }

  // Returns true if processed a frame.
  pub fn process(&mut self, input_data: &InputData) -> Result<bool> {
    if let Some(last_time) = self.last_time {
      if input_data.time < last_time {
        warn!("Discarding unordered sample.");
        return Ok(false);
      }
    }
    self.last_time = Some(input_data.time);

    match input_data.sensor {
      InputDataSensor::Frame(ref frame) => {
        assert!(frame.images[0].width > 0 && frame.images[0].height > 0);
        assert!(frame.images[1].width > 0 && frame.images[1].height > 0);
        self.frame_number += 1;
        if (self.frame_number - 1) % self.frame_sub == 0 {
          self.pose_trail_frame_numbers.pop_back();
          self.pose_trail_frame_numbers.push_back(self.frame_number);

          self.process_frame(frame)?;
          self.update_debug_data_3d();
          return Ok(true);
        }
      },
      InputDataSensor::Gyroscope(gyroscope) => {
        self.last_gyroscope = Some((input_data.time, gyroscope));
      },
      InputDataSensor::Accelerometer(accelerometer) => {
        self.last_accelerometer = Some((input_data.time, accelerometer));
      },
    }

    // Very basic sample synchronization that only aims to cover the case that
    // the gyroscope and accelerometer samples are already paired one-to-one in
    // the input data, but it's not known if the accelerometer or gyroscope
    // sample comes first in the stream.
    if let (Some((time_g, gyroscope)), Some((time_a, accelerometer)))
      = (self.last_gyroscope, self.last_accelerometer)
    {
      if time_a >= time_g {
        self.process_imu(time_g, gyroscope, accelerometer);
        self.last_gyroscope = None;
      }
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
    let frame1 = self.frames.iter().rev().nth(0).unwrap();
    self.tracker.process(frame0, frame1, &self.cameras, self.frame_number);

    if self.stationary.check(self.tracker.get_tracks()) {
      self.kalman_filter.update_zero_velocity(self.kf_noise_zero_velocity);
    }

    self.visual_update.process(
      &mut self.kalman_filter,
      self.tracker.get_tracks(),
      [&self.cameras[0], &self.cameras[1]],
      &self.pose_trail_frame_numbers,
    );

    self.kalman_filter.augment_pose();
    self.pose_trail_frame_numbers.push_back(*self.pose_trail_frame_numbers.back().unwrap());
    while self.pose_trail_frame_numbers.len() > self.pose_trail_len {
      self.pose_trail_frame_numbers.pop_front();
    }

    Ok(())
  }

  fn process_imu(&mut self, time: f64, gyroscope: Vector3d, accelerometer: Vector3d) {
    self.kalman_filter.predict(time, gyroscope, accelerometer);
  }

  fn update_debug_data_3d(&self) {
    let indices: Vec<_> = (0..self.pose_trail_frame_numbers.len()).collect();
    let d = &mut DEBUG_DATA_3D.lock().unwrap();
    self.kalman_filter.get_imu_to_worlds(&indices, &mut d.pose_trail);
  }
}
