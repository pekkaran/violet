use crate::all::*;

lazy_static! {
  pub static ref PARAMETER_SET: Mutex<ParameterSet> = Mutex::new(ParameterSet::default());
}

#[derive(Debug, Default)]
#[derive(clap::Parser)]
pub struct ParameterSet {
  #[clap(long, default_value = "1")]
  pub frame_sub: usize,

  // Tracker module.
  #[clap(long, default_value = "400")]
  pub max_tracks: usize,

  // (Extended) Kalman Filter.
  #[clap(long, default_value = "1e-3")]
  pub kf_noise_a: f64,
  #[clap(long, default_value = "1e-4")]
  pub kf_noise_g: f64,
  #[clap(long, default_value = "1e-1")]
  pub kf_noise_vel: f64,
  #[clap(long, default_value = "1e-5")]
  pub kf_noise_pos: f64,
  #[clap(long, default_value = "1e-2")]
  pub kf_noise_ori: f64,
  #[clap(long, default_value = "1e-3")]
  pub kf_noise_bga: f64,
  #[clap(long, default_value = "1e-6")]
  pub kf_noise_baa: f64,

  // TODO Use another parameter struct with the clap flattening option?
  // Pyramidal Lucas-Kanade feature tracker.
  #[clap(long, default_value = "3")]
  pub lk_levels: usize,
  #[clap(long, default_value = "10")]
  pub lk_iters: usize,
  #[clap(long, default_value = "7")]
  pub lk_win_size: usize,
  #[clap(long, default_value = "0.1")]
  pub lk_term: f64,
  #[clap(long, default_value = "1e-4")]
  pub lk_min_eig: f64,
  #[clap(long, default_value = "2")]
  pub lk_epipolar_max_dist: f64,

  #[clap(long, default_value = "20")]
  pub pose_trail_len: usize,
  #[clap(long, default_value = "9.81")]
  pub gravity: f64,

  // Visualizations.
  #[clap(long)]
  pub show_features: bool,
  #[clap(long)]
  pub show_mask: bool,
  #[clap(long)]
  pub show_pyramid: bool,
  #[clap(long)]
  pub show_tracks: bool,
  #[clap(long)]
  pub show_flow0: bool,
  #[clap(long)]
  pub show_flow1: bool,
  #[clap(long)]
  pub show_flow2: bool,
  #[clap(long)]
  pub show_epipolar: bool,
}
