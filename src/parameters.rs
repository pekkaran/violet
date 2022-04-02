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

  // Pyramidal Lucas-Kanade feature tracker.
  #[clap(long, default_value = "3")]
  pub lk_levels: usize,
  #[clap(long, default_value = "10")]
  pub lk_iters: usize,
  #[clap(long, default_value = "7")]
  pub lk_win_size: usize,

  // Visualizations.
  #[clap(long)]
  pub show_features: bool,
  #[clap(long)]
  pub show_mask: bool,
  #[clap(long)]
  pub show_pyramid: bool,
  #[clap(long)]
  pub show_flow0: bool,
  #[clap(long)]
  pub show_flow1: bool,
  #[clap(long)]
  pub show_flow2: bool,
}
