use crate::all::*;

lazy_static! {
  pub static ref PARAMETER_SET: Mutex<ParameterSet> = Mutex::new(ParameterSet::default());
}

#[derive(Debug, Default)]
#[derive(clap::Parser)]
pub struct ParameterSet {
  #[clap(long, default_value = "1")]
  pub frame_sub: usize,
  #[clap(long, default_value = "3")]
  pub lk_levels: usize,
  #[clap(long, default_value = "5")]
  pub lk_iters: usize,
  #[clap(long, default_value = "7")]
  pub lk_win_size: usize,
}
