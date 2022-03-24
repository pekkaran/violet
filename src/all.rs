// NOTE This kind of import-all file isn't a common Rust idiom.

pub use crate::{
  debug::*,
  detector::*,
  event_loop::*,
  input::*,
  tracker::*,
  util::*,
  video::*,
  vio::*,
  visualize::*,
};

pub use {
  std::{
    fs::File,
    io::{BufRead, BufReader, Read},
    path::{Path, PathBuf},
    sync::Mutex,
  },
  log::{debug, error, info, warn, LevelFilter},
  nalgebra::DMatrix,
  anyhow::{anyhow, bail, Context as AnyhowContext, Result},
};

// Eigen-like aliases.
pub type Vector3d = nalgebra::Vector3::<f64>;
pub type Pixel = nalgebra::Vector2::<i32>;
