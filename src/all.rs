// NOTE This kind of import-all file isn't a common Rust idiom.

pub use crate::{
  event_loop::*,
  input::*,
  tracker::*,
  util::*,
  video::*,
  vio::*,
};

pub use {
  std::{
    fs::File,
    io::{BufRead, BufReader, Read},
    path::{Path, PathBuf},
  },
  log::{debug, error, info, warn, LevelFilter},
  nalgebra::DMatrix,
  anyhow::{anyhow, bail, Context as AnyhowContext, Result},
};

// Eigen-like aliases.
pub type Vector3d = nalgebra::Vector3::<f64>;
