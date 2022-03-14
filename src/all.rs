// NOTE This kind of import-all file isn't a common Rust idiom.

pub use crate::{
  input::*,
  util::*,
};

pub use {
  std::{
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
  },
  log::{debug, error, info, warn, LevelFilter},
  nalgebra::DMatrix,
  // NOTE Masks the language `Result` type.
  anyhow::{anyhow, bail, Context as AnyhowContext, Result},
};

// C++ Eigen-like aliases.
pub type Vector3d = nalgebra::Vector3::<f64>;
