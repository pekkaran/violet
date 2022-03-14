mod all;
mod input;
mod util;

use all::*;

use clap::Parser;

#[derive(Parser)]
struct Args {
  #[clap(short)]
  input_folder: String,
}

fn main() {
  if let Err(err) = run() {
    for (i, e) in err.chain().enumerate() {
      println!("  {}: {}", i + 1, e);
    }
  }
}

fn run() -> Result<()> {
  env_logger::Builder::new()
    .filter_level(LevelFilter::Info)
    .format(util::format_log)
    .init();

  let args = Args::parse();
  let input_folder_path = Path::new(&args.input_folder);
  let mut input = Input::new(&input_folder_path.join("data.jsonl"))?;

  while let Some(data) = input.next()? {
    match data {
      #[allow(unused_variables)]
      InputData::Gyroscope { time, v } => {
      },
      #[allow(unused_variables)]
      InputData::Accelerometer { time, v } => {
      },
    }
  }
  Ok(())
}
