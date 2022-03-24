mod all;
mod debug;
mod detector;
mod input;
mod event_loop;
mod tracker;
mod util;
mod video;
mod vio;
mod visualize;

use all::*;

#[macro_use] extern crate lazy_static;
use clap::Parser;

use softbuffer::GraphicsContext;
use winit::event_loop::{ControlFlow, EventLoop};
use winit::window::WindowBuilder;
use winit::platform::run_return::EventLoopExtRunReturn;

#[derive(Parser)]
struct Args {
  #[clap(short)]
  input_folder: String,
  // Could the parameters struct be flattened here to allow defining parameters
  // at commandline?
}

fn handle_error(err: &anyhow::Error) {
  for (i, e) in err.chain().enumerate() {
    println!("  {}: {}", i + 1, e);
  }
}

fn main() {
  if let Err(err) = run() {
    handle_error(&err);
  }
}

fn run() -> Result<()> {
  let args = Args::parse();
  let input_folder_path = Path::new(&args.input_folder);
  let mut input = Input::new(&input_folder_path)?;

  let width = 1920;
  let height = 1080;
  let size = winit::dpi::PhysicalSize::new(width, height);
  let mut event_loop = EventLoop::new();
  let window = WindowBuilder::new()
    .with_resizable(false)
    .with_decorations(false)
    .with_min_inner_size(size)
    .with_max_inner_size(size)
    .build(&event_loop)
    .unwrap();
  let mut graphics_context = unsafe { GraphicsContext::new(window) }.unwrap();

  // Start logging after winit setup to skip a debug print.
  env_logger::Builder::new()
    .filter_level(LevelFilter::Info)
    .format(util::format_log)
    .init();

  let mut buffer = vec![];
  let mut args = EventLoopArgs {
    input: &mut input,
    buffer: &mut buffer,
    graphics_context: &mut graphics_context,
    vio: Vio::new(),
    step_mode: false,
    advance: false,
  };

  event_loop.run_return(move |event, _, mut control_flow| {
    if let Err(err) = handle_event(event, &mut control_flow, &mut args) {
      handle_error(&err);
      *control_flow = ControlFlow::Exit;
    }
  });
  Ok(())
}
