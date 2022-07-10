// For math we often want to use capital letters to denote matrices.
#![allow(non_snake_case)]

mod all;
mod camera;
mod camera_pinhole;
mod camera_setup;
mod debug;
mod detector;
mod event_loop;
mod frame;
mod image;
mod input;
mod kalman_filter;
mod math;
mod optical_flow;
mod parameters;
mod pyramid;
mod stationary;
mod track;
mod tracker;
mod types;
mod util;
mod video;
mod vio;
mod vio_init;
mod visualize;
mod visualize_3d;

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
  #[clap(flatten)]
  parameter_set: ParameterSet,
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
  *PARAMETER_SET.lock().unwrap() = args.parameter_set;

  let input_folder_path = Path::new(&args.input_folder);
  let mut input = Input::new(&input_folder_path)?;
  let cameras = Camera::load(&input_folder_path)
    .context("Could not load camera setups.")?;

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

  // Start logging after winit setup to skip a specific useless debug print.
  env_logger::Builder::new()
    .filter_level(LevelFilter::Info)
    .format(util::format_log)
    .init();

  let (quit_3d_tx, quit_3d_rx) = mpsc::channel();
  let (quit_2d_tx, quit_2d_rx) = mpsc::channel();
  let mut visualize_3d_handle = None;
  if PARAMETER_SET.lock().unwrap().show_3d {
    visualize_3d_handle = Some(std::thread::spawn(move || {
      run_visualize_3d(quit_3d_rx);
      _ = quit_2d_tx.send(());
    }));
  }

  let mut buffer = vec![];
  let mut args = EventLoopArgs {
    input: &mut input,
    buffer: &mut buffer,
    graphics_context: &mut graphics_context,
    step_mode: false,
    advance: false,
    vio_init: VioInit::new(cameras),
    vio: None,
  };

  event_loop.run_return(move |event, _, mut control_flow| {
    if let Err(err) = handle_event(event, &mut control_flow, &mut args) {
      handle_error(&err);
      *control_flow = ControlFlow::Exit;
    }
    if let Ok(_) = quit_2d_rx.try_recv() {
      *control_flow = ControlFlow::Exit;
    }
  });

  if let Some(visualize_3d_handle) = visualize_3d_handle {
    // Signal to quit 3d visualization thread.
    _ = quit_3d_tx.send(());
    _ = visualize_3d_handle.join();
  }
  Ok(())
}
