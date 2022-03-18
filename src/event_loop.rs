use crate::all::*;

use softbuffer::GraphicsContext;
use winit::event::{ElementState, Event, KeyboardInput, VirtualKeyCode, WindowEvent};
use winit::event_loop::ControlFlow;
use winit::window::Window;

pub struct EventLoopArgs<'a> {
  pub input: &'a mut Input,
  pub buffer: &'a mut Vec<u32>,
  pub graphics_context: &'a mut GraphicsContext<Window>,
}

pub fn handle_event(
  event: Event<()>,
  control_flow: &mut ControlFlow,
  args: &mut EventLoopArgs,
) -> Result<()> {
  let (window_width, window_height) = {
    let size = args.graphics_context.window().inner_size();
    (size.width as usize, size.height as usize)
  };
  if args.buffer.is_empty() {
    *args.buffer = vec![0; window_width * window_height];
  }
  assert_eq!(window_width * window_height, args.buffer.len());

  match event {
    Event::RedrawRequested(window_id) if window_id == args.graphics_context.window().id() => {
      args.graphics_context.set_buffer(&args.buffer, window_width as u16, window_height as u16);
    },
    Event::WindowEvent {
      event,
      window_id,
    } => {
      if event == WindowEvent::CloseRequested && window_id == args.graphics_context.window().id() {
        *control_flow = ControlFlow::Exit;
      }
      match event {
        WindowEvent::KeyboardInput {
          input: KeyboardInput {
            state: ElementState::Pressed,
            virtual_keycode: Some(keycode),
            scancode: _,
            ..
          },
          is_synthetic: _,
          device_id: _,
        } => {
          match keycode {
            VirtualKeyCode::Escape | VirtualKeyCode::Q => {
              *control_flow = ControlFlow::Exit;
            },
            _ => {}, // Other keys.
          }
        },
        _ => {}, // Other window events.
      }
    },
    _ => {}, // Other events.
  }

  match args.input.next()? {
    Some(data) => {
      match data {
        #[allow(unused_variables)]
        InputData::Gyroscope { time, v } => {
          // dbg!(v);
        },
        #[allow(unused_variables)]
        InputData::Accelerometer { time, v } => {
          // dbg!(v);
        },
        #[allow(unused_variables)]
        InputData::Frame(frame) => {
          for i in 0..frame.video.height {
            if i >= window_height { continue }
            for j in 0..frame.video.width {
              if j >= window_width { continue }
              let gray = frame.video.data[i * frame.video.width + j] as u32;
              args.buffer[i * window_width + j] = gray | (gray << 8) | (gray << 16);
            }
          }
          args.graphics_context.window().request_redraw();
        },
      }
    },
    None => *control_flow = ControlFlow::Exit,
  }
  Ok(())
}
