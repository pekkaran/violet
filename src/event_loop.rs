use crate::all::*;

use softbuffer::GraphicsContext;
use winit::event::{ElementState, Event, KeyboardInput, VirtualKeyCode, WindowEvent};
use winit::event_loop::ControlFlow;
use winit::window::Window;

pub fn handle_event(
  event: Event<()>,
  input: &mut Input,
  buffer: &mut Vec<u32>,
  graphics_context: &mut GraphicsContext<Window>,
  control_flow: &mut ControlFlow,
) -> Result<()> {
  let (window_width, window_height) = {
    let size = graphics_context.window().inner_size();
    (size.width as usize, size.height as usize)
  };
  if buffer.is_empty() {
    *buffer = vec![0; window_width * window_height];
  }
  assert_eq!(window_width * window_height, buffer.len());

  match event {
    Event::RedrawRequested(window_id) if window_id == graphics_context.window().id() => {
      graphics_context.set_buffer(&buffer, window_width as u16, window_height as u16);
    },
    Event::WindowEvent {
      event,
      window_id,
    } => {
      if event == WindowEvent::CloseRequested && window_id == graphics_context.window().id() {
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

  match input.next()? {
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
              buffer[i * window_width + j] = gray | (gray << 8) | (gray << 16);
            }
          }
          graphics_context.window().request_redraw();
        },
      }
    },
    None => *control_flow = ControlFlow::Exit,
  }
  Ok(())
}
