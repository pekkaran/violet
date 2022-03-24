use crate::all::*;

pub struct VisualizeArgs<'a> {
  pub buffer: &'a mut Vec<u32>,
  pub video_w: usize,
  pub video_h: usize,
  pub buffer_w: usize,
  pub buffer_h: usize,
}

fn set_frame_value(args: &mut VisualizeArgs, p: &Pixel, v: u32) {
  args.buffer[p[1] as usize * args.buffer_w + p[0] as usize] = v;
}

pub fn visualize(args: &mut VisualizeArgs) {
  let detections = &DEBUG_DATA.lock().unwrap().detections;
  for p in detections {
    set_frame_value(args, p, 255 * 255);
  }
}
