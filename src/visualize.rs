use crate::all::*;

pub const VISUALIZE_FEATURES: bool = true;
pub const VISUALIZE_MASK: bool = false;

pub struct VisualizeArgs<'a> {
  pub buffer: &'a mut Vec<u32>,
  pub video_w: usize,
  pub video_h: usize,
  pub buffer_w: usize,
  pub buffer_h: usize,
}

#[inline(always)]
fn draw_pixel(args: &mut VisualizeArgs, p: &Pixel, v: u32) {
  if p[0] < 0 || p[0] > args.buffer_w as i32 || p[0] > args.video_w as i32 { return }
  if p[1] < 0 || p[1] > args.buffer_h as i32 || p[1] > args.video_h as i32 { return }
  args.buffer[p[1] as usize * args.buffer_w + p[0] as usize] = v;
}

fn draw_square(args: &mut VisualizeArgs, p: &Pixel, v: u32, r: i32) {
  for z in (-r)..(r+1) {
    draw_pixel(args, &(p + Pixel::new(z, -r)), v);
    draw_pixel(args, &(p + Pixel::new(z, r)), v);
    draw_pixel(args, &(p + Pixel::new(-r, z)), v);
    draw_pixel(args, &(p + Pixel::new(r, z)), v);
  }
}

pub fn visualize(args: &mut VisualizeArgs) {
  let d = DEBUG_DATA.lock().unwrap();
  if VISUALIZE_MASK {
    for i in 0..d.detection_mask.len() {
      if !d.detection_mask[i] { continue }
      draw_pixel(args, &from_usize(&PixelUsize::new(i % args.video_w, i / args.video_w)), 255 * 255 * 255);
    }
  }
  if VISUALIZE_FEATURES {
    for p in &d.detections {
      draw_square(args, p, 255 * 255, 3);
    }
  }
}
