use crate::all::*;

pub const VISUALIZE_FEATURES: bool = true;
pub const VISUALIZE_MASK: bool = false;
pub const VISUALIZE_PYRAMID: bool = false;

pub struct VisualizeArgs<'a> {
  pub buffer: &'a mut Vec<u32>,
  pub frames: &'a [Frame],
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

fn draw_buffer(
  args: &mut VisualizeArgs,
  data: &[u8],
  w: usize,
  h: usize,
  ax: usize,
  ay: usize,
) {
  for y in 0..h {
    if y + ay >= args.buffer_h { continue }
    for x in 0..w {
      if x + ax >= args.buffer_w { continue }
      let gray = data[y * w + x] as u32;
      args.buffer[(y + ay) * args.buffer_w + x + ax] = gray | (gray << 8) | (gray << 16);
    }
  }
}

pub fn visualize(args: &mut VisualizeArgs) -> Result<()> {
  let frame = args.frames.iter().last().ok_or(anyhow!("Cannot visualize before processing the first frame."))?;
  draw_buffer(args, &frame.data, frame.width, frame.height, 0, 0);

  if VISUALIZE_PYRAMID {
    let mut a = [0, 0];
    for (i, level) in frame.pyramid.levels.iter().enumerate() {
      let dimension = i % 2;
      a[dimension] += frame.pyramid.size(i)[dimension];
      let size = frame.pyramid.size(i + 1);
      draw_buffer(args, level, size[0], size[1], a[0], a[1]);
    }
  }

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
  Ok(())
}
