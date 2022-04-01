use crate::all::*;

pub const VISUALIZE_FEATURES: bool = false;
pub const VISUALIZE_MASK: bool = false;
pub const VISUALIZE_PYRAMID: bool = false;
pub const VISUALIZE_OPTICAL_FLOW: bool = true;

pub struct VisualizeArgs<'a> {
  pub buffer: &'a mut Vec<u32>,
  pub frames: &'a [Frame],
  pub video_w: usize,
  pub video_h: usize,
  pub buffer_w: usize,
  pub buffer_h: usize,
}

#[inline(always)]
fn draw_pixel(args: &mut VisualizeArgs, p: &Vector2i, v: u32) {
  if p[0] < 0 || p[0] >= args.buffer_w as i32 || p[0] >= args.video_w as i32 { return }
  if p[1] < 0 || p[1] >= args.buffer_h as i32 || p[1] >= args.video_h as i32 { return }
  args.buffer[p[1] as usize * args.buffer_w + p[0] as usize] = v;
}

fn draw_square(args: &mut VisualizeArgs, p: &Vector2i, v: u32, r: i32) {
  for z in (-r)..(r+1) {
    draw_pixel(args, &(p + Vector2i::new(z, -r)), v);
    draw_pixel(args, &(p + Vector2i::new(z, r)), v);
    draw_pixel(args, &(p + Vector2i::new(-r, z)), v);
    draw_pixel(args, &(p + Vector2i::new(r, z)), v);
  }
}

fn draw_line(args: &mut VisualizeArgs, mut p0: Vector2i, mut p1: Vector2i, v: u32) {
  let dx = p1[0] - p0[0];
  let dy = p1[1] - p0[1];
  if dx.abs() < dy.abs() {
    if p0[1] > p1[1] { (p0, p1) = (p1, p0); }
    let k = dx as f32 / dy as f32;
    for y in p0[1] ..= p1[1] {
      let x = p0[0] + (k * (y - p0[1]) as f32).round() as i32;
      draw_pixel(args, &Vector2i::new(x, y), v);
    }
  }
  else {
    if p0[0] > p1[0] { (p0, p1) = (p1, p0); }
    let k = dy as f32 / dx as f32;
    for x in p0[0] ..= p1[0] {
      let y = p0[1] + (k * (x - p0[0]) as f32).round() as i32;
      draw_pixel(args, &Vector2i::new(x, y), v);
    }
  }
}

fn draw_buffer(
  args: &mut VisualizeArgs,
  // Data to be drawn.
  data: &[u8],
  w: usize,
  h: usize,
  // Top-left coordinates of drawing target.
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
  let fc0 = &frame.cameras[0];
  let fc1 = &frame.cameras[1];
  draw_buffer(args, &fc0.data, fc0.width, fc0.height, 0, 0);
  draw_buffer(args, &fc1.data, fc1.width, fc1.height, fc0.width, 0);

  if VISUALIZE_PYRAMID {
    let mut a = [0, 0];
    for (i, level) in fc0.pyramid.levels.iter().enumerate() {
      a[i % 2] += fc0.pyramid.size(i)[i % 2];
      let size = fc0.pyramid.size(i + 1);
      draw_buffer(args, level, size[0], size[1], a[0], a[1]);
    }
  }

  let d = DEBUG_DATA.lock().unwrap();
  if VISUALIZE_MASK {
    for i in 0..d.detection_mask.len() {
      if !d.detection_mask[i] { continue }
      draw_pixel(args, &from_usize(&Vector2usize::new(i % args.video_w, i / args.video_w)), 255 * 255 * 255);
    }
  }
  if VISUALIZE_FEATURES {
    for p in &d.detections {
      draw_square(args, p, 255 * 255, 3);
    }
  }
  if VISUALIZE_OPTICAL_FLOW {
    for (p0, p1) in &d.flow {
      draw_line(args, from_f64(p0), from_f64(p1), 255 * 255);
      draw_square(args, &from_f64(p1), 255 * 255, 3);
    }
  }
  Ok(())
}
