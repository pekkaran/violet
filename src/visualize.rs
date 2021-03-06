use crate::all::*;

pub struct VisualizeArgs<'a> {
  pub buffer: &'a mut Vec<u32>,
  pub frames: &'a [Frame],
  pub video_w: usize,
  pub video_h: usize,
  pub buffer_w: usize,
  pub buffer_h: usize,
}

#[inline(always)]
fn draw_pixel(args: &mut VisualizeArgs, p: Vector2i, v: u32) {
  if p[0] < 0 || p[0] >= args.buffer_w as i32 { return }
  if p[1] < 0 || p[1] >= args.buffer_h as i32 { return }
  args.buffer[p[1] as usize * args.buffer_w + p[0] as usize] = v;
}

fn draw_square(args: &mut VisualizeArgs, p: Vector2i, v: u32, r: i32) {
  for z in (-r)..(r+1) {
    draw_pixel(args, p + Vector2i::new(z, -r), v);
    draw_pixel(args, p + Vector2i::new(z, r), v);
    draw_pixel(args, p + Vector2i::new(-r, z), v);
    draw_pixel(args, p + Vector2i::new(r, z), v);
  }
}

#[allow(dead_code)]
fn draw_line(args: &mut VisualizeArgs, mut p0: Vector2i, mut p1: Vector2i, v: u32) {
  let dx = p1[0] - p0[0];
  let dy = p1[1] - p0[1];
  if dx.abs() < dy.abs() {
    if p0[1] > p1[1] { std::mem::swap(&mut p0, &mut p1); }
    let k = dx as f32 / dy as f32;
    for y in p0[1] ..= p1[1] {
      let x = p0[0] + (k * (y - p0[1]) as f32).round() as i32;
      draw_pixel(args, Vector2i::new(x, y), v);
    }
  }
  else {
    if p0[0] > p1[0] { std::mem::swap(&mut p0, &mut p1); }
    let k = dy as f32 / dx as f32;
    for x in p0[0] ..= p1[0] {
      let y = p0[1] + (k * (x - p0[0]) as f32).round() as i32;
      draw_pixel(args, Vector2i::new(x, y), v);
    }
  }
}

fn draw_buffer(
  args: &mut VisualizeArgs,
  image: &Image,
  // Top-left coordinates of drawing target.
  ax: usize,
  ay: usize,
) {
  let w = image.width;
  let h = image.height;
  for y in 0..h {
    if y + ay >= args.buffer_h { continue }
    for x in 0..w {
      if x + ax >= args.buffer_w { continue }
      let gray = image.data[y * w + x] as u32;
      args.buffer[(y + ay) * args.buffer_w + x + ax] = gray | (gray << 8) | (gray << 16);
    }
  }
}

#[allow(dead_code)]
fn draw_scaled(
  args: &mut VisualizeArgs,
  image: &Image,
  s: f64,
  // Top-left coordinates of drawing target.
  ax: usize,
  ay: usize,
) {
  assert!(s > 0.);
  let is = 1. / s;
  let w = 1 + (s * (image.width - 1) as f64).floor() as usize;
  let h = 1 + (s * (image.height - 1) as f64).floor() as usize;
  for y in 0..h {
    if y + ay >= args.buffer_h { continue }
    let isy = is * y as f64;
    for x in 0..w {
      if x + ax >= args.buffer_w { continue }
      let isx = is * x as f64;
      let gray = bilinear(image, Vector2d::new(isx, isy)).round() as u32;
      args.buffer[(y + ay) * args.buffer_w + x + ax] = gray | (gray << 8) | (gray << 16);
    }
  }
}

pub fn visualize(args: &mut VisualizeArgs) -> Result<()> {
  // Clear buffer.
  for y in 0..args.buffer_h {
    for x in 0..args.buffer_w {
      args.buffer[y * args.buffer_w + x] = 0;
    }
  }

  let frame = args.frames.iter().last().ok_or(anyhow!("Cannot visualize before processing the first frame."))?;
  let im0 = &frame.cameras[0].image;
  let im1 = &frame.cameras[1].image;
  draw_buffer(args, &im0, 0, 0);
  draw_buffer(args, &im1, im0.width, 0);

  let d = DEBUG_DATA.lock().unwrap();
  let p = PARAMETER_SET.lock().unwrap();
  let mut ax = 0;
  for (image, s) in &d.images {
    draw_scaled(args, &image, *s, ax, im0.height);
    ax += (s * image.width as f64) as usize;
  }

  if p.show_pyramid {
    let mut a = [0, 0];
    for (i, level) in frame.cameras[0].pyramid.levels.iter().enumerate() {
      a[i % 2] += level.size(i % 2);
      draw_buffer(args, &level, a[0], a[1]);
    }
  }

  if p.show_mask {
    for i in 0..d.detection_mask.len() {
      if !d.detection_mask[i] { continue }
      draw_pixel(args, from_usize(Vector2usize::new(i % args.video_w, i / args.video_w)), 255 * 255 * 255);
    }
  }

  if p.show_features {
    for feature in &d.detections {
      draw_square(args, from_f64(feature.point), 255 * 255, 3);
    }
  }


  let a = [ Vector2i::new(0, 0), Vector2i::new(args.video_w as i32, 0) ];
  if p.show_tracks {
    let blue = 0;
    for track in &d.tracks {
      for i in 1..track.points.len() {
        let length = 20;
        if i >= length { continue }
        let n = track.points.len() - i;
        let red = 255 * (length - i) / length;
        let green = 255 * i / length;
        let color = blue | ((green as u32) << 8) | ((red as u32) << 16);
        let tp1 = &track.points[n].coordinates;
        let tp0 = &track.points[n - 1].coordinates;
        for k in 0..2 {
          if i == 1 {
            draw_square(args, &from_f64(tp1[k]) + a[k], color, 3);
          }
          draw_line(args, from_f64(tp0[k]) + a[k], from_f64(tp1[k]) + a[k], color);
        }
      }
    }
  }

  if [p.show_flow0, p.show_flow1, p.show_flow2].iter().map(|x| *x as usize).sum::<usize>() > 1 {
    warn!("Only one optical flow visualization is supported at a time.");
  }
  // TODO Use ids to look up previous coordinates.
  /*
  if p.show_flow0 {
    for (p0, p1) in &d.flow {
      draw_line(args, from_f64(p0), from_f64(p1), 255 * 255);
      draw_square(args, &from_f64(p1), 255 * 255, 3);
    }
  }
  */
  let ax = Vector2d::new(im0.width as f64, 0.);
  if p.show_flow1 || p.show_flow2 {
    for (f0, f1) in d.flow0.iter().zip(d.flow1.iter()) {
      let p0 = f0.point;
      let p1 = f1.point + ax;
      // Could randomize a color for each track.
      draw_line(args, from_f64(p0), from_f64(p1), 255 * 255);
      draw_square(args, from_f64(p0), 255 * 255, 3);
      draw_square(args, from_f64(p1), 255 * 255, 3);
      draw_square(args, from_f64(p0 + ax), 255 * 255 * 255, 3);
    }
  }

  if p.show_epipolar {
    let mut rng = thread_rng();
    for e in &d.epipolar {
      let color: [u8; 3] = [rng.gen(), rng.gen(), rng.gen()];
      let color = (color[0] as u32) | ((color[1] as u32) << 8) | ((color[2] as u32) << 16);
      draw_square(args, from_f64(e.p0), color, 3);
      // if let Some(p1_initial) = e.p1_initial {
      //   draw_square(args, from_f64(p1_initial + ax), white, 3);
      //   draw_line(args, from_f64(p1_initial + ax),  from_f64(e.p1 + ax), white);
      // }
      draw_square(args, from_f64(e.p1 + ax), color, 3);
      for i in 1..e.curve1.len() {
        draw_line(args, from_f64(e.curve1[i - 1] + ax),  from_f64(e.curve1[i] + ax), color);
      }
    }
  }
  Ok(())
}
