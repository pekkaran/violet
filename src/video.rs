use crate::all::*;

use std::process::{ChildStdout, Command, Stdio};

pub struct VideoInput {
  child_stdout: ChildStdout,
  video_frame: VideoFrame,
}

pub struct VideoFrame {
  pub data: Vec<u8>,
  pub width: usize,
  pub height: usize,
}

impl VideoInput {
  pub fn new(path: &Path) -> Result<VideoInput> {
    let path = path.to_str().ok_or(anyhow!("Failed to parse video path."))?;
    let cmd_str = format!("ffmpeg -i {} -f rawvideo -pix_fmt gray - 2>/dev/null", path);
    // `bash -c` splits the command into tokens.
    let child = Command::new("bash").args(["-c", &cmd_str])
      .stdout(Stdio::piped())
      .spawn()?;
    Ok(VideoInput {
      child_stdout: child.stdout.unwrap(),
      video_frame: VideoFrame {
        data: vec![],
        width: 0,
        height: 0,
      },
    })
  }

  pub fn read(&mut self) -> Result<&VideoFrame> {
    // TODO Probe dimensions from the video file.
    let width = 752;
    let height = 480;
    let n = width * height;
    if self.video_frame.data.len() != n {
      self.video_frame.data.resize(n, 0);
    }
    self.child_stdout.read_exact(&mut self.video_frame.data)
      .context("Reading bytes from video input failed.")?;
    self.video_frame.width = width;
    self.video_frame.height = height;
    Ok(&self.video_frame)
  }
}
