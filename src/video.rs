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

    // Probe video frame dimensions.
    let cmd_str = format!("ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 {}", path);
    // `bash -c` splits the command into tokens.
    let output = Command::new("bash").args(["-c", &cmd_str])
      .output()?
      .stdout;
    let resolution: Vec<usize> = std::str::from_utf8(&output)
      .context(format!("Could not parse ffprobe resolution string: {:?}", output))?
      .split("x")
      .map(|x| x.trim().parse::<usize>().unwrap())
      .collect();
    assert_eq!(resolution.len(), 2);

    let cmd_str = format!("ffmpeg -i {} -f rawvideo -pix_fmt gray - 2>/dev/null", path);
    let child = Command::new("bash").args(["-c", &cmd_str])
      .stdout(Stdio::piped())
      .spawn()?;
    Ok(VideoInput {
      child_stdout: child.stdout.unwrap(),
      video_frame: VideoFrame {
        data: vec![],
        width: resolution[0],
        height: resolution[1],
      },
    })
  }

  pub fn read(&mut self) -> Result<&VideoFrame> {
    let n = self.video_frame.width * self.video_frame.height;
    if self.video_frame.data.len() != n {
      self.video_frame.data.resize(n, 0);
    }
    self.child_stdout.read_exact(&mut self.video_frame.data)
      .context("Reading bytes from video input failed.")?;
    Ok(&self.video_frame)
  }
}
