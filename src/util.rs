pub fn format_log(
  buf: &mut env_logger::fmt::Formatter,
  record: &log::Record,
) -> std::io::Result<()> {
  use std::io::Write;
  let mut style = buf.style();
  use env_logger::fmt::Color::*;
  use log::Level::*;
  style.set_color(match record.level() {
    Error => Red,
    Warn => Rgb(200, 200, 200),
    Info => Green,
    Debug => Magenta,
    Trace => Blue,
  });

  let s = format!("{:30}{}",
    format!("{}:{}",
      record.file().unwrap_or("?"),
      record.line().unwrap_or(0),
    ),
    record.args()
  );
  writeln!(buf, "{}", style.value(s))
}
