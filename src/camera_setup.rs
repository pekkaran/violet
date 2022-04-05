use crate::all::*;

#[derive(Deserialize)]
#[allow(non_snake_case)]
pub struct CameraSetup {
  pub focalLengthX: f64,
  pub focalLengthY: f64,
  pub principalPointX: f64,
  pub principalPointY: f64,
  pub distortionCoefficients: Vec<f64>,
  pub imuToCamera: Vec<Vec<f64>>,
  pub model: String,
  // TODO Should enable these but my test data is missing them.
  //      They are needed to scale camera intrinsics if input video is scaled.
  // pub imageWidth: usize,
  // pub imageHeight: usize,
}

#[derive(Deserialize)]
pub struct CameraSetupRoot {
  pub cameras: Vec<CameraSetup>,
}

const MAX_PARENT_DIRECTORY_HEIGHT: usize = 1;
const SETUP_FILE_NAME: &'static str = "calibration.json";

impl Camera {
  pub fn load(path: &Path) -> Result<Vec<Camera>> {
    let path = path.to_path_buf();
    for _ in 0..(MAX_PARENT_DIRECTORY_HEIGHT + 1) {
      let setup_path = path.join(SETUP_FILE_NAME);
      if setup_path.exists() {
        return parse_setup(&setup_path);
      }
    }
    bail!("Failed to find a {}.", SETUP_FILE_NAME);
  }
}

fn parse_setup(path: &Path) -> Result<Vec<Camera>> {
  let s = std::fs::read_to_string(path)
    .context(format!("Failed to read file {}.", path.display()))?;
  let root: CameraSetupRoot = serde_json::from_str(&s)
    .context(format!("Failed to parse {}.", path.display()))?;
  root.cameras.into_iter()
    .map(|x| convert_setup(x))
    .collect::<Result<Vec<_>>>()
}

fn convert_setup(d: CameraSetup) -> Result<Camera> {
  let camera_matrix = Matrix3d::new(
    d.focalLengthX, 0., d.principalPointX,
    0., d.focalLengthY, d.principalPointY,
    0., 0., 1.
  );
  let kind = convert_model(&d.model)?;
  let model = match kind {
    CameraKind::Pinhole => Box::new(PinholeModel::new(camera_matrix, d.distortionCoefficients)),
    CameraKind::KannalaBrandt4 => unimplemented!(),
  };
  Ok(Camera {
    kind,
    imu_to_camera: Matrix4d::from_iterator(d.imuToCamera.into_iter().flatten()).transpose(),
    model,
  })
}

fn convert_model(model: &str) -> Result<CameraKind> {
  match model {
    "pinhole" => Ok(CameraKind::Pinhole),
    "kannala-brandt4" => Ok(CameraKind::KannalaBrandt4),
    _ => bail!("Unknown camera model {}.", &model),
  }
}
