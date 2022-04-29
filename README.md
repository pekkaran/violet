# Violet

A toy stereo visual inertial odometry (VIO) system, based on a Kalman Filter.

## Status

The implementation of the VIO is not yet complete, but some of the sub-components have been tested to work:

- [X] Data input:
  - [X] IMU parsing
  - [X] Video decoding
  - [X] Calibration parsing
- [X] Stereo feature tracker:
  - [X] Feature detection: FAST corners
  - [X] Optical flow: Pyramidal Lucas-Kanade
  - [X] Track spatial distribution enhancement
- [ ] Odometry: Extended Kalman Filter (EKF)
  - [X] Prediction step (the dynamic model)
  - [ ] Update step
  - [ ] Visual feature triangulation
  - [ ] Visual update
  - [ ] Visual update outlier detection
  - [ ] Stationarity update
- [X] Camera models: Pinhole with radial distortion
- [ ] Visualizations
  - [X] Various 2D visualizations
  - [ ] VIO output track visualization, 2D
  - [ ] VIO output track visualization, 3D

Goals:

* Implement various algorithms from scratch to demonstrate and increase my understanding. See for example `src/optical_flow.rs` that implements the pyramidal Lucas-Kanade tracker in all its gnarly details.
* Related to previous, do not depend on any computer vision libraries such as OpenCV.
* Demonstrate plausibility and advantages of using Rust over C++ for this kind of real-time sensor fusion applications. Consequently a goal is to make the implementation run relatively fast.

Non-goals:

* Monocular camera support. For technical reasons, a robust VIO is much easier to implement when a stereo camera setup can be assumed.
* Simultaneous Localization and Mapping (SLAM).
* State-of-the-art accuracy and robustness. For a commercial-grade solution, please see [Spectacular AI](https://www.spectacularai.com/).

## Obtaining test data

The input data format supported by the VIO is the same as described [here](https://github.com/AaltoML/vio_benchmark#benchmark-data-format), except that the calibration is read from a JSON format. A conversion for the [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) dataset exists:

```bash
pip install pyyaml numpy
python scripts/download_euroc.py --fast --case v1-01-easy
```

The converted datasets are saved under `data/benchmark/`. Note that about 10 GB of space is required for converting all the EuRoC datasets.

To implement conversion for other datasets, please examine format of the converted EuRoC datasets.

## Running the VIO

Install Rust, see for example the official [get started](https://www.rust-lang.org/learn/get-started) guide. Clone this git repository and run from its root:

```bash
cargo run -- -i data/benchmark/euroc/v1-01-easy --show-tracks
```

See all the available visualizations and options with:

```bash
cargo run -- --help
```

## License

Licensed under **GPLv3**. Note that the algorithms are heavily based on [HybVIO](https://github.com/SpectacularAI/HybVIO) which is licensed under GPLv3.
