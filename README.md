# FusionEKF

A multi-sensor **error-state Kalman filter (ESKF)** for vehicle localization. It fuses a
high-rate **IMU** with **GNSS**, **wheel speed** and **visual odometry**, and is written in
modern C++23 on top of Eigen.

## Design goals

- **Deterministic replay.** Every timestamp comes from the sensor data (`SensorClock`), never
  from the wall clock, so a log replays bit-for-bit.
- **Type-safe time and frames.** `std::chrono` nanosecond time types, so unit mix-ups are
  compile errors.
- **Allocation-free hot path.** Fixed-size Eigen types in IMU propagation and in the update step.
- **Generic measurement models.** Sensors plug in through C++20 concepts, with no virtual
  dispatch in the update loop.
- **Tested numerics.** Unit tests for every component, Jacobians checked against finite
  differences, and filter consistency checked with NEES/NIS.

## Architecture

```
IMU ──► SampleBuffer<ImuSample> ──► propagate (strapdown, F, Q)
                                         │  nominal state x, covariance P (15x15)
GNSS  ─┐                                 ▼
Wheel ─┼─► time-ordered measurement queue ──► model: h(x), H, R ──► ESKF update + reset
Vision ┘
```

Error state (15): position, velocity, attitude, accelerometer bias, gyroscope bias.

## Status

- [x] Build system: CMake presets, warnings as errors, ASan/UBSan, GoogleTest
- [ ] Sensor time types and a time-ordered sample buffer
- [ ] SO(3) / quaternion utilities
- [ ] Nominal and error state, IMU propagation
- [ ] GNSS position/velocity update with lever arm and χ² gating
- [ ] Wheel speed + non-holonomic constraints
- [ ] Visual odometry update (stochastic cloning)
- [ ] Out-of-order / delayed measurement handling
- [ ] Simulator and log replay, evaluation (RMSE, NEES)
- [ ] Multithreaded sensor pipeline

## Build & test

You need: a C++23 compiler (Clang 17+ or GCC 13+), CMake ≥ 3.25 and Ninja. Eigen 3.4 and
GoogleTest are fetched automatically.

```bash
cmake --preset debug          # Debug + AddressSanitizer + UBSan
cmake --build --preset debug
ctest --preset debug
```

Use `--preset release` for an optimized build.

## Layout

```
include/fusion/   public headers (core/, sensors/, filter/, models/)
src/              implementations
tests/            GoogleTest unit tests
cmake/            warnings, sanitizers, dependencies
```
