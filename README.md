# FusionEKF

A multi-sensor **error-state Kalman filter (ESKF)** for vehicle localization. It fuses a
high-rate **IMU** with **raw GNSS pseudorange and Doppler (tightly coupled)**, **wheel speed**
and **visual odometry**, and is written in modern C++23 on top of Eigen.

Tight coupling means the filter keeps working with fewer than four satellites and rejects
multipath outliers per satellite, which is what makes urban driving work.

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
IMU ──► SampleBuffer<ImuSample> ──► ECEF strapdown + clock model ──► x, P
                                                                     │
RINEX nav ──► ephemeris ──► satellite pos/vel/clock ─┐               ▼
GNSS raw  ──► per-satellite pseudorange, Doppler ────┼─► TC GNSS model ─┐
Wheel     ──► speed + non-holonomic constraints ─────┼─► wheel model  ──┼─► ESKF update
Camera    ──► visual odometry ───────────────────────┘   vision model ──┘   (χ² gate, reset)
```

Error state, expressed in ECEF:

| Block | Dim |
|---|---|
| position, velocity, attitude | 3 + 3 + 3 |
| accelerometer bias, gyroscope bias | 3 + 3 |
| receiver clock bias, clock drift | 1 + 1 |
| inter-system biases (multi-constellation) | N_sys − 1 |
| odometer scale factor | 1 |
| vision pose clones | 6 × N (dynamic) |

## Status

- [x] Build system: CMake presets, warnings as errors, ASan/UBSan, GoogleTest
- [ ] Sensor time types and a time-ordered sample buffer
- [ ] SO(3) / quaternion utilities
- [ ] WGS-84 frames (ECEF / LLA / ENU), GNSS time scales
- [ ] Nominal and error state, ECEF IMU propagation with receiver clock model
- [ ] RINEX 3 parsing, broadcast ephemeris → satellite position / velocity / clock
- [ ] Atmospheric and relativistic corrections, single-point positioning (SPP)
- [ ] Tightly coupled pseudorange + Doppler update, lever arm, per-satellite χ² gating
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
