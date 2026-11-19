# traj

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: GPL-3.0-only" src="https://img.shields.io/badge/license-GPL--3.0--only-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Motion profiles — how to get from here to there without jerking, in one C file.

Placeholder — not written yet.

## What it will be

- Trapezoidal profiles: accelerate, cruise, decelerate.
- S-curve profiles, where the jerk limit matters more than the time.
- Ask it for position, velocity and acceleration at time t. That is the whole
  interface.
- No timers, no threads, no motor. Portable and testable on a host.

## Why

Both stepper drivers in the workshop deliberately stop short of this.
`esp-tmc2209-stepper` says so outright — *no step pulse generation, no ramps*.
`esp-tmc5240-stepper` has a ramp generator in hardware but nothing tells it
what shape to follow. Same gap on the arm and the AGV.

## License

© 2026 JonasAtelier, GPL-3.0-only
