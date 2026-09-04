# path

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Waypoints to a smooth path, and a tracker that follows it, in one C file.

You have a rover and a handful of points you want it to visit. Two jobs sit
between those and the motors:

- **Build** — turn the corners of a waypoint list into a smooth line, evenly
  sampled, with the distance along it worked out.
- **Pursue** — given where the robot is now, work out how hard to turn to get
  back onto that line and stay on it.

- One header, one source file. Drop them into any C project.
- Only needs `libm`. No `malloc` — works fine on a microcontroller.
- Fixed-size storage: 64 waypoints in, 512 path points out, no allocation.

## From waypoints to wheel speeds

`path` gives you a turn rate. [`mob`](../mob) turns that into wheel commands.
They share the same frames and nothing else — neither library needs the other.

```c
path_point route[] = { { 0, 0 }, { 2, 0 }, { 4, 2 }, { 6, 2 } };
path p;

path_build(&p, route, 4, 0.1);                 /* once, at the start */

while (!done) {
    double wz;
    path_pursue(&p, pose.x, pose.y, pose.theta, /* where the robot is  */
                1.0, 0.6,                       /* lookahead, speed    */
                &wz, &done);                    /* how hard to turn    */

    mob_twist want = { 0.6, 0.0, wz };
    mob_inverse(&base, &want, wheels);          /* wheels -> the motors */
}
```

Three calls: build the path once, then pursue and drive on every tick. Where
`pose` comes from is your problem — `mob_integrate` off the encoders is the
cheap answer, and it drifts.

## Try it

```sh
make check    # run the tests
make run      # drive a rover round an S-bend
```

## Picking the lookahead

The lookahead is the one number you have to tune. The tracker aims at the point
that far ahead on the path and turns onto it, so it decides everything about how
the robot behaves.

**Too long and it cuts every corner.** Aiming a long way ahead means the arc it
drives leaves the path in the middle of a bend and rejoins it later.

**Too short and it chases.** The arc becomes tight and the turn rate large, so
every small heading error produces a big correction. In the perfect simulation
in `examples/follow.c` that still tracks beautifully; on a real robot, where the
pose is a tick old and the wheels slip, it is what makes a rover weave down a
straight line.

These are measured from the S-bend example, a 0.6 m/s rover on a path with two
2 m swings in it:

| lookahead | worst distance off the line | what you get |
|-----------|-----------------------------|--------------|
| 0.2 m | 0.004 m | glued to the path; the first thing to weave on real hardware |
| 1.0 m | 0.088 m | a good place to start |
| 2.0 m | 0.406 m | corners visibly cut |
| 4.0 m | 0.970 m | it is driving its own path now |

**Start at about twice the wheelbase** — 1 m for a 0.5 m rover — then shorten it
until the robot starts to weave, and back off. If you drive at very different
speeds, scale it with speed rather than picking one number.

## The functions

```c
int    path_build(&path, waypoints, n, spacing);   /* corners -> a smooth line */
double path_length(&path);
int    path_at(&path, s, &point);                  /* the point s metres along */
int    path_nearest(&path, point, &s);             /* how far along the nearest bit is */
int    path_pursue(&path, x, y, theta, lookahead, v, &wz, &done);
```

| call | what it does |
|------|--------------|
| `path_build` | fits a Catmull-Rom spline through your waypoints and resamples it evenly |
| `path_length` | how long the built path is, in metres |
| `path_at` | the point at a given distance along; clamped at both ends |
| `path_nearest` | the distance along of the closest point to somewhere — your cross-track error, if you subtract |
| `path_pursue` | the turn rate that steers the robot onto the path, and whether it has arrived |

Every one returns `PATH_OK` or `PATH_EARG`, except `path_length`, which has
nothing to fail at and returns `0` for a `NULL` path. `PATH_EARG`
means a `NULL` pointer, a number that is not finite, fewer than two waypoints or
more than `PATH_MAX_WAYPOINTS`, a `spacing` or `lookahead` that is not positive,
or a path that would need more than `PATH_MAX_POINTS` points. A build that fails
leaves the path empty rather than half-filled.

The spline passes through every waypoint you give it — the resampled path always
has a point sitting exactly on each one. The step between points is *close to*
`spacing`, not exactly it: each stretch between two waypoints is divided into a
whole number of equal steps, so 2.2 m at a spacing of 0.5 m becomes four steps of
0.55 m. That is what keeps the waypoints on the path.

`done` goes to `1` when the nearest point is within one lookahead of the end,
which is a lookahead *before* the robot gets there. Pure pursuit has nothing left
to aim at past the end of the path, so stop the robot yourself when `done` goes
up, or add a waypoint beyond where you actually want to stop.

## Units and frames

Everything is metres, radians and seconds. Waypoints and the pose are in the
world frame; `wz` comes back in the body frame, x out of the nose and y out of
the left side, which is what `mob` wants.

| name | meaning | unit |
|------|---------|------|
| `x`, `y` | where the robot is in the world | m |
| `theta` | which way it points, `0` is along world +x, counter-clockwise | rad |
| `s` | how far along the path, from `0` at the first waypoint | m |
| `spacing` | gap between the built path's points | m |
| `lookahead` | how far ahead on the path to aim | m |
| `v` | forward speed you are driving at | m/s |
| `wz` | turning, counter-clockwise seen from above | rad/s |

`theta` may be wrapped or not, whatever your odometry hands you.

## What it does not do

- No obstacle avoidance. It follows the line you gave it, through whatever is
  standing there.
- No speed planning. You pass `v` in and it is used as given — no slowing for
  corners, no acceleration limits. That is [`traj`](../traj)'s job.
- No reversing. `v` is meant to be positive; pure pursuit steers a robot that is
  driving forwards.
- No closed loops. The path is open, and it ends at the last waypoint. For a
  circuit, repeat the first waypoint at the end and rebuild.
- No limits on the output. If the geometry asks for 4 rad/s, that is what you
  get. Clamp it to what your base can actually do.
- It does not know where the robot is. You tell it, every tick, and the quality
  of that answer sets the quality of everything above.

## License

MIT. See [LICENSE](LICENSE).
