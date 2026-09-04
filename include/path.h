/* path.h - waypoints to a smooth path, and a pure-pursuit tracker.
 *
 * C99, no dependencies beyond libm, no malloc, no globals.
 * Link with -lm.
 */
#ifndef PATH_H
#define PATH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Return codes. */
#define PATH_OK    0
#define PATH_EARG (-1)   /* bad argument (NULL, not finite, or it will not fit) */

#define PATH_MAX_WAYPOINTS 64
#define PATH_MAX_POINTS    512     /* the resampled path; PATH_MAX_WAYPOINTS x spacing must fit */

typedef struct { double x, y; } path_point;

typedef struct {
    path_point pts[PATH_MAX_POINTS];   /* the built path, evenly spaced along its length */
    double     s[PATH_MAX_POINTS];     /* arc length at each point; s[0] = 0 */
    int        n;                      /* points in use */
} path;

/* Fit a Catmull-Rom spline through the waypoints (it passes through every one), then
 * resample it every `spacing` metres. Two waypoints give a straight line. PATH_EARG if
 * fewer than two, more than PATH_MAX_WAYPOINTS, non-finite, spacing <= 0, or the result
 * would need more than PATH_MAX_POINTS.
 *
 * The step is close to `spacing`, not exactly it: each stretch between two waypoints is
 * cut into a whole number of equal steps, so that every waypoint is itself a point on the
 * built path. A stretch of 2.2 m at a spacing of 0.5 m becomes four steps of 0.55 m. */
int path_build(path *p, const path_point *waypoints, int n, double spacing);

double path_length(const path *p);

/* The point at arc length s, clamped to [0, length]. */
int path_at(const path *p, double s, path_point *out);

/* Arc length of the point on the path nearest to q. */
int path_nearest(const path *p, path_point q, double *s);

/* Pure pursuit. Given where the robot is (x, y, theta in the world frame, theta
 * counter-clockwise from +x) and how far ahead to aim, returns the turn rate wz
 * (rad/s, counter-clockwise) that steers it onto the path at forward speed v.
 * done becomes 1 when the nearest point is within `lookahead` of the end. Pair the
 * result with mob: twist = { v, 0, wz }. */
int path_pursue(const path *p, double x, double y, double theta,
                double lookahead, double v, double *wz, int *done);

#ifdef __cplusplus
}
#endif
#endif /* PATH_H */
