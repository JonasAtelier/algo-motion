#include "path.h"
#include <math.h>

/* Sub-steps per spline segment when measuring its arc length. Twenty chords are
 * within a millimetre of the true length for any curve a rover can drive. */
#define SUB 20

/* ---- the spline ------------------------------------------------------ */

static int fin(double v)
{
    return isfinite(v) != 0;
}

/* Uniform Catmull-Rom, tension 0.5, in one axis. */
static double cr(double p0, double p1, double p2, double p3, double t)
{
    return 0.5 * ((2.0 * p1) +
                  (-p0 + p2) * t +
                  (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t * t +
                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t * t * t);
}

/* The point at t in [0, 1] along segment i, the stretch from waypoint i to i+1.
 * A Catmull-Rom segment needs a waypoint either side of it for the tangents; at
 * the two ends there is none, so the end waypoint stands in for itself. That is
 * what makes the curve start and finish exactly on the waypoints you gave. */
static path_point seg_at(const path_point *w, int n, int i, double t)
{
    const path_point *a = &w[i > 0 ? i - 1 : 0];
    const path_point *b = &w[i];
    const path_point *c = &w[i + 1];
    const path_point *d = &w[i + 2 < n ? i + 2 : n - 1];
    path_point q;

    q.x = cr(a->x, b->x, c->x, d->x, t);
    q.y = cr(a->y, b->y, c->y, d->y, t);
    return q;
}

/* Arc length of one segment, as the sum of SUB straight chords. */
static double seg_length(const path_point *w, int n, int i)
{
    path_point prev = seg_at(w, n, i, 0.0);
    double len = 0.0;
    int k;

    for (k = 1; k <= SUB; k++) {
        path_point cur = seg_at(w, n, i, (double)k / SUB);
        len += hypot(cur.x - prev.x, cur.y - prev.y);
        prev = cur;
    }
    return len;
}

/* ---- building -------------------------------------------------------- */

int path_build(path *p, const path_point *waypoints, int n, double spacing)
{
    double len[PATH_MAX_WAYPOINTS - 1];
    int steps[PATH_MAX_WAYPOINTS - 1];
    int i, total;

    if (!p) return PATH_EARG;
    p->n = 0;   /* nothing usable until the whole path is built */

    if (!waypoints || n < 2 || n > PATH_MAX_WAYPOINTS) return PATH_EARG;
    if (!fin(spacing) || spacing <= 0.0) return PATH_EARG;
    for (i = 0; i < n; i++)
        if (!fin(waypoints[i].x) || !fin(waypoints[i].y)) return PATH_EARG;

    /* First pass: measure. Cutting each segment into a whole number of equal
     * steps puts a point on every waypoint, at the price of a step that is only
     * close to `spacing`. Doing the count up front means we never half-fill the
     * path and then run out of room. */
    total = 1;
    for (i = 0; i < n - 1; i++) {
        len[i] = seg_length(waypoints, n, i);
        if (!(len[i] / spacing < (double)PATH_MAX_POINTS)) return PATH_EARG;
        steps[i] = (int)(len[i] / spacing + 0.5);
        if (steps[i] < 1) steps[i] = 1;     /* a repeated waypoint still gets one */
        total += steps[i];
        if (total > PATH_MAX_POINTS) return PATH_EARG;
    }

    /* Second pass: walk the same chords again and emit. */
    p->pts[0] = waypoints[0];
    p->s[0] = 0.0;
    p->n = 1;

    for (i = 0; i < n - 1; i++) {
        path_point prev = seg_at(waypoints, n, i, 0.0);
        path_point cur  = seg_at(waypoints, n, i, 1.0 / SUB);
        double acc = 0.0;                                   /* length up to prev */
        double d = hypot(cur.x - prev.x, cur.y - prev.y);   /* prev to cur */
        int k = 1, j;

        for (j = 1; j <= steps[i]; j++) {
            path_point e;

            if (j == steps[i]) {
                e = waypoints[i + 1];   /* the exact waypoint, not a rounded chord */
            } else {
                double target = len[i] * j / steps[i], f;

                while (k < SUB && acc + d < target) {
                    acc += d;
                    prev = cur;
                    k++;
                    cur = seg_at(waypoints, n, i, (double)k / SUB);
                    d = hypot(cur.x - prev.x, cur.y - prev.y);
                }
                f = (d > 0.0) ? (target - acc) / d : 0.0;
                if (f < 0.0) f = 0.0;
                if (f > 1.0) f = 1.0;
                e.x = prev.x + f * (cur.x - prev.x);
                e.y = prev.y + f * (cur.y - prev.y);
            }

            /* s measures the path as it will actually be used -- the polyline
             * through the emitted points -- so path_at agrees with it exactly. */
            p->s[p->n] = p->s[p->n - 1] + hypot(e.x - p->pts[p->n - 1].x,
                                                e.y - p->pts[p->n - 1].y);
            p->pts[p->n] = e;
            p->n++;
        }
    }
    return PATH_OK;
}

/* ---- looking things up ----------------------------------------------- */

double path_length(const path *p)
{
    return (p && p->n > 0) ? p->s[p->n - 1] : 0.0;
}

int path_at(const path *p, double s, path_point *out)
{
    int i;
    double d, f;

    if (!p || p->n < 1 || !out || !fin(s)) return PATH_EARG;

    if (s <= 0.0 || p->n == 1)      { *out = p->pts[0];        return PATH_OK; }
    if (s >= p->s[p->n - 1])        { *out = p->pts[p->n - 1]; return PATH_OK; }

    for (i = 1; i < p->n - 1; i++)
        if (p->s[i] >= s) break;

    d = p->s[i] - p->s[i - 1];
    f = (d > 0.0) ? (s - p->s[i - 1]) / d : 0.0;
    out->x = p->pts[i - 1].x + f * (p->pts[i].x - p->pts[i - 1].x);
    out->y = p->pts[i - 1].y + f * (p->pts[i].y - p->pts[i - 1].y);
    return PATH_OK;
}

/* Squared distance from q to the segment a-b, and how far along it that lands. */
static double seg_dist2(path_point a, path_point b, path_point q, double *f)
{
    double dx = b.x - a.x, dy = b.y - a.y;
    double dd = dx * dx + dy * dy;
    double t = 0.0, ex, ey;

    if (dd > 0.0) {
        t = ((q.x - a.x) * dx + (q.y - a.y) * dy) / dd;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
    }
    *f = t;
    ex = a.x + t * dx - q.x;
    ey = a.y + t * dy - q.y;
    return ex * ex + ey * ey;
}

int path_nearest(const path *p, path_point q, double *s)
{
    int i, best = 0;
    double bd = 0.0, d, f;

    if (!p || p->n < 1 || !s || !fin(q.x) || !fin(q.y)) return PATH_EARG;

    /* ponytail: linear scan; add a hint index if paths grow past PATH_MAX_POINTS */
    for (i = 0; i < p->n; i++) {
        double dx = p->pts[i].x - q.x, dy = p->pts[i].y - q.y;
        d = dx * dx + dy * dy;
        if (i == 0 || d < bd) { bd = d; best = i; }
    }
    *s = p->s[best];

    /* The true nearest point is on one of the two segments touching the nearest
     * corner, so project onto both and keep whichever beats the corner itself. */
    if (best > 0) {
        d = seg_dist2(p->pts[best - 1], p->pts[best], q, &f);
        if (d < bd) { bd = d; *s = p->s[best - 1] + f * (p->s[best] - p->s[best - 1]); }
    }
    if (best < p->n - 1) {
        d = seg_dist2(p->pts[best], p->pts[best + 1], q, &f);
        if (d < bd) { *s = p->s[best] + f * (p->s[best + 1] - p->s[best]); }
    }
    return PATH_OK;
}

/* ---- pure pursuit ---------------------------------------------------- */

int path_pursue(const path *p, double x, double y, double theta,
                double lookahead, double v, double *wz, int *done)
{
    path_point me, target;
    double s0, dx, dy, c, sn, bx, by, l2;

    if (!p || p->n < 1 || !wz || !done) return PATH_EARG;
    if (!fin(x) || !fin(y) || !fin(theta) || !fin(lookahead) || !fin(v)) return PATH_EARG;
    if (lookahead <= 0.0) return PATH_EARG;

    me.x = x;
    me.y = y;
    if (path_nearest(p, me, &s0) != PATH_OK) return PATH_EARG;
    if (path_at(p, s0 + lookahead, &target) != PATH_OK) return PATH_EARG;

    *done = (s0 + lookahead >= path_length(p)) ? 1 : 0;

    /* The target seen from the driving seat: bx out of the nose, by out to the left. */
    dx = target.x - x;
    dy = target.y - y;
    c  = cos(theta);
    sn = sin(theta);
    bx =  c * dx + sn * dy;
    by = -sn * dx + c * dy;

    /* One arc joins the robot to the target while staying tangent to the heading,
     * and its curvature is 2*by / L^2. Turning at k*v drives exactly that arc. */
    l2 = bx * bx + by * by;
    *wz = (l2 > 1e-12) ? (2.0 * by / l2) * v : 0.0;   /* sitting on it: hold straight */
    return PATH_OK;
}
