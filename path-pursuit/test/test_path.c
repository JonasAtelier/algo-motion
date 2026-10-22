/* Self-check: assert-based, no framework. Build with `make check`. */
#include "path.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846
#define NEAR(a, b) (fabs((a) - (b)) < 1e-9)

/* The exact-arc dead-reckoning step, copied from mob's mob_integrate so the
 * closed-loop test can drive a rover without depending on mob. */
static void arc_step(double *x, double *y, double *th, double v, double wz, double dt)
{
    double dth = wz * dt, s0 = sin(*th), c0 = cos(*th);

    if (fabs(dth) < 1e-9) {
        *x += v * c0 * dt;
        *y += v * s0 * dt;
    } else {
        double s1 = sin(*th + dth), c1 = cos(*th + dth);
        *x +=  v * (s1 - s0) / wz;
        *y += -v * (c1 - c0) / wz;
    }
    *th += dth;
}

/* How far the robot is off the path, measured through path_nearest. */
static double cross_track(const path *p, double x, double y)
{
    path_point q, on;
    double s;

    q.x = x; q.y = y;
    assert(path_nearest(p, q, &s) == PATH_OK);
    assert(path_at(p, s, &on) == PATH_OK);
    return hypot(on.x - x, on.y - y);
}

static const path_point straight[2] = { { 0.0, 0.0 }, { 10.0, 0.0 } };

static const path_point zigzag[5] = {
    { 0.0, 0.0 }, { 1.0, 1.0 }, { 2.0, -1.0 }, { 3.0, 1.0 }, { 4.0, 0.0 },
};

static void test_straight(void)
{
    path p;
    int i;

    assert(path_build(&p, straight, 2, 0.5) == PATH_OK);
    assert(p.n == 21);                           /* 10 m in 0.5 m steps, plus the start */
    assert(NEAR(path_length(&p), 10.0));

    for (i = 0; i < p.n; i++) {
        assert(fabs(p.pts[i].y) < 1e-12);        /* two waypoints make a straight line */
        assert(NEAR(p.pts[i].x, 0.5 * i));
        assert(NEAR(p.s[i], 0.5 * i));
    }
}

static void test_passes_through_waypoints(void)
{
    path p;
    double polyline = 0.0;
    int i, j;

    assert(path_build(&p, zigzag, 5, 0.25) == PATH_OK);

    for (i = 0; i < 5; i++) {
        double best = 1e9;
        for (j = 0; j < p.n; j++) {
            double d = hypot(p.pts[j].x - zigzag[i].x, p.pts[j].y - zigzag[i].y);
            if (d < best) best = d;
        }
        assert(best < 1e-6);                     /* the curve hits every waypoint */
    }

    for (i = 0; i < 4; i++)
        polyline += hypot(zigzag[i + 1].x - zigzag[i].x, zigzag[i + 1].y - zigzag[i].y);
    assert(path_length(&p) >= polyline);         /* rounding corners cannot shorten it */
}

static void test_at_and_nearest(void)
{
    path p;
    path_point q, out;
    double s;

    assert(path_build(&p, straight, 2, 0.5) == PATH_OK);

    assert(path_at(&p, -5.0, &out) == PATH_OK);          /* clamped to the start */
    assert(NEAR(out.x, 0.0) && NEAR(out.y, 0.0));

    assert(path_at(&p, 999.0, &out) == PATH_OK);         /* clamped to the end */
    assert(NEAR(out.x, 10.0) && NEAR(out.y, 0.0));

    assert(path_at(&p, 3.25, &out) == PATH_OK);          /* between two points */
    assert(NEAR(out.x, 3.25) && NEAR(out.y, 0.0));

    q.x = 5.0; q.y = 3.0;                                /* straight out to the side */
    assert(path_nearest(&p, q, &s) == PATH_OK);
    assert(NEAR(s, 5.0));

    q.x = -4.0; q.y = 0.0;                               /* behind the start */
    assert(path_nearest(&p, q, &s) == PATH_OK);
    assert(NEAR(s, 0.0));
}

static void test_pursue_steers_back(void)
{
    path p;
    double wz;
    int done;

    assert(path_build(&p, straight, 2, 0.5) == PATH_OK);

    /* One metre to the left of the line, pointing along it: turn right. */
    assert(path_pursue(&p, 0.0, 1.0, 0.0, 2.0, 1.0, &wz, &done) == PATH_OK);
    assert(wz < 0.0);
    assert(done == 0);

    /* One metre to the right: turn left. Mirrored, so the same size. */
    {
        double wz2;
        assert(path_pursue(&p, 0.0, -1.0, 0.0, 2.0, 1.0, &wz2, &done) == PATH_OK);
        assert(wz2 > 0.0);
        assert(NEAR(wz2, -wz));
    }

    /* On the line and pointing along it: no turn at all. */
    assert(path_pursue(&p, 3.0, 0.0, 0.0, 2.0, 1.0, &wz, &done) == PATH_OK);
    assert(fabs(wz) < 1e-9);

    /* Within a lookahead of the end. */
    assert(path_pursue(&p, 9.5, 0.0, 0.0, 2.0, 1.0, &wz, &done) == PATH_OK);
    assert(done == 1);
}

static void test_follows_a_circle(void)
{
    path_point circle[8];
    path p;
    double x = 3.15, y = 0.0, th = PI / 2;   /* 15 cm outside the ring, facing along it */
    double dt = 1.0 / 50.0, v = 0.5, travelled = 0.0, worst = 0.0;
    int i, done = 0, steps;

    for (i = 0; i < 8; i++) {
        circle[i].x = 3.0 * cos(2.0 * PI * i / 8.0);
        circle[i].y = 3.0 * sin(2.0 * PI * i / 8.0);
    }
    assert(path_build(&p, circle, 8, 0.1) == PATH_OK);

    for (steps = 0; steps < 3000 && !done; steps++) {
        double wz;
        assert(path_pursue(&p, x, y, th, 0.8, v, &wz, &done) == PATH_OK);
        arc_step(&x, &y, &th, v, wz, dt);
        travelled += v * dt;

        /* The first metre is the robot pulling itself onto the path. */
        if (travelled > 1.0) {
            double e = cross_track(&p, x, y);
            if (e > worst) worst = e;
        }
    }
    assert(done == 1);                       /* it got to the end */
    assert(steps < 3000);
    assert(worst < 0.15);                    /* and stayed on the line getting there */
}

static void test_bad_args(void)
{
    path_point one = { 0.0, 0.0 };
    path_point bad[2] = { { 0.0, 0.0 }, { 1.0, 0.0 } };
    path_point q = { 0.0, 0.0 };
    path_point out;
    path p;
    double s, wz;
    int done;

    assert(path_build(&p, straight, 1, 0.5) == PATH_EARG);     /* one waypoint is not a path */
    assert(p.n == 0);                                          /* a failed build leaves nothing */
    assert(path_build(&p, straight, 2, 0.0) == PATH_EARG);
    assert(path_build(&p, straight, 2, -1.0) == PATH_EARG);
    assert(path_build(&p, straight, 2, NAN) == PATH_EARG);
    assert(path_build(&p, straight, PATH_MAX_WAYPOINTS + 1, 0.5) == PATH_EARG);
    assert(path_build(&p, NULL, 2, 0.5) == PATH_EARG);
    assert(path_build(NULL, straight, 2, 0.5) == PATH_EARG);
    assert(path_build(&p, &one, 1, 0.5) == PATH_EARG);

    bad[1].y = NAN;
    assert(path_build(&p, bad, 2, 0.5) == PATH_EARG);

    /* 10 m at 1 cm needs 1001 points and only PATH_MAX_POINTS fit. */
    assert(path_build(&p, straight, 2, 0.01) == PATH_EARG);
    assert(p.n == 0);                                          /* and nothing was left behind */

    assert(path_build(&p, straight, 2, 0.5) == PATH_OK);
    assert(path_at(NULL, 1.0, &out) == PATH_EARG);
    assert(path_at(&p, 1.0, NULL) == PATH_EARG);
    assert(path_at(&p, NAN, &out) == PATH_EARG);
    assert(path_nearest(NULL, q, &s) == PATH_EARG);
    assert(path_nearest(&p, q, NULL) == PATH_EARG);
    q.x = NAN;
    assert(path_nearest(&p, q, &s) == PATH_EARG);

    assert(path_pursue(NULL, 0.0, 0.0, 0.0, 1.0, 1.0, &wz, &done) == PATH_EARG);
    assert(path_pursue(&p, 0.0, 0.0, 0.0, 1.0, 1.0, NULL, &done) == PATH_EARG);
    assert(path_pursue(&p, 0.0, 0.0, 0.0, 1.0, 1.0, &wz, NULL) == PATH_EARG);
    assert(path_pursue(&p, 0.0, 0.0, 0.0, 0.0, 1.0, &wz, &done) == PATH_EARG);   /* no lookahead */
    assert(path_pursue(&p, 0.0, 0.0, 0.0, -1.0, 1.0, &wz, &done) == PATH_EARG);
    assert(path_pursue(&p, NAN, 0.0, 0.0, 1.0, 1.0, &wz, &done) == PATH_EARG);
    assert(path_pursue(&p, 0.0, 0.0, NAN, 1.0, 1.0, &wz, &done) == PATH_EARG);
    assert(path_pursue(&p, 0.0, 0.0, 0.0, 1.0, NAN, &wz, &done) == PATH_EARG);

    assert(path_length(NULL) == 0.0);
}

int main(void)
{
    test_straight();
    test_passes_through_waypoints();
    test_at_and_nearest();
    test_pursue_steers_back();
    test_follows_a_circle();
    test_bad_args();
    printf("all tests passed\n");
    return 0;
}
