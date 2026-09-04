/* A differential rover following an S-bend: six waypoints in, a smooth path out,
 * and pure pursuit steering it round. Prints where it is and how far off the line
 * it is, twice a second, until it reaches the end.
 *
 * The rover is simulated with the same exact-arc step mob uses for odometry. On a
 * real robot you would feed wz to mob_inverse and let the wheels do the driving. */
#include "path.h"
#include <math.h>
#include <stdio.h>

#define DT        0.02   /* a 50 Hz control loop */
#define SPEED     0.6    /* m/s, held constant: this library does not plan speed */
#define LOOKAHEAD 1.0    /* m ahead to aim, about 2x a 0.5 m wheelbase */
#define SPACING   0.1    /* m between path points */

/*                        start        swing out         and back        straight */
static const path_point route[6] = {
    { 0.0, 0.0 }, { 2.0, 0.0 }, { 4.0, 2.0 }, { 6.0, 2.0 }, { 8.0, 0.0 }, { 10.0, 0.0 },
};

/* The exact-arc dead-reckoning step, copied from mob's mob_integrate. */
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

/* How far the robot is off the path: nearest point on it, then the distance to it. */
static double cross_track(const path *p, double x, double y)
{
    path_point q, on;
    double s;

    q.x = x;
    q.y = y;
    if (path_nearest(p, q, &s) != PATH_OK) return 0.0;
    if (path_at(p, s, &on) != PATH_OK) return 0.0;
    return hypot(on.x - x, on.y - y);
}

int main(void)
{
    path p;
    double x = 0.0, y = 0.0, th = 0.0;
    int step, done = 0;

    if (path_build(&p, route, 6, SPACING) != PATH_OK) {
        printf("that path will not fit\n");
        return 1;
    }

    printf("six waypoints -> a %.2f m path in %d points, %.2f m apart\n",
           path_length(&p), p.n, path_length(&p) / (p.n - 1));
    printf("following it at %.1f m/s with a %.1f m lookahead\n\n", SPEED, LOOKAHEAD);
    printf("  time        x       y   theta      wz   off the line\n");

    for (step = 0; step < 4000 && !done; step++) {
        double wz;

        if (path_pursue(&p, x, y, th, LOOKAHEAD, SPEED, &wz, &done) != PATH_OK) {
            printf("pursue refused the arguments\n");
            return 1;
        }
        if (step % 25 == 0)   /* every half second */
            printf("  %4.1f s  %6.2f  %6.2f  %6.2f  %6.2f   %6.3f m\n",
                   step * DT, x, y, th, wz, cross_track(&p, x, y));

        arc_step(&x, &y, &th, SPEED, wz, DT);
    }

    printf("\ndone after %.1f s at x %.2f  y %.2f, %.3f m off the line.\n",
           step * DT, x, y, cross_track(&p, x, y));
    printf("the last %.1f m is the lookahead: pure pursuit stops aiming before it\n"
           "arrives, so park the robot yourself once done goes to 1.\n", LOOKAHEAD);
    return 0;
}
