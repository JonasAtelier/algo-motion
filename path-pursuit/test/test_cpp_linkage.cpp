/* The header claims C++ can use it. Prove the extern "C" block works. */
#include "path.h"

#include <cassert>
#include <cmath>

int main()
{
	static const path_point line[2] = { { 0.0, 0.0 }, { 10.0, 0.0 } };
	path p;
	path_point at;
	double s, wz;
	int done;

	assert(path_build(&p, line, 2, 0.5) == PATH_OK);
	assert(std::fabs(path_length(&p) - 10.0) < 1e-9);

	assert(path_at(&p, 2.5, &at) == PATH_OK);
	assert(std::fabs(at.x - 2.5) < 1e-9);	/* a quarter of the way along */

	path_point q = { 5.0, 3.0 };
	assert(path_nearest(&p, q, &s) == PATH_OK);
	assert(std::fabs(s - 5.0) < 1e-9);

	assert(path_pursue(&p, 0.0, 1.0, 0.0, 2.0, 1.0, &wz, &done) == PATH_OK);
	assert(wz < 0.0);			/* off to the left, so it steers right */
	return 0;
}
