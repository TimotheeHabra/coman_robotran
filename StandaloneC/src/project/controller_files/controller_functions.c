//---------------------------
// Creation : 29/10/2013
// Last update : 08/07/2014
//
// Useful functions
//
//---------------------------

#include "controller_def.h"

// Return an angle in the ]-pi ; pi] interval
double limit_angle(double angle)
{
	while(angle <= -PI)
	{
		angle += 2.0 * PI;
	}
	while(angle > PI)
	{
		angle -= 2.0 * PI;
	}

	return angle;
}

// return 'value' bounded in the [this_min; this_max] interval
double limit_value(double value, double this_min, double this_max)
{
	if (value < this_min)
	{
		return this_min;
	}
	else if (value > this_max)
	{
		return this_max;
	}
	else
	{
		return value;
	}
}

// return the interpolation for the abscissa x (x1, x2, y1 and y2 define the line)
double linear_interpolation(double x, double x1, double x2, double y1, double y2)
{
    double slope;

    if (x1 == x2)
    {
        printf("ERROR: linear interpolation with x1 == x2 !\n");
        return 0.0;
    }

    slope = (y2 - y1) / (x2 - x1);

    return y1 + slope * (x - x1);
}
