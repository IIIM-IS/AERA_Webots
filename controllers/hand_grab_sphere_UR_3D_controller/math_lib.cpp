#include "math_lib.h"


namespace mathLib
{
	float rad(const float& degree)
	{
		return (degree * M_PI / 180);
	}

	float deg(const float& rad)
	{
		return (rad * 180 / M_PI);
	}

	double get_distance(double x1, double y1, double x2, double y2) {
		return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
	}


	Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& DHparams)
	{
		Eigen::Matrix4f individualTransformationMatrix;
		individualTransformationMatrix << cos(DHparams[3]), -sin(DHparams[3]), 0, DHparams[1],
			(sin(DHparams[3]) * cos(DHparams[0])), (cos(DHparams[3]) * cos(DHparams[0])), -sin(DHparams[0]), (-sin(DHparams[0]) * DHparams[2]),
			(sin(DHparams[3]) * sin(DHparams[0])), (cos(DHparams[3]) * sin(DHparams[0])), cos(DHparams[0]), (cos(DHparams[0]) * DHparams[2]),
			0, 0, 0, 1;

		return individualTransformationMatrix;
	}


} //namespace mathLib