#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "Eigen/Eigen/Dense"

namespace mathLib
{
	//const double PI = std::atan(1.0) * 4;

	float rad(const float& degree);
	float deg(const float& rad);



	double get_distance(double x1, double y1, double x2 = 0.0, double y2 = 0.0);

	Eigen::Matrix4f calcTransformationMatrix(const Eigen::RowVector4f& DHparams);

} //namespace mathLib