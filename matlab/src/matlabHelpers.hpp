#include "Eigen/Dense"

#pragma once

Eigen::Matrix<bool, 6, 1> poseElements(bool x, bool y, bool z, bool a, bool b, bool g)
	{ return (Eigen::Matrix<bool, 6, 1>() << x, y, z, a, b, g).finished(); }
