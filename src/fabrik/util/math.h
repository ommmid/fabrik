#pragma once

#include <Eigen/Geometry>
#include <random>

#include <stdexcept>
#include <string>

// copied from ompl
namespace fabrik
{

/** \brief An affine3d (4x4) matrix with only rotation around z and no translation */
Eigen::Affine3d rotation_z(const double& theta);

double random_double(const double& start, const double& end);

// template <typename T>
// T make_unit_vector(const T& vector)
// {
//      = vector(0) * vector(0) + vector(1) * vector(1) + vector(2) * vector(2);
// }

void make_unit_vector3d(Eigen::Vector3d& vector);


}

