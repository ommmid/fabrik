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

void make_unit_vector3d(Eigen::Vector3d& vector);

/** \brief Find the angle between two arbitrary vectors */
double angleBetweenTwoVectors(Eigen::Vector3d& v1, Eigen::Vector3d& v2);

}

