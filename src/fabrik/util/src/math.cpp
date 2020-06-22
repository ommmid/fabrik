#include <Eigen/Geometry>
#include <random>

#include <stdexcept>
#include <string>

#include <fabrik/util/math.h>

namespace fabrik{

Eigen::Affine3d rotation_z(const double& theta)
{
    Eigen::Affine3d hm_z = Eigen::Affine3d(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
    return hm_z;
} 

double random_double(const double& start, const double& end)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(start, end);

    return distribution(generator);
}

void make_unit_vector3d(Eigen::Vector3d& vector)
{
     double sum = vector(0) * vector(0) + vector(1) * vector(1) + vector(2) * vector(2);
     
     vector(0) = vector(0) / sum;
     vector(1) = vector(1) / sum;
     vector(2) = vector(2) / sum;
}

double angleBetweenTwoVectors(Eigen::Vector3d& v1, Eigen::Vector3d& v2)
{
    v1.normalize();
    v2.normalize();

   // check if the angle is the smaller not the larger ???
    double angle = std::acos(v1.dot(v2));

    return angle;
}

}

