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

double randomDouble(const double start, const double end)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(start, end);

    return distribution(generator);
}

double signedAngleBetweenTwoVectors(Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& n)
{
    v1.normalize();
    v2.normalize();

    double cost = v1.dot(v2);
    double sint = (n.cross(v1)).dot(v2);
    double angle = std::atan2(sint, cost);

    // I could make angle to be positive between 0 and 2*pi but having it negative is OK too.

    return angle;
}


}

