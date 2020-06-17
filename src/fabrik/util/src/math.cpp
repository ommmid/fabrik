#include <Eigen/Geometry>
#include <random>

#include <stdexcept>
#include <string>

#include <fabrik/util/math.h>


Eigen::Affine3d fabrik::rotation_z(const double& theta)
{
    Eigen::Affine3d hm_z = Eigen::Affine3d(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
    return hm_z;
} 

double fabrik::random_double(const double& start, const double& end)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(start, end);

    return distribution(generator);
}

void fabrik::make_unit_vector3d(Eigen::Vector3d& vector)
{
     double sum = vector(0) * vector(0) + vector(1) * vector(1) + vector(2) * vector(2);
     
     vector(0) = vector(0) / sum;
     vector(1) = vector(1) / sum;
     vector(2) = vector(2) / sum;
}




