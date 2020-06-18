/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "fabrik/robot_state/robot_state.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"

#include "fabrik/base/fabrik.h"


namespace fabrik
{

PositionBasedCalculator::PositionBasedCalculator()
{

}

double PositionBasedCalculator::calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2)
{
    // distance between the location of two frames is the error
    Eigen::Vector3d frame_1_to_frame_2 = frame_2.translation() - frame_1.translation();
    return frame_1_to_frame_2.norm();
}

double PositionBasedCalculator::calculateReach(const Eigen::Affine3d& start_frame_reaching,
                                               const Eigen::Affine3d& end_frame_reaching,
                                               const Eigen::Affine3d& frame_aimed_at)
{
    // example, consider forward: .. e2][s3 e3] --> [s4 e4]
    // find the vector connecting s3 to s4: trans34 = trans4 - trans3
    Eigen::Vector3d start_to_aim = frame_aimed_at.translation() - start_frame_reaching.translation();

    // create the plane whose normal is joint 3 
    Eigen::Vector3d j_plane_normal(start_frame_reaching.rotation()(0,2),
                                    start_frame_reaching.rotation()(1,2),
                                    start_frame_reaching.rotation()(2,2));
    Eigen::Vector3d j_origin(start_frame_reaching.translation);                                    
    Eigen::Hyperplane<double, 3> j_plane(j_plane_normal, j_origin);

    // project start_to_aim on joint plane
    Eigen::Vector3d start_to_aim_projected = j_plane.projection(start_to_aim);

    // find the vector connecting s3 to e3
    Eigen::Vector3d start_to_end = end_frame_reaching.translation() - start_frame_reaching.translation();

    // find the angle between start_to_end and start_to_aim_projected
    double angle = angleBetweenTwoVectors(start_to_end, start_to_aim_projected);

    // what happens to joint angle now ???

    return ;
}


// project a point on a plane. The plane is passing through the origin of the point.
inline double angleBetweenTwoVectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
    v1.normalize();
    v2.normalize();

   // check if the angle is the smaller not the larger ???
    double angle = std::acos(v1.dot(v2));

    return projectedPoint;
}


}