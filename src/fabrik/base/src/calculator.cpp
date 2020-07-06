/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "fabrik/robot_state/robot_state.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"
#include "fabrik/util/output.h"

#include "fabrik/base/calculator.h"


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
    // example, consider forward to find J_3: .. e2][s3 e3] --> [s4 e4]
    // find the vector connecting s3 to s4: trans34 = trans4 - trans3
    // what about backward ????
    start_to_aim = frame_aimed_at.translation() - start_frame_reaching.translation();

    // create the plane whose normal is J_3 
    Eigen::Vector3d j_plane_normal( start_frame_reaching.rotation()(0,2),
                                    start_frame_reaching.rotation()(1,2),
                                    start_frame_reaching.rotation()(2,2));
    Eigen::Vector3d j_origin(start_frame_reaching.translation());                                    
    Eigen::Hyperplane<double, 3> j_plane(j_plane_normal, j_origin);

    // project a point on a plane. The plane is passing through the origin of the point.
    // project start_to_aim on joint plane
    // start_to_aim_projected = j_plane.projection(start_to_aim);
    start_to_aim_projected = start_to_aim - (start_to_aim.dot(j_plane_normal)) * j_plane_normal;

    // normal and the projected one must be perpendicular
    double result = j_plane_normal.dot(start_to_aim_projected);
    std::cout << "====>>> is start_to_aim_projected perpendicular to plane's normal: " << result << std::endl;

    // find the vector connecting s3 to e3
    start_to_end = end_frame_reaching.translation() - start_frame_reaching.translation();
    
    // start_to_end might not lie on the joint plane either. we have to project this as well
    // start_to_end_projected = j_plane.projection(start_to_end);
    start_to_end_projected = start_to_end - (start_to_end.dot(j_plane_normal)) * j_plane_normal;
    result = j_plane_normal.dot(start_to_end_projected);
    std::cout << "====>>> is start_to_end_projected perpendicular to plane's normal: " << result << std::endl;

    // find the angle FROM start_to_end_projected TO start_to_aim_projected. 
    double angle = signedAngleBetweenTwoVectors(start_to_end_projected, start_to_aim_projected,
                                                j_plane_normal);

    return angle; 
}

OrientationBasedCalculator::OrientationBasedCalculator()
{

}

double OrientationBasedCalculator::calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2)
{
    
    return 0;
}

double OrientationBasedCalculator::calculateReach(const Eigen::Affine3d& start_frame_reaching,
                          const Eigen::Affine3d& end_frame_reaching,
                          const Eigen::Affine3d& frame_aimed_at)
{
    
    // what is the joint angle after calculating the angle between two vectors ???
    return  10; // correct it
}


ComboCalculator::ComboCalculator()
{

}


double ComboCalculator::calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2)
{
    
    return 0;
}

double ComboCalculator::calculateReach(const Eigen::Affine3d& start_frame_reaching,
                          const Eigen::Affine3d& end_frame_reaching,
                          const Eigen::Affine3d& frame_aimed_at)
{
    
    // what is the joint angle after calculating the angle between two vectors ???
    return  10; // correct it
}


}