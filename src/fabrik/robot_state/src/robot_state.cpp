/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "fabrik/robot_state/robot_state.h"

#include "fabrik/util/exception.h"


namespace robot_state
{

Eigen::Affine3d rotation_z(const double theta){
    start_end(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
} 

// -------------------- RobotState --------------------
RobotState::RobotState(const robot_model::Chain& chain):
chain_(chain)
{
    reaching_direction_ = robot_state::FORWARD;

    int dof = chain->getDOF();
    int reaching_at_ = dof + 1;
    
    joints_values_.resize(dof);
    frames_.resize(dof);

    std::vector<robot_model::Link> robot_structure = chain->getRobotStructur();

    Eigen::Affine3d start_world, end_world;
    end_world = base_;
    for(int i = 0; i < dof; ++i)
    {
        joints_values_[i] = 0;

        start_world = end_world * rotation_z(joints_values_[i]);
        end_world = start_world * robot_structure[i].getLinkFrame();
        
        std::pair<Eigen::Affine3d, Eigen::Affine3d> link_pair(start_world, end_world);
        frames_[i] = link_pair;
    }
}



}