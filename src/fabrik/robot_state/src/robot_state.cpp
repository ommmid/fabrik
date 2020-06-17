/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "fabrik/robot_state/robot_state.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"


namespace robot_state
{

// -------------------- RobotState --------------------
RobotState::RobotState(const std::vector<robot_model::Link>& chain, const Eigen::Affine3d& base):
chain_(chain), base_(base)
{
    std::cout << "Constructing RobotState ... " << std::endl;
    reaching_direction_ = robot_state::FORWARD;

    dof_ = chain.size();
    reaching_at_ = dof_;
    
    std::cout << "1 ... " << std::endl;

    joints_values_.resize(dof_);
    frames_.resize(dof_);
    std::cout << "2 ... " << std::endl;

    // Set all the joint to zero and update the frames
    for(int i = 0; i < dof_; ++i)
    {
        joints_values_[i] = 0;
        // frames number start from 1 not 0
        updateFrame(joints_values_[i], i );
    }
    std::cout << "3 ... " << std::endl;
}

void RobotState::updateFrame(const double& joint_value, const int& joint_number)
{
    std::cout << "updateFrame ... " << std::endl;
    std::cout << "reaching direction: " << reaching_direction_ << std::endl;

    // update s_i and e_i
    if (reaching_direction_ == robot_state::ReachingDirection::FORWARD)
    {
        joints_values_[joint_number] = joint_value;

        // For index 1, we need end frame from the base link
        Eigen::Affine3d end_i_minus_1 = (joint_number == 0) ? base_ : frames_[joint_number - 1].second;
        // s_i = e_{i-1} * rotation_z
        frames_[joint_number].first = end_i_minus_1 * fabrik::rotation_z(joint_value);
        // e_i = s_i * relative transformation of link_i
        frames_[joint_number].second = frames_[joint_number].first * chain_[joint_number].getLinkFrame();
    }
    // update s_{i-1} and e_{i-1}
    else if (reaching_direction_ == robot_state::ReachingDirection::BACKWARD)
    {       
        if(joint_number == 0)
        {
            fabrik::Exception("joint number 0 can not be updated in backward reaching");
        }
        else
        {
            // We assume the joint value found by the solve process is going to locate 
            // e_{i-1} with respect to s_i which is in the opposite direction of the joint 
            joints_values_[joint_number] = -joint_value;

            Eigen::Affine3d start_i = frames_[joint_number].first;

            // e_{i-1} = s_i * inv(rotation_z(theta)) = s_i * rotation_z(-theta)
            // joint_value is already in the opposite direction of the joint so we do not have to negate it.
            frames_[joint_number - 1].second = start_i * fabrik::rotation_z(joint_value);
            // e_i = s_i * relative transformation of link_i
            frames_[joint_number - 1].first = frames_[joint_number - 1].second * chain_[joint_number].getLinkFrame().inverse();
        }
    }
}
            

}