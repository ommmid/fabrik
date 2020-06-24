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
    if(chain.size() == 0)
    {
        throw fabrik::Exception("chain must have at least one link.");
    }

    reaching_direction_ = ReachingDirection::FORWARD;

    dof_ = chain.size();
    reached_at_ = dof_ - 1;
    
    joints_values_.resize(dof_);
    frames_.resize(dof_);

    // Set all the joint to zero and update the frames
    for(int i = 0; i < dof_; ++i)
    {
        joints_values_[i] = 0;
        // frames number start from 0 
        updateState(joints_values_[i], i );
    }
}

void RobotState::updateState(Eigen::Affine3d target)
{
    if(reaching_direction_ == ReachingDirection::FORWARD)
    {
        throw fabrik::Exception("Wrong Update. 'target' can not be set at FORWARD reaching.");
    }

    frames_[dof_ - 1].second = target;
    // s_i = e_i * inverse(relative transformation of link_i)
    frames_[dof_ - 1].first = target * chain_[dof_ - 1].getLinkFrame().inverse();

    reached_at_ = dof_ - 1;
}

void RobotState::updateState(const double& joint_value, const int& joint_number)
{
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

        reached_at_ = joint_number;
    }
    // update s_{i-1} and e_{i-1}
    else if (reaching_direction_ == robot_state::ReachingDirection::BACKWARD)
    {       
        if(joint_number == 0)
        {
            throw fabrik::Exception("joint number 0 can not be updated in backward reaching");
        }

        // We assume the joint value found by the solve process is going to locate 
        // e_{i-1} with respect to s_i which is in the opposite direction of the joint 
        joints_values_[joint_number] = -joint_value;

        Eigen::Affine3d start_i = frames_[joint_number].first;

        // e_{i-1} = s_i * inv(rotation_z(theta)) = s_i * rotation_z(-theta)
        // joint_value is already in the opposite direction of the joint so we do not have to negate it.
        frames_[joint_number - 1].second = start_i * fabrik::rotation_z(joint_value);
        // s_i = e_i * inverse (relative transformation of link_i)
        frames_[joint_number - 1].first = frames_[joint_number - 1].second * chain_[joint_number].getLinkFrame().inverse();

        reached_at_ = joint_number - 1;
    }
}

void RobotState::printState(std::string name) const
{
    std::cout << "--------------------------- Robot State --------------------------- " << std::endl; 
    std::cout << "From: " << name << std::endl;

    // reaching direction
    std::string reaching_direction = reaching_direction_ == 0 ? "FORWARD" : "BACKWARD";
    std::cout << reaching_direction << " Reaching =====>>>" << std::endl;
    std::cout << "Reached at: " << reached_at_ << std::endl;
    for (int i = 0; i < dof_; ++i)
        std::cout << "joint value_" << i << ": " << joints_values_[i] << std::endl;

    std::cout << "------------------------------------------------------------------- " << std::endl; 
}

}