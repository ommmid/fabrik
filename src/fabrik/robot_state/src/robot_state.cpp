/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "fabrik/robot_state/robot_state.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"

namespace fabrik
{

// -------------------- RobotState --------------------
RobotState::RobotState(const RobotModelPtr& robot_model):
robot_model_(robot_model)
{
    base_ = robot_model_->getBase();
    chain_ = robot_model_->getRobotChain();

    reaching_direction_ = ReachingDirection::FORWARD;

    dof_ = robot_model_->getDOF();
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
    // std::cout << "joint number: " << joint_number << " joint value: " << joint_value << std::endl;

    // update s_i and e_i
    if (reaching_direction_ == fabrik::ReachingDirection::FORWARD)
    {
        joints_values_[joint_number] = joint_value;

        // For index 0, we need the end frame of the base link
        Eigen::Affine3d end_i_minus_1 = (joint_number == 0) ? base_ : frames_[joint_number - 1].second;
        // s_i = e_{i-1} * rotation_z
        frames_[joint_number].first = end_i_minus_1 * fabrik::rotation_z(joint_value);
        // e_i = s_i * relative transformation of link_i
        frames_[joint_number].second = frames_[joint_number].first * chain_[joint_number].getLinkFrame();

        reached_at_ = joint_number;

        // std::cout << "rotation_z: \n" << fabrik::rotation_z(joint_value).matrix() << std::endl;
        // std::cout << "base: \n" << base_.matrix() << std::endl;
        // std::cout << "end_i_minus_1: \n" << end_i_minus_1.matrix() << std::endl;
        // std::cout << "start_i: \n" << frames_[joint_number].first.matrix() << std::endl;
        // std::cout << "end_i: \n" << frames_[joint_number].second.matrix() << std::endl;

    }
    // update s_{i-1} and e_{i-1}
    else if (reaching_direction_ == fabrik::ReachingDirection::BACKWARD)
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

    // printState("update state ......", std::vector<int>{0,1,2});
}

void RobotState::printState(const std::string text, const std::vector<int>& which_frames) const
{
    std::cout << "--------------------------- Robot State --------------------------- " << std::endl; 
    std::cout << "From: " << text << std::endl;

    // reaching direction
    std::string reaching_direction = reaching_direction_ == 0 ? "FORWARD" : "BACKWARD";
    std::cout << reaching_direction << std::endl;
    std::cout << "Reached at: " << reached_at_ << std::endl;
    std::string dash_board = createDashBoard();
    std::cout << dash_board << std::endl;

    Eigen::Affine3d end_effector = frames_[dof_ - 1].second;
    std::cout << "\nend effector: \n" << end_effector.matrix() << std::endl;

    if(which_frames[0] != -1)
    {
        std::cout << "base:\n" << base_.matrix() <<std::endl;
        for(int k : which_frames)
        {
            Eigen::MatrixXd start_HM = frames_[k].first.matrix();
            Eigen::MatrixXd end_HM = frames_[k].second.matrix();
            std::cout << "s" << std::to_string(k) << ": \n" << start_HM << std::endl;
            std::cout << "e" << std::to_string(k) << ": \n" << end_HM << std::endl;
        }
    }

    // Eigen::Affine3d ff = fabrik::rotation_z(0.2);
    // std::cout << ff.matrix() << std::endl;
    std::cout << "------------------------------------------------------------------- " << std::endl; 
}


std::string RobotState::createDashBoard() const
{
    std::string arrow = reaching_direction_ == fabrik::ReachingDirection::FORWARD ?
             "===>" : "<===";

    std::string dash_board = "      |";
    for (int k = 0; k < dof_; ++k)
    {
        std::string end_char = k == dof_-1 ? " " : "|";
        
        if (k == reached_at_)
        {
            dash_board += "   " + arrow + "  " + end_char;
        }
        else
        {
            dash_board += "         " + end_char;
        }
    }

    // second line
    dash_board += "\n[base]|";
    for (int k = 0; k < dof_; ++k)
    {
        if (k == dof_ - 1)
        {
            dash_board += "[s" + std::to_string(k) + "   e" + std::to_string(k) + "]";
        }else
        {
            dash_board += "[s" + std::to_string(k) + "   e" + std::to_string(k) + "]|";
        }
    }

    // third line
    dash_board += "\n      |";
    for (int k = 0; k < dof_; ++k)
    {
        std::string end_char = k == dof_-1 ? " " : "|";
        dash_board += "         " + end_char;        
    }

    // fourth line
    dash_board += "\n    ";
    for (int k = 0; k < dof_; ++k)
    {
        // to_string always creates 6 decmial digits. 
        dash_board += std::to_string(joints_values_[k]) + "  ";        
    }


return dash_board;
}


}