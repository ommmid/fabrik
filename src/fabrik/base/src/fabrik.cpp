/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "fabrik/robot_state/robot_state.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"

#include "fabrik/base/fabrik.h"
#include "fabrik/base/calculator.h"

namespace fabrik
{

// ------------------------- FABRIK -------------------------
FABRIK::FABRIK(robot_state::RobotStatePtr& robot_state,
           std::vector<double>& initial_configuration,
           Eigen::Affine3d& target,
           double threshold,
           double requested_iteration_num,
           CalculatorType calculator_type):
robot_state_(robot_state), initial_configuration_(initial_configuration), 
target_(target), threshold_(threshold), requested_iteration_num_(requested_iteration_num)
{
    // set the state of the robot to the given initial_configuration
    int dof = robot_state_->getDOF();
    for (int k = 0; k < dof; ++k)
        robot_state_->updateState(initial_configuration_[k], k);

    // set the calculator 
    calculator_ = FABRIK::createCalculator(calculator_type);
}

CalculatorPtr FABRIK::createCalculator(const CalculatorType& calculator_type)
{
    switch (calculator_type)
    {
        case CalculatorType::POSITION:
            return CalculatorPtr(new PositionBasedCalculator());
            break;
        case CalculatorType::ORIENTATION:
            return CalculatorPtr(new OrientationBasedCalculator());
            break;
        case CalculatorType::COMBINATION:
            return CalculatorPtr(new ComboCalculator());
            break;
    }
}

bool FABRIK::solve(FabrikOutput& output)
{
   
// e_{n-1}_w is the end_effector, the end frame of the last link
const Eigen::Affine3d& end_effector = robot_state_->getChain().back().getLinkFrame();
double target_ee_error = calculator_->calculateError(end_effector, target_); 

int iteration_num = 0;
while (target_ee_error > threshold_ || (iteration_num == requested_iteration_num_))
{
    ++iteration_num;

    // do one backward reaching
    FABRIK::backwardReaching();
    
    // do one forward reaching
    FABRIK::forwardReaching();

    target_ee_error = calculator_->calculateError(end_effector, target_); 
}

output.final_iteration_num_ = iteration_num;
int dof = robot_state_->getDOF();
for(int k = 0; k < dof; ++k)
    output.joints_values_[k] = robot_state_->getJointsValues(k);
output.target_ee_error = target_ee_error;

return true;
}

void FABRIK::backwardReaching()
{
    robot_state_->setReachingDirection(robot_state::ReachingDirection::BACKWARD);
    // set the last frame in frames_ to the target
    robot_state_->updateState(target_);

    int dof = robot_state_->getDOF();   
    // J_0 is not calculated in backward reaching. That is why we loop through
    // J_(dof-1) ... J_1
    for(int joint_number = dof - 1; joint_number > 0; --joint_number)
    {
        // Example: dof = 6. to claculate J_2, we need e1, s1 and e0
        // - e1 should be set equal to s2. Have 0 for joint value of J_2
        // - update s1 based on this new e1
        // - project e1_s1 and e1_e0
        // - find the angel between the two lines
        // - update e1 and s1
        Eigen::Affine3d s_2 = robot_state_->getFrames(joint_number).first;
        Eigen::Affine3d e_1_new = s_2;
        Eigen::Affine3d link_frame_1 = robot_state_->getLink(joint_number - 1).getLinkFrame();
        Eigen::Affine3d s_1_new = e_1_new * link_frame_1.inverse();

        Eigen::Affine3d e_0;
        if (joint_number == 1)
        {
            e_0 = robot_state_->getBase();
        }else
        {
            e_0 = robot_state_->getFrames(joint_number - 2).second;
        }

        double reaching_angle = calculator_->calculateReach(e_1_new,
                                                            s_1_new,
                                                            e_0);
        
        // We do not need to negate reaching_angle. updateState will update the right frames
        // in the right way based on the reaching direction set.
        robot_state_->updateState(reaching_angle, joint_number);
    }
}

void FABRIK::forwardReaching()
{
    robot_state_->setReachingDirection(robot_state::ReachingDirection::FORWARD);

    int dof = robot_state_->getDOF();   
    for(int joint_number = 0; joint_number < dof ; ++joint_number)
    {
        // Example: dof = 6. to claculate J_1, we need e1, s1 and s2
        // - s1 should be set equal to e0. Have 0 for joint value of J_1
        // - update e1 based on this new s1
        // - project s1_e1 and s1_s2
        // - find the angel between the two lines
        // - update e1 and s1
        Eigen::Affine3d e_0 = robot_state_->getFrames(joint_number - 1).second;
        Eigen::Affine3d s_1_new = e_0;
        Eigen::Affine3d link_frame_1 = robot_state_->getLink(joint_number).getLinkFrame();
        Eigen::Affine3d e_1_new = s_1_new * link_frame_1;

        Eigen::Affine3d s_2;
        if (joint_number == (dof - 1))
        {
            s_2 = target_;
        }else
        {
            s_2 = robot_state_->getFrames(joint_number + 1).first;
        }

        double reaching_angle = calculator_->calculateReach(s_1_new,
                                                            e_1_new,
                                                            s_2);
        
        robot_state_->updateState(reaching_angle, joint_number);
    }
}


}