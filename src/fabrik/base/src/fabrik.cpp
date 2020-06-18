/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "fabrik/robot_state/robot_state.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"

#include "fabrik/base/fabrik.h"

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

    // set the calculator 
    calculator_ = createCalculator(calculator_type);
}

CalculatorPtr FABRIK::createCalculator(const CalculatorType& calculator_type)
{
    switch (calculator_type)
    {
        case CalculatorType::POSITION:
            CalculatorPtr calculator(new PositionBasedCalculator())
            return calculator;
            break;
        case CalculatorType::ORIENTATION:
            CalculatorPtr calculator(new OrientationBasedCalculator())
            return calculator;
            break;
        case CalculatorType::COMBINATION:
            CalculatorPtr calculator(new ComboCalculator())
            return calculator;
            break;
    }
}


bool FABRIK::solve(Output& output)
{
   
// e_{n-1}_w is the end_effector, the end frame of the last link
Eigen::Affine3d& end_effector = robot_state_->getChain().back();
double target_ee_error = calculator_->calculateError(end_effector, target); 

int iteration_num = 0;
while (target_ee_error > threshold_ || (iteration_num == requested_iteration_num_))
{
    ++iteration_num;

    // do one backward reaching
    FABRIK::backwardReaching();
    
    // do one forward reaching
    FABRIK::forwardReaching();

    target_ee_error = calculator_->calculateError(end_effector, target); 
}

output_.final_iteration_num_ = iteration_num;
output_.joints_values_ = ;
output_.error = ;

}

FABRIK::backwardReaching()
{
    robot_state_->setReachingDirection(robot_state::ReachingDirection::BACKWARD);

    int dof = robot_state_->getDOF();
    frames_[dof - 1].second = target_;
    // s_i = e_i * inverse(relative transformation of link_i)
    frames_[dof - 1].first = target_ * chain_[dof - 1].getLinkFrame().inverse();

    // J_0 is not calculated in backward reaching, that is why we loop through
    // J_(dof-1) ... J_1
    for(int joint_number = dof - 1; joint_number > 0; --joint_number)
    {
        // Example: dof = 6. to claculate J_5, we need e4, s4 and e3
        Eigen::Affine3d e_i_previous = frames_[joint_number - 1].second;
        Eigen::Affine3d s_i_previous = frames_[joint_number - 1].first;
        Eigen::Affine3d e_i_previous_previous;
        if (joint_number != 1)
        {
            e_i_previous_previous = frames_[joint_number - 2].second;
        }else
        {
            e_i_previous_previous = robot_state_->getBase();
        }

        double joint_value = calculator_->calculateReach(e_i_previous,
                                                         s_i_previous,
                                                         e_i_previous_previous);
        
        // This will update the right frames based on backward or forward reaching
        robot_state_->updateState(joint_value, joint_number);
    }
}

FABRIK::forwardReaching()
{
    robot_state_->setReachingDirection(robot_state::ReachingDirection::FORWARD);

    for(int joint_number = 0; joint_number < dof ; ++joint_number)
    {
        double joint_value = caclulateJointValue(); // based on solve type varies ????
        robot_state_->updateState(joint_value, joint_number);
    }
}


}