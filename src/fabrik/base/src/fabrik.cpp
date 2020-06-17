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
            CalculatorPtr out(new PositionBasedCalculator())
            return out;
            break;
        case CalculatorType::ORIENTATION:
            CalculatorPtr out(new OrientationBasedCalculator())
            return out;
            break;
        case CalculatorType::COMBINATION:
            CalculatorPtr out(new ComboCalculator())
            return out;
            break;
    }
}

// double FABRIK::calcError(double(*calculator)(const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target))
// {
//     return calculator(const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target));
// }

bool FABRIK::solve(Output& output)
{
   
// e_{n-1}_w is the end_effector, the end frame of the last link
Eigen::Affine3d& end_effector = robot_state_->getChain().back();
double error = calculator_->calculateError(end_effector, target); // based on solver type varies ????

int iteration_num = 0;
while (err > threshold_ || (iteration_num == requested_iteration_num_))
{
    ++iteration_num;

    // do one backward reaching
    FABRIK::backwardReaching();
    
    // do one forward reaching
    FABRIK::forwardReaching();

    calcError()
}

output_.final_iteration_num_ = iteration_num;
output_.joints_values_ = ;
output_.error = ;

}

FABRIK::backwardReaching()
{
    robot_state_->setReachingDirection(robot_state::ReachingDirection::BACKWARD);

    // J_0 is not calculated in backward reaching, that is why we loop through
    // 0 ... (dof - 2)
    for(int joint_number = 0; joint_number < dof - 1; ++joint_number)
    {

        double joint_value = caclulateJointValue(); // based on solve type varies ????
        robot_state_->updateFrame(joint_value, joint_number);
    }
}

FABRIK::forwardReaching()
{
    robot_state_->setReachingDirection(robot_state::ReachingDirection::FORWARD);

    for(int joint_number = 0; joint_number < dof ; ++joint_number)
    {
        double joint_value = caclulateJointValue(); // based on solve type varies ????
        robot_state_->updateFrame(joint_value, joint_number);
    }
}

// ------------------------- ErrorCalculator -------------------------
double ErrorCalculator::shortestDistance(const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target)
{
    Eigen::Vector3d target_to_end_effector = end_effector.translation() - target.translation();
    return target_to_end_effector.norm();
}

}