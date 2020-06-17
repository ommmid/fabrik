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
           double requested_iteration_num):
robot_state_(robot_state), initial_configuration_(initial_configuration), 
target_(target), threshold_(threshold), requested_iteration_num_(requested_iteration_num)
{
    // set the state of the robot to the given initial_configuration
}

double FABRIK::calcError(const SolverType& solver_type, const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target)
{
    switch (solver_type)
    {
        case SolverType::POSITION:
            return FABRIK::calcError(ErrorCalculator::shortestDistance, const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target);
            break;
        case SolverType::ORIENTATION:
            return FABRIK::calcError(ErrorCalculator::nearestOrientation, const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target);
            break;
        case SolverType::COMBINATION:
            return FABRIK::calcError(ErrorCalculator::combo, const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target);
            break;
    }
}

double FABRIK::calcError(double(*calculator)(const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target))
{
    return calculator(const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target));
}

bool FABRIK::solve(const SolverType& solver_type, Output& output)
{
   

// e_{n-1}_w is the end_effector
Eigen::Affine3d& end_effector = robot_state_->getChain().back();
double err = calcError(solver_type, end_effector, target); // based on solver type varies ????

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