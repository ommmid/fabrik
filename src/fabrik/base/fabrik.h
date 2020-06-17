/* Author: Omid Heidari */
#pragma once

#include <memory>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "fabrik/robot_model/robot_model.h"
#include "fabrik/util/class_forward.h"

namespace fabrik
{  

FABRIK_CLASS_FORWARD(FABRIK);

/** \brief This class includes the problem input, output and the solver
 */
class FABRIK
{
public:
    /** \brief Construct a FABRIK object */
    FABRIK(robot_state::RobotStatePtr& robot_state,
           std::vector<double>& initial_configuration,
           Eigen::Affine3d& target);
    
    /** \brief */
    bool solve(const SolverType& solver_type, Output& output);

private:

robot_state::RobotStatePtr robot_state_;

/** \brief Joint values at the initial configuration. */
std::vector<double> initial_configuration_;

/** \brief The target we want to reach to */
Eigen::Affine3d target_;

double threshold_;
int requested_iteration_num_;

struct Output
{
    /** \brief Joints values at the solution */ 
    std::vector<double> joints_values_;

    /** \brief Some sort of error */

    /** \brief Number of iterations */
    int final_iteration_num_;
};

Output output_;

double calcError(const SolverType& solver_type, const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target);
double calcError(int(*calculator_type)(int), const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target);

void backwardReaching();

void forwardReaching();

};

enum SolverType
{
    POSITION,
    ORIENTATION,
    COMBINATION
}


class ErrorCalculator
{
    double shortestDistance(const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target);

    double closestOrientation(const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target);

    double combo(const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target);

}

class Solver
{
    shortestDistance(const Eigen::Affine3d& reaching_frame, const Eigen::Affine3d& aimed_frame);

    closestOrientation(const Eigen::Affine3d& reaching_frame, const Eigen::Affine3d& aimed_frame);

    combo(const Eigen::Affine3d& reaching_frame, const Eigen::Affine3d& aimed_frame);

}


}