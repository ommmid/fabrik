/* Author: Omid Heidari */
#pragma once

#include <memory>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "fabrik/robot_model/robot_model.h"
#include "fabrik/util/class_forward.h"
#include "fabrik/base/calculator.h"

namespace fabrik
{  

FABRIK_CLASS_FORWARD(FABRIK);

enum CalculatorType
{
    POSITION,
    ORIENTATION,
    COMBINATION
};

struct FabrikOutput
{
    /** \brief Joints values at the solution */ 
    std::vector<double> joints_values_;

    /** \brief Some sort of error */

    /** \brief Number of iterations */
    int final_iteration_num_;
};


/** \brief This class includes the problem input, output and the solver
 */
class FABRIK
{
public:
    /** \brief Construct a FABRIK object */
    FABRIK(robot_state::RobotStatePtr& robot_state,
           std::vector<double>& initial_configuration,
           Eigen::Affine3d& target,
           double threshold,
           double requested_iteration_num,
           CalculatorType calculator_type);
    
    /** \brief */
    bool solve(FabrikOutput& output);

private:

robot_state::RobotStatePtr robot_state_;

/** \brief Joint values at the initial configuration. */
std::vector<double> initial_configuration_;

/** \brief The target we want to reach to */
Eigen::Affine3d target_;

double threshold_;
int requested_iteration_num_;

CalculatorPtr calculator_;

CalculatorPtr createCalculator(const CalculatorType& calculator_type);

// double calcError(const SolverType& solver_type, const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target);
// double calcError(int(*calculator_type)(int), const Eigen::Affine3d& end_effector, const Eigen::Affine3d& target);

void backwardReaching();

void forwardReaching();

};



}