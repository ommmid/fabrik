/* Author: Omid Heidari */
#pragma once

#include <memory>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "fabrik/robot_model/robot_model.h"
#include "fabrik/util/class_forward.h"
#include "fabrik/base/calculator.h"
#include "fabrik/util/io.h"

#include "fabrik/robot_state/robot_state.h"

namespace fabrik
{  

FABRIK_CLASS_FORWARD(FABRIK);

/** \brief This class includes the problem input, output and the solver
 */
class FABRIK
{
public:
    /** \brief Construct a FABRIK object with a robot_model
     * robot_state will be set to all-joint-zero position
     */
    FABRIK( const RobotModelPtr& robot_model);

    /** \brief Construct a FABRIK object with a robot_model
     * robot_state will be set to initial configuration
     */
    FABRIK( const RobotModelPtr& robot_model, const std::vector<double>& initial_configuration);
    
    void setInverseKinematicsInput(const Eigen::Affine3d& target,
                                   const double threshold,
                                   const double requested_iteration_num,
                                   const CalculatorType calculator_type);

    /** \brief Solve inverse kinematics */
    bool solveIK(IKOutput& output);

    /** \brief Solve forward kinematics */
    bool solveFK(const std::vector<double>& configuration, Eigen::Affine3d& target);

    // TODO[Omid]: how to get a const robot state to return from fabrik object ?????
    // fabrik::RobotStateConstPtr getConstRobotState()
    // {
    //     fabrik::RobotStateConstPtr const_robot_state = robot_state_;
    //     return 
    // }

private:

fabrik::RobotStatePtr robot_state_;

// int dof_;

/** \brief Joint values at the initial configuration. */
std::vector<double> initial_configuration_;

/** \brief The target we want to reach to */
Eigen::Affine3d target_;

double threshold_;
int requested_iteration_num_;

CalculatorPtr calculator_;

CalculatorPtr createCalculator(const CalculatorType& calculator_type);

void backwardReaching(IKOutput& output);

void forwardReaching(IKOutput& output);

};



}