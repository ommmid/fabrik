/* Author: Omid Heidari */
#pragma once

#include <memory>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "fabrik/robot_model/robot_model.h"
#include "fabrik/util/class_forward.h"
#include "fabrik/base/calculator.h"
#include "fabrik/util/output.h"

#include "fabrik/robot_state/robot_state.h"

namespace fabrik
{  

FABRIK_CLASS_FORWARD(FABRIK);

enum CalculatorType
{
    POSITION,
    ORIENTATION,
    COMBINATION
};


/** \brief This class includes the problem input, output and the solver
 */
class FABRIK
{
public:
    /** \brief Construct a FABRIK object */
    FABRIK( const RobotModelPtr& robot_model,
            std::vector<double>& initial_configuration,
            Eigen::Affine3d& target,
            double threshold,
            double requested_iteration_num,
            CalculatorType calculator_type);
    
    /** \brief */
    bool solve(FabrikOutput& output);

    // how to get a const robot state to return from fabrik object ?????
    // fabrik::RobotStateConstPtr getConstRobotState()
    // {
    //     fabrik::RobotStateConstPtr const_robot_state = robot_state_;
    //     return 
    // }

private:

fabrik::RobotStatePtr robot_state_;

/** \brief Joint values at the initial configuration. */
std::vector<double> initial_configuration_;

/** \brief The target we want to reach to */
Eigen::Affine3d target_;

double threshold_;
int requested_iteration_num_;

CalculatorPtr calculator_;

CalculatorPtr createCalculator(const CalculatorType& calculator_type);

void backwardReaching(FabrikOutput& output);

void forwardReaching(FabrikOutput& output);

};



}