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
    std::vector<double> solution_joints_values;

    /** \brief Some sort of error */
    double target_ee_error;

    /** \brief Number of iterations */
    int final_iteration_num;

    /** \brief a data structure to store all the frames in forward and backward reaching.
     * each row is a "frames_" and the first row is backward reaching followed by a forward
     * reaching in the second row. Obviously the last row is a forward reaching.
     */
    std::vector<std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>>> frames_matrix;
};


/** \brief This class includes the problem input, output and the solver
 */
class FABRIK
{
public:
    /** \brief Construct a FABRIK object */
    FABRIK( Eigen::Affine3d base,
            std::vector<robot_model::Link> chain,
            std::vector<double>& initial_configuration,
            Eigen::Affine3d& target,
            double threshold,
            double requested_iteration_num,
            CalculatorType calculator_type);
    
    /** \brief */
    bool solve(FabrikOutput& output);

    // how to get a const robot state to return from fabrik object ?????
    // robot_state::RobotStateConstPtr getConstRobotState()
    // {
    //     robot_state::RobotStateConstPtr const_robot_state = robot_state_;
    //     return 
    // }

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

void backwardReaching();

void forwardReaching();

};



}