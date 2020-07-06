/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "fabrik/robot_model/robot_model.h"
#include "fabrik/robot_state/robot_state.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"
#include "fabrik/util/output.h"

#include "fabrik/base/fabrik.h"
#include "fabrik/base/calculator.h"

namespace fabrik
{

// ------------------------- FABRIK -------------------------
FABRIK::FABRIK( const RobotModelPtr& robot_model,
                std::vector<double>& initial_configuration,
                Eigen::Affine3d& target,
                double threshold,
                double requested_iteration_num,
                CalculatorType calculator_type):
initial_configuration_(initial_configuration), 
target_(target), threshold_(threshold), requested_iteration_num_(requested_iteration_num)
{
    robot_state_ = std::make_shared<fabrik::RobotState>(robot_model);

    // set the state of the robot to the given initial_configuration
    int dof = robot_model->getDOF();
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
std::cout << "========================== sooooolllvvvvveeeeee ======================" << std::endl;
std::cout << "======================================================================" << std::endl;
std::cout << "======================================================================" << std::endl;

int dof = robot_state_->getRobotModel()->getDOF();

// e_{n-1}_w is the end_effector, the end frame of the last link
Eigen::Affine3d end_effector = robot_state_->getFrames(dof - 1).second;
double target_ee_error = calculator_->calculateError(end_effector, target_); 
output.target_ee_error_track.push_back(target_ee_error);

std::cout << "initial error: \n" << target_ee_error << std::endl;
std::cout << "target: \n" << target_.matrix() << std::endl;
robot_state_->printState("fabrik::fabrik.cpp: Initial Configuration", std::vector<int>{0,1,2});

int iteration_num = 0;
while (target_ee_error > threshold_ && (iteration_num != requested_iteration_num_))
{
    std::cout << "------------------------ iteration number: " << iteration_num << std::endl;

    // do one backward reaching
    FABRIK::backwardReaching(output);
    output.frames_matrix.push_back(robot_state_->getFrames());

    robot_state_->printState("fabrik::fabrik.cpp: backwardReaching", std::vector<int>{0,1,2});

    // do one forward reaching
    FABRIK::forwardReaching(output);
    output.frames_matrix.push_back(robot_state_->getFrames());

    end_effector = robot_state_->getFrames(dof - 1).second;
    target_ee_error = calculator_->calculateError(end_effector, target_); 
    output.target_ee_error_track.push_back(target_ee_error);

    std::cout << "\ntarget_ee_error: " << target_ee_error << std::endl;
    robot_state_->printState("fabrik::fabrik.cpp: forwardReaching", std::vector<int>{0,1,2});

    ++iteration_num;
}

output.final_iteration_num = iteration_num;
output.solution_joints_values = robot_state_->getJointsValues();

output.target_ee_error = target_ee_error;

return true;
}

void FABRIK::backwardReaching(FabrikOutput& output)
{
    robot_state_->setReachingDirection(fabrik::ReachingDirection::BACKWARD);
    // set the last frame in frames_ to the target
    robot_state_->updateState(target_);

    int dof = robot_state_->getRobotModel()->getDOF(); 

    std::vector<Eigen::Vector3d> start_to_aim_vec(dof);
    std::vector<Eigen::Vector3d> start_to_aim_projected_vec(dof);
    std::vector<Eigen::Vector3d> start_to_end_vec(dof);
    std::vector<Eigen::Vector3d> start_to_end_projected_vec(dof);
    std::vector<double> angle_vec(dof);
      
    // J_0 is not calculated in backward reaching. That is why we loop through
    // J_(dof-1) ... J_1
    for(int joint_number = dof - 1; joint_number > 0; --joint_number)
    {
        // Example: to claculate J_2, we need e1, s1 and e0
        // - e1 should be set equal to s2. Have 0 for joint value of J_2
        // - update s1 based on this new e1
        // - project e1_s1 and e1_e0
        // - find the angel between the two lines
        // - update e1 and s1
        Eigen::Affine3d s_2 = robot_state_->getFrames(joint_number).first;
        Eigen::Affine3d e_1_new = s_2;
        Eigen::Affine3d link_frame_1 = robot_state_->getRobotModel()->getLink(joint_number - 1).getLinkFrame();
        Eigen::Affine3d s_1_new = e_1_new * link_frame_1.inverse();

        Eigen::Affine3d e_0;
        if (joint_number == 1)
        {
            e_0 = robot_state_->getRobotModel()->getBase();
        }else
        {
            e_0 = robot_state_->getFrames(joint_number - 2).second;
        }

        double reaching_angle = calculator_->calculateReach(e_1_new,
                                                            s_1_new,
                                                            e_0);
        
        // cast to derived class so I can extract the info about the position based calculator
        PositionBasedCalculatorPtr pbc = std::static_pointer_cast<PositionBasedCalculator>(calculator_);
        start_to_aim_vec[joint_number] = pbc->start_to_aim;
        start_to_aim_projected_vec[joint_number] = pbc->start_to_aim_projected;
        start_to_end_vec[joint_number] = pbc->start_to_end;
        start_to_end_projected_vec[joint_number]= pbc->start_to_end_projected;
        angle_vec[joint_number] = reaching_angle;

        // We do not need to negate reaching_angle. updateState will update the right frames
        // in the right way based on the reaching direction set.
        robot_state_->updateState(reaching_angle, joint_number);


        // robot_state_->printState("Backward Reaching .....", std::vector<int>{0,1,2});
    }
    output.start_to_aim_track.push_back(start_to_aim_vec);
    output.start_to_aim_projected_track.push_back(start_to_aim_projected_vec);
    output.start_to_end_track.push_back(start_to_end_vec);
    output.start_to_end_projected_track.push_back(start_to_end_projected_vec);
    output.angle_track.push_back(angle_vec);
    for(int s = 0; s < angle_vec.size(); ++s)
        std::cout << "angle: " << angle_vec[s] << std::endl;
}

void FABRIK::forwardReaching(FabrikOutput& output)
{
    robot_state_->setReachingDirection(fabrik::ReachingDirection::FORWARD);

    int dof = robot_state_->getRobotModel()->getDOF();

    std::vector<Eigen::Vector3d> start_to_aim_vec(dof);
    std::vector<Eigen::Vector3d> start_to_aim_projected_vec(dof);
    std::vector<Eigen::Vector3d> start_to_end_vec(dof);
    std::vector<Eigen::Vector3d> start_to_end_projected_vec(dof);
    std::vector<double> angle_vec(dof);
       
    for(int joint_number = 0; joint_number < dof ; ++joint_number)
    {
        // Example: dof = 6. to claculate J_1, we need e1, s1 and s2
        // - s1 should be set equal to e0. Have 0 for joint value of J_1
        // - update e1 based on this new s1
        // - project s1_e1 and s1_s2
        // - find the angel between the two lines
        // - update e1 and s1
        Eigen::Affine3d e_0;
        if (joint_number == 0)
        {
            e_0 = robot_state_->getRobotModel()->getBase();
        }else
        {
            e_0 = robot_state_->getFrames(joint_number - 1).second;            
        }
        
        Eigen::Affine3d s_1_new = e_0;
        Eigen::Affine3d link_frame_1 = robot_state_->getRobotModel()->getLink(joint_number).getLinkFrame();
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
        
        PositionBasedCalculatorPtr pbc = std::static_pointer_cast<PositionBasedCalculator>(calculator_);
        start_to_aim_vec[joint_number] = pbc->start_to_aim;
        start_to_aim_projected_vec[joint_number] = pbc->start_to_aim_projected;
        start_to_end_vec[joint_number] = pbc->start_to_end;
        start_to_end_projected_vec[joint_number] = pbc->start_to_end_projected;
        angle_vec[joint_number] = reaching_angle;

        robot_state_->updateState(reaching_angle, joint_number);

        // robot_state_->printState("Forward Reaching .....", std::vector<int>{0,1,2});
    }
    // start_to_aim has dof elements
    output.start_to_aim_track.push_back(start_to_aim_vec);
    output.start_to_aim_projected_track.push_back(start_to_aim_projected_vec);
    output.start_to_end_track.push_back(start_to_end_vec);
    output.start_to_end_projected_track.push_back(start_to_end_projected_vec);
    output.angle_track.push_back(angle_vec);
    for(int s = 0; s < angle_vec.size(); ++s)
        std::cout << "angle: " << angle_vec[s] << std::endl;
}


}