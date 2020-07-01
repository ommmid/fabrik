#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Fabrik"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include <boost/test/unit_test.hpp>

#include "fabrik/world/world.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"
#include "fabrik/robot_model/robot_model.h"
#include "fabrik/robot_state/robot_state.h"

#include "fabrik/base/calculator.h"
#include "fabrik/base/fabrik.h"


const double TRANSLATION_ERROR = 1e-6;
const double ANGLE_ERROR = 1e-5;

class MakerPlanar
{
public:

    MakerPlanar()
    {
        makeChain();
        makeBase();
    }

    void makeChain()
    {
        Eigen::Vector3d vec1(0,0,1);
        vec1.normalize();
        Eigen::Affine3d link1_frame(Eigen::AngleAxisd(0, vec1));
        link1_frame.translation() = Eigen::Vector3d(1, 0, 0);
        robot_model::Link link1("link1",  link1_frame);

        Eigen::Vector3d vec2(0,0,1);
        vec2.normalize();
        Eigen::Affine3d link2_frame(Eigen::AngleAxisd(0, vec2));
        link2_frame.translation() = Eigen::Vector3d(1, 0, 0);
        robot_model::Link link2("link2",  link2_frame);

        Eigen::Vector3d vec3(0,0,1);
        vec3.normalize();
        Eigen::Affine3d link3_frame(Eigen::AngleAxisd(0, vec3));
        link3_frame.translation() = Eigen::Vector3d(1, 0, 0);
        robot_model::Link link3("link3",  link3_frame);

        chain.push_back(link1);
        chain.push_back(link2);
        chain.push_back(link3);
    }

    void makeBase()
    {
        Eigen::Vector3d vec0(0,0,1);
        vec0.normalize();
        Eigen::Affine3d base_transformation(Eigen::AngleAxisd(0, vec0));
        base_transformation.translate(Eigen::Vector3d(0, 0, 0));
        base = base_transformation;
    }

    std::vector<robot_model::Link> chain;
    Eigen::Affine3d base;
};

class MakerSpatial
{
public:

    MakerSpatial()
    {
        makeChain();
        makeBase();
    }

    void makeChain()
    {
        Eigen::Vector3d vec1(5,1,3);
        vec1.normalize();
        Eigen::Affine3d link1_frame(Eigen::AngleAxisd(fabrik::randomDouble(0, 1), vec1));
        link1_frame.translation() = Eigen::Vector3d(1,2.2,3);
        robot_model::Link link1("link1",  link1_frame);

        Eigen::Vector3d vec2(3,2,3);
        vec2.normalize();
        Eigen::Affine3d link2_frame(Eigen::AngleAxisd(fabrik::randomDouble(0, 1), vec2));
        link2_frame.translation() = Eigen::Vector3d(1.7, 3, 2.5);
        robot_model::Link link2("link2",  link2_frame);

        Eigen::Vector3d vec3(1,6,4);
        vec3.normalize();
        Eigen::Affine3d link3_frame(Eigen::AngleAxisd(fabrik::randomDouble(0, 1), vec3));
        link3_frame.translation() = Eigen::Vector3d(0.5, 4, 1);
        robot_model::Link link3("link3",  link3_frame);

        chain.push_back(link1);
        chain.push_back(link2);
        chain.push_back(link3);
    }

    void makeBase()
    {
        Eigen::Vector3d vec0(0,1,1);
        vec0.normalize();
        Eigen::Affine3d base_transformation(Eigen::AngleAxisd(0, vec0));
        base_transformation.translate(Eigen::Vector3d(0.5, 0.5, 0.5));
        base = base_transformation;
    }

    std::vector<robot_model::Link> chain;
    Eigen::Affine3d base;
};


BOOST_AUTO_TEST_CASE(FABRIK2D)
{
    MakerPlanar maker;

    // ---------------------- Solve a forward kinematics first:
    robot_state::RobotStatePtr robot_state_1 = 
        std::make_shared<robot_state::RobotState>(maker.chain, maker.base);

    robot_state_1->setReachingDirection(robot_state::ReachingDirection::FORWARD);
    std::vector<double> fk_joints_values_1 = {M_PI_4, 0, 0};
    for (int k = 0; k < 3; ++k)
        robot_state_1->updateState(fk_joints_values_1[k], k);

    robot_state_1->printState("FABRIK - first configuration", std::vector<int>{-1});
    
    Eigen::Affine3d end_effector_1 = robot_state_1->getFrames(2).second;
    Eigen::Vector3d expected_location_1(3 * std::cos(M_PI_4),
                                      3 * std::sin(M_PI_4),
                                      0);

    for(int k = 0; k < 3; ++k)
    {
        BOOST_TEST(abs(end_effector_1.translation()[k] - expected_location_1(k)) < TRANSLATION_ERROR);
    }

    // ---------------------- Solve another forward kinematics close to the first one:
    robot_state::RobotStatePtr robot_state_2 = 
        std::make_shared<robot_state::RobotState>(maker.chain, maker.base);
    
    robot_state_2->setReachingDirection(robot_state::ReachingDirection::FORWARD);
    double theta_1 = M_PI_4 + 0.1;
    double theta_2 = 0.1;
    double theta_3 = 0.1;
    std::vector<double> fk_joints_values_2 = {theta_1, theta_2, theta_3};
    for (int k = 0; k < 3; ++k)
        robot_state_2->updateState(fk_joints_values_2[k], k);

    robot_state_2->printState("FABRIK - second configuration", std::vector<int>{-1});

    Eigen::Affine3d end_effector_2 = robot_state_2->getFrames(2).second;
    Eigen::Vector3d expected_location_2(
        std::cos(theta_1) + std::cos(theta_1 + theta_2) + std::cos(theta_1 + theta_2 + theta_3),
        std::sin(theta_1) + std::sin(theta_1 + theta_2) + std::sin(theta_1 + theta_2 + theta_3),
                                      0);

    for(int k = 0; k < 3; ++k)
    {
        BOOST_TEST(abs(end_effector_2.translation()[k] - expected_location_2(k)) < TRANSLATION_ERROR);
    }

    // get the end effector frame of the second state
    Eigen::Affine3d target = end_effector_2;
    double threshold = 0.001;
    double requested_iteration_num = 3;

    fabrik::FABRIKPtr fabrik(new fabrik::FABRIK(maker.base,
                                                maker.chain,
                                                fk_joints_values_1,
                                                target,
                                                threshold,
                                                requested_iteration_num,
                                                fabrik::CalculatorType::POSITION));
    
    fabrik::FabrikOutput output;
    bool solved = fabrik->solve(output);

    std::cout << "solved? " << solved << std::endl;
    if(solved)
    {
        std::cout << "total iteration: " << output.final_iteration_num << std::endl;
        std::cout << "error: " << output.target_ee_error << std::endl;
        for (int k = 0; k < 3; ++k)
            std::cout << "joint value_" << k << ":" << output.solution_joints_values[k] << std::endl;
    }

    for (int i = 0; i < 2 * requested_iteration_num; ++i)
    {
        std::cout << "-------------- row number: " << i << std::endl;
        for (int j = 0; j < 3; ++j)
        {
            std::cout << "start: \n" << output.frames_matrix[i][j].first.translation() << std::endl;
            std::cout << "end: \n" << output.frames_matrix[i][j].second.translation()  << std::endl;
        }
    }

}

BOOST_AUTO_TEST_CASE(FABRIK3D)
{
    // MakerSpatial maker;

    // // Solve a forward kinematics first:
    // robot_state::RobotStatePtr robot_state_1 = 
    //     std::make_shared<robot_state::RobotState>(maker.chain, maker.base);

    // robot_state_1->setReachingDirection(robot_state::ReachingDirection::FORWARD);
    // std::vector<double> fk_joints_values_1 = {0.3, 0.5, 0.4};
    // for (int k = 0; k < 3; ++k)
    //     robot_state_1->updateState(fk_joints_values_1[k], k);

    // robot_state_1->printState("FABRIK - first configuration", std::vector<int>{-1});
    
    // // Solve another forward kinematics close to the first one:
    // robot_state::RobotStatePtr robot_state_2 = 
    //     std::make_shared<robot_state::RobotState>(maker.chain, maker.base);
    
    // robot_state_2->setReachingDirection(robot_state::ReachingDirection::FORWARD);
    // std::vector<double> fk_joints_values_2 = {0.35, 0.52, 0.43};
    // for (int k = 0; k < 3; ++k)
    //     robot_state_2->updateState(fk_joints_values_2[k], k);

    // robot_state_2->printState("FABRIK - second configuration", std::vector<int>{-1});

    // // get the end effector frame of the second state
    // Eigen::Affine3d target = robot_state_2->getFrames(2).second;
  
    // double threshold = 0.001;
    // double requested_iteration_num = 10;

    // fabrik::FABRIKPtr fabrik(new fabrik::FABRIK(maker.base,
    //                                             maker.chain,
    //                                             fk_joints_values_1,
    //                                             target,
    //                                             threshold,
    //                                             requested_iteration_num,
    //                                             fabrik::CalculatorType::POSITION));
    
    // fabrik::FabrikOutput output(maker.chain.size());
    // bool solved = fabrik->solve(output);

    // std::cout << "solved? " << solved << std::endl;
    // if(solved)
    // {
    //     std::cout << "total iteration" << output.final_iteration_num << std::endl;
    //     std::cout << "error" << output.target_ee_error << std::endl;
    //     for (int k = 0; k < 3; ++k)
    //         std::cout << "joint value_" << k << ":" << output.joints_values[k] << std::endl;
    // }

}

