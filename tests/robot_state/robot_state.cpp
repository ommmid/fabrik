#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Robot State"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include <boost/test/unit_test.hpp>

#include "fabrik/world/world.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"
#include "fabrik/robot_model/robot_model.h"
#include "fabrik/robot_state/robot_state.h"


BOOST_AUTO_TEST_CASE(RobotState)
{
    // BOOST_TEST_MESSAGE( "Created an " << "empty chain" );
    Eigen::Vector3d vec1(5,1,3);
    fabrik::make_unit_vector3d(vec1);
    Eigen::Affine3d link1_frame(Eigen::AngleAxisd(fabrik::random_double(0, 1), vec1));
    robot_model::Link link1("link1",  link1_frame);

    Eigen::Vector3d vec2(3,2,3);
    fabrik::make_unit_vector3d(vec2);
    Eigen::Affine3d link2_frame(Eigen::AngleAxisd(fabrik::random_double(0, 1), vec2));
    robot_model::Link link2("link2",  link2_frame);

    Eigen::Vector3d vec3(1,6,4);
    fabrik::make_unit_vector3d(vec3);
    Eigen::Affine3d link3_frame(Eigen::AngleAxisd(fabrik::random_double(0, 1), vec3));
     robot_model::Link link3("link3",  link3_frame);

    std::vector<robot_model::Link> chain;
    chain.push_back(link1);
    chain.push_back(link2);
    chain.push_back(link3);
    
    Eigen::Vector3d vec0(1,6,2);
    fabrik::make_unit_vector3d(vec0);
    Eigen::Affine3d base(Eigen::AngleAxisd(fabrik::random_double(0, 1), vec0));
    base.translate(Eigen::Vector3d(0.5, 0.5, 0.5));
    robot_state::RobotState robot_state(chain, base);

    BOOST_TEST_MESSAGE("... dof: " << robot_state.getDOF()); 

    std::cout << "... dof: " << robot_state.getDOF() << std::endl;
    std::cout << "... reaching direction: " << robot_state.getReachingDirection() << std::endl;
    std::cout << "... reaching at: " << robot_state.getReachingAt() << std::endl;
    for (int i = 0; i < robot_state.getDOF(); ++i)
        std::cout << "joint value " << i << " " << robot_state.getJointsValues()[i] << std::endl;

    // BOOST_CHECK_THROW(robot_model::Chain("my_manipulator", robot_structure), 
    //                                                     std::exception);
}
