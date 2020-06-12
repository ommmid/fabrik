#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Robot Model"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include <boost/test/unit_test.hpp>

#include "fabrik/world/world.h"
#include "fabrik/util/exception.h"
#include "fabrik/robot_model/robot_model.h"

BOOST_AUTO_TEST_SUITE(Suite1)

BOOST_AUTO_TEST_CASE(GetFrame)
{
     worlds::world world_1(20);

    // world_1.describe();

    // Eigen::Affine3d start_frame(Eigen::Affine3d::Identity());
    // Eigen::Affine3d end_frame(Eigen::Affine3d::Identity());

    // std::string link3_name = "link_3";
    // robot_model::Link link(link3_name, start_frame, end_frame);

    // std::cout << "the homogenous matrix is: \n" << link.getStartFrame().matrix() << std::endl;

    // Eigen::Matrix3d r = link.getStartFrame().rotation();
    // std::cout << "the rotation matrix is: \n" << r << std::endl;

    // Eigen::Vector3d t = link.getStartFrame().translation();
    // std::cout << "the position vector is: \n" << t << std::endl;

    BOOST_CHECK_EQUAL(1, 1);
}


// test if the Exception works correctly, constructing the Manipulator with empty chain
BOOST_AUTO_TEST_CASE(Exception)
{
    BOOST_TEST_MESSAGE( "Created an " << "empty chain" );
    std::vector<robot_model::Link> robot_structure;
    
    BOOST_CHECK_THROW(robot_model::Chain("my_manipulator", robot_structure), 
                                                        std::exception);
}
BOOST_AUTO_TEST_SUITE_END()
