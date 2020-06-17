#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Math"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include <boost/test/unit_test.hpp>

#include "fabrik/world/world.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"
#include "fabrik/robot_model/robot_model.h"
#include "fabrik/robot_state/robot_state.h"


BOOST_AUTO_TEST_CASE(Rotation_Z)
{
    double theta = 0;
    Eigen::Affine3d rotation_z = fabrik::rotation_z(theta);
    Eigen::MatrixXd rot_around_z(4,4);
    rot_around_z << std::cos(theta), -std::sin(theta), 0, 0,
                    std::sin(theta),  std::cos(theta), 0, 0,
                    0,                  0,             1, 0,
                    0,                  0,             0, 1;
    
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            BOOST_CHECK_EQUAL(rotation_z(i,j), rot_around_z(i,j));
        }
    }
}
