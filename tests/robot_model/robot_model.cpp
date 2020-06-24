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
        

    BOOST_CHECK_EQUAL(1, 1);
}


BOOST_AUTO_TEST_SUITE_END()
