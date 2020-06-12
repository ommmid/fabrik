#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "World"

#include <iostream>
#include <boost/test/unit_test.hpp>


#include "fabrik/world/world.h"


// To generate and initialize a main function to call my tests and monitor their status
// no main function needed
BOOST_AUTO_TEST_CASE(World_Simple)
{
    worlds::world world_1(20);

    world_1.describe();

    std::cerr << "hereeeeeeeeeeeeee" << std::endl;

    BOOST_CHECK_EQUAL(1, 1);
}


BOOST_AUTO_TEST_CASE(World_dummy)
{
    worlds::world world_1(20);

    world_1.describe();

    BOOST_CHECK_EQUAL(1, 1);
}