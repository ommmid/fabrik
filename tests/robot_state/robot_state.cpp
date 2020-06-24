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


class Maker
{
public:

    Maker()
    {
        makeRobot();
        robot_state_ptr = std::make_shared<robot_state::RobotState>(chain_, base_);
    }

    void makeRobot()
    {
        Eigen::Vector3d vec1(5,1,3);
        vec1.normalize();
        Eigen::Affine3d link1_frame(Eigen::AngleAxisd(fabrik::randomDouble(0, 1), vec1));
        robot_model::Link link1("link1",  link1_frame);

        Eigen::Vector3d vec2(3,2,3);
        vec2.normalize();
        Eigen::Affine3d link2_frame(Eigen::AngleAxisd(fabrik::randomDouble(0, 1), vec2));
        robot_model::Link link2("link2",  link2_frame);

        Eigen::Vector3d vec3(1,6,4);
        vec3.normalize();
        Eigen::Affine3d link3_frame(Eigen::AngleAxisd(fabrik::randomDouble(0, 1), vec3));
        robot_model::Link link3("link3",  link3_frame);

        chain_.push_back(link1);
        chain_.push_back(link2);
        chain_.push_back(link3);

        Eigen::Vector3d vec0(1,6,2);
        vec0.normalize();
        Eigen::Affine3d base(Eigen::AngleAxisd(fabrik::randomDouble(0, 1), vec0));
        base.translate(Eigen::Vector3d(0.5, 0.5, 0.5));
        base_ = base;
    }

     robot_state::RobotStatePtr robot_state_ptr;

private:
    std::vector<robot_model::Link> chain_;
    Eigen::Affine3d base_;
};



BOOST_AUTO_TEST_CASE(EmptyChain)
{
    std::cout << "========== Empty Chain" << std::endl;
    std::vector<robot_model::Link> chain;
    const Eigen::Affine3d& base = Eigen::Affine3d::Identity();
    
    BOOST_CHECK_THROW(robot_state::RobotState(chain, base), std::exception);
}

BOOST_AUTO_TEST_CASE(RobotStateConstructor)
{
    std::cout << "========== Robot State Constructor ==========" << std::endl;

    Maker maker;
    robot_state::RobotStatePtr& robot_state = maker.robot_state_ptr;

    robot_state->printState("Empty Chain Test");
}

BOOST_AUTO_TEST_CASE(updateStateTarget)
 {
    std::cout << "========== Update State Target ==========" << std::endl;

    Maker maker;
    robot_state::RobotStatePtr& robot_state = maker.robot_state_ptr;

    // check updateState for setting target at FORWARD reaching
    robot_state->setReachingDirection(robot_state::ReachingDirection::FORWARD);
    Eigen::Affine3d target(Eigen::AngleAxisd(M_PI/7, Eigen::Vector3d::UnitZ()));

    BOOST_CHECK_THROW(robot_state->updateState(target), std::exception);
}

BOOST_AUTO_TEST_CASE(updateState)
 {
    std::cout << "========== Update State ==========" << std::endl;

    Maker maker;
    robot_state::RobotStatePtr& robot_state = maker.robot_state_ptr;
    robot_state->printState("update state test - initial - forward completed");

    // one backward reaching
    robot_state->setReachingDirection(robot_state::ReachingDirection::BACKWARD);

    robot_state->updateState(0.3, 2);
    robot_state->printState("update state test - update J_2 - forward");

    robot_state->updateState(0.4, 1);
    robot_state->printState("update state test - update J_1 - forward");

    BOOST_CHECK_THROW(robot_state->updateState(0.5, 0), std::exception);
    robot_state->printState("update state test - update J_0 - forward");

    // one forward reaching
    robot_state->setReachingDirection(robot_state::ReachingDirection::FORWARD);

    robot_state->updateState(0.6, 0);
    robot_state->printState("update state test - update J_0 - backward");

    robot_state->updateState(0.45, 1);
    robot_state->printState("update state test - update J_1 - backward");

    robot_state->updateState(0.26, 2);
    robot_state->printState("update state test - update J_2 - backward");
}