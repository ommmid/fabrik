#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Calculator"

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

const double TRANSLATION_ERROR = 1e-6;
const double ANGLE_ERROR = 1e-5;

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



BOOST_AUTO_TEST_CASE(CalculateError)
 {
    std::cout << "========== Calculate Error ==========" << std::endl;

    fabrik::CalculatorPtr calculator(new fabrik::PositionBasedCalculator());

    // no distance in translation for frame_1 and frame_2
    Eigen::Affine3d frame_1(Eigen::AngleAxisd(0.30, Eigen::Vector3d(5,1,2)));
    Eigen::Affine3d frame_2(Eigen::AngleAxisd(0.45, Eigen::Vector3d(9,6,2)));
    double err = calculator->calculateError(frame_1, frame_2);
    BOOST_TEST(std::abs(err) < TRANSLATION_ERROR);

    Eigen::Vector3d translation_1(1,1,1);
    frame_1.translation() = translation_1;    
    Eigen::Vector3d translation_2(1,1,-1);
    frame_2.translation() = translation_2;
    
    err = calculator->calculateError(frame_1, frame_2);
    double distance = (translation_1 - translation_2).norm();

    std::cout << "error: " << err << "   distance: " << distance << std::endl;
    BOOST_TEST(std::abs(err - distance) < TRANSLATION_ERROR);
}


BOOST_AUTO_TEST_CASE(CalculateReach2D)
{
    std::cout << "========== Calculate Reach 2D ==========" << std::endl;

    fabrik::CalculatorPtr calculator(new fabrik::PositionBasedCalculator());

    // frames in 2D so I can calculate the reaching angle manulally easier
    Eigen::Affine3d start_frame_reaching(Eigen::AngleAxisd(0.30, Eigen::Vector3d(0,0,1)));
    start_frame_reaching.translation() = Eigen::Vector3d(0,0,0);

    Eigen::Affine3d end_frame_reaching(Eigen::AngleAxisd(0.40, Eigen::Vector3d(0,0,1)));
    end_frame_reaching.translation() = Eigen::Vector3d(1,1,0);

    Eigen::Affine3d frame_aimed_at(Eigen::AngleAxisd(0.50, Eigen::Vector3d(0,0,1)));
    frame_aimed_at.translation() = Eigen::Vector3d(-2,2,0);

    // The angle shoul be pi/2
    double angle = calculator->calculateReach(start_frame_reaching,
                               end_frame_reaching,
                               frame_aimed_at);

    std::cout << "reaching angle: " << angle * 180 / M_PI << std::endl;
    BOOST_TEST((angle - M_PI_2) < ANGLE_ERROR);
}

BOOST_AUTO_TEST_CASE(CalculateReach3D)
{
    std::cout << "========== Calculate Reach 3D ==========" << std::endl;

    fabrik::CalculatorPtr calculator(new fabrik::PositionBasedCalculator());

    // frames in 3D so I can calculate the reaching angle manually easier
    Eigen::Affine3d start_frame_reaching(Eigen::AngleAxisd(0.30, Eigen::Vector3d(0,0,1)));
    start_frame_reaching.translation() = Eigen::Vector3d(0,0,0);

    Eigen::Affine3d end_frame_reaching(Eigen::AngleAxisd(0.40, Eigen::Vector3d(3,1,1)));
    end_frame_reaching.translation() = Eigen::Vector3d(1,1,1);

    Eigen::Affine3d frame_aimed_at(Eigen::AngleAxisd(0.50, Eigen::Vector3d(2,0,1)));
    frame_aimed_at.translation() = Eigen::Vector3d(-2,2,2);

    double angle = calculator->calculateReach(start_frame_reaching,
                               end_frame_reaching,
                               frame_aimed_at);

    std::cout << "reaching angle: " << angle * 180 / M_PI << std::endl;
    BOOST_TEST((angle - M_PI_2) < ANGLE_ERROR);
}

