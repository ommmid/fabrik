
#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include "world.h"
#include "robot_model.h"



int main(){
    std::cout << "Hello, from testing world!\n";

    worlds::world world_1(20);

    world_1.describe();

    Eigen::Affine3d start_frame(Eigen::Affine3d::Identity());
    Eigen::Affine3d end_frame(Eigen::Affine3d::Identity());

    std::string link3_name = "link_3";
    core::Link link(link3_name, start_frame, end_frame);

    std::cout << "the homogenous matrix is: \n" << link.getStartFrame().matrix() << std::endl;

    Eigen::Matrix3d r = link.getStartFrame().rotation();
    std::cout << "the rotation matrix is: \n" << r << std::endl;

    Eigen::Vector3d t = link.getStartFrame().translation();
    std::cout << "the position vector is: \n" << t << std::endl;

    // test if the Exception works
    std::vector<core::Link> chain;
    core::Manipulator("my_manipulator", chain);
    std::cout << "is the program aborted or not" << std::endl;
}
