/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "fabrik/robot_model/robot_model.h"

#include "fabrik/util/exception.h"


namespace fabrik
{

// -------------------- Link --------------------
Link::Link(const std::string& link_name, const Eigen::Affine3d& link_frame):
link_name_(link_name), link_frame_(link_frame)
{
}

Link::Link(const std::string& link_name, const Eigen::Affine3d& start_frame, const Eigen::Affine3d& end_frame):
link_name_(link_name)
{
    link_frame_ = start_frame.inverse() * end_frame;
}

// // -------------------- Chain --------------------
RobotModel::RobotModel(const std::string& robot_name, const Eigen::Affine3d& base, const std::vector<Link>& chain):
robot_name_(robot_name), base_(base), chain_(chain)
{   
    int dof = (int)chain_.size();

    if( dof == 0)
    {
         throw fabrik::Exception("a chain can not be created by 0 link");
    } 
    else
    {
        dof_ = dof;
    }
}

RobotModelPtr makeSimpleRobot2D()
{
    std::string robot_name = "simple 2D robot";
    std::vector<fabrik::Link> chain;
    Eigen::Affine3d base;

    Eigen::Vector3d vec1(0,0,1);
    vec1.normalize();
    Eigen::Affine3d link1_frame(Eigen::AngleAxisd(0, vec1));
    link1_frame.translation() = Eigen::Vector3d(1, 0, 0);
    fabrik::Link link1("link1",  link1_frame);

    Eigen::Vector3d vec2(0,0,1);
    vec2.normalize();
    Eigen::Affine3d link2_frame(Eigen::AngleAxisd(0, vec2));
    link2_frame.translation() = Eigen::Vector3d(1, 0, 0);
    fabrik::Link link2("link2",  link2_frame);

    Eigen::Vector3d vec3(0,0,1);
    vec3.normalize();
    Eigen::Affine3d link3_frame(Eigen::AngleAxisd(0, vec3));
    link3_frame.translation() = Eigen::Vector3d(1, 0, 0);
    fabrik::Link link3("link3",  link3_frame);

    chain.push_back(link1);
    chain.push_back(link2);
    chain.push_back(link3);
    
    Eigen::Vector3d vec0(0,0,1);
    vec0.normalize();
    Eigen::Affine3d base_transformation(Eigen::AngleAxisd(0, vec0));
    base_transformation.translate(Eigen::Vector3d(0, 0, 0));
    base = base_transformation;

    RobotModelPtr robot_model = std::make_shared<RobotModel>(robot_name, base, chain);
    return robot_model;
}

} // namespace fabrik