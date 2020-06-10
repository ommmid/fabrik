/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "robot_model.h"

#include "exception.h"


namespace robot_model
{

Link::Link(const std::string& link_name, const Eigen::Affine3d& start_frame, const Eigen::Affine3d& end_frame):
link_name_(link_name), start_frame_(start_frame), end_frame_(end_frame)
{
}

Manipulator::Manipulator(const std::string& manipulator_name, std::vector<Link>& chain):
manipulator_name_(manipulator_name), chain_(chain)
{
    if(chain.size() == 0)
    {
         throw fabrik::Exception("manipulator can not be created by a chain with size 0");
    }
        
// I need link end and start frame in the world frame

// here I need to create the relative displacement to have the transformation to go from the 
// origin of each link to the end of it.
    for(auto link : chain)
    {
        
    }
}



}