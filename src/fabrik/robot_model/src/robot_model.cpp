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
// Chain::Chain(const std::string& chain_name, const std::vector<Link>& chain_structure):
// chain_name_(chain_name), chain_structure_(chain_structure)
// {   
//     int dof = (int)chain_structure_.size();

//     if( dof == 0)
//     {
//          throw fabrik::Exception("a chain can not be created by 0 link");
//     } 
//     else
//     {
//         dof_ = dof;
//     }
        
// }



}