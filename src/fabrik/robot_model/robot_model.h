/* Author: Omid Heidari */
#pragma once

#include <Eigen/Geometry>

#include <memory>

#include <vector>

#include "fabrik/util/class_forward.h"


/** \brief Modeling robot parts */
namespace fabrik
{

enum FabrikError
{
    SUCCESS = 0,
    INVALID_CHAIN = 1
};

FABRIK_CLASS_FORWARD(RobotModel);


/** \brief Link has two frames. In the beginning {s_i} and end {e_i} of each link
 * with z-axis being the same as joint direction expressing both in world frame.
 * All the links of the manipulators
 * have joint at both ends except the end-effector and the base link.
 * Each link can be defined fully by a relative transformation between {s_i} and {e_i};
 * writing {e_i} in {s_i} frame.
 */
class Link
{
public:
    /** \brief Construct a Link by a relative transformation representing the structure of the link
     */
    Link(const std::string& link_name, const Eigen::Affine3d& link_frame);

    /** \brief Construct a Link by two homogeneous transformations at start and end part of the link
     * these frames are expressed in world coordinate
     */
    Link(const std::string& link_name, const Eigen::Affine3d& start_frame, const Eigen::Affine3d& end_frame);

    const std::string getLinkName() const  { return link_name_; }
    const Eigen::Affine3d getLinkFrame() const  { return link_frame_; }
    
    // do I need copy constructor?  

protected:
    /** \brief The name of the manipulator. */
    std::string link_name_;

    /** \brief a relative transformation representing the structure of the line, the relationship
     * between the start frame and end frame. This frame is expressed in the start frame
     *  of the link not in the world coordinate*/
    Eigen::Affine3d link_frame_;
};

/** \brief Robot model including Links. */
class RobotModel
{
public:
    /** \brief Construct a Chaun by a series of Links relative transformation and assigns 
     * default names to the links
     */
    RobotModel(const std::string& robot_name, const Eigen::Affine3d& base, const std::vector<Link>& chain);
   
    /** \brief Get the robot struacture which is the set of links */
    const std::vector<Link> getRobotChain() const
    {
        return chain_;
    }

    /** \brief Get dof */
    const int getDOF() const
    {
        return dof_;
    }

    const Link getLink(int index) const
    {
        return chain_[index];
    }

    const Eigen::Affine3d getBase() const
    {
        return base_;
    }

 protected:
    /** \brief The name of the manipulator. */
    std::string robot_name_;

    /** \brief The base of the robot. The frame that is fixed and attached to the world expressed
     * in the world frame.
     */
    Eigen::Affine3d base_;

    /** \brief A list of link frames (realative transformations of each link) representing the 
     * structure of the robot. */
    std::vector<Link> chain_;

    /** \brief The degrees of freedom of the chain */
    int dof_;
};

RobotModelPtr makeSimpleRobot2D();


}