/* Author: Omid Heidari */
#pragma once

#include <Eigen/Geometry>

#include <memory>

#include <vector>

/** \brief Core components of FABRIK */
namespace robot_model
{

/** \brief Link has two frames. In the beginning and end of each link
 * with z-axis being the same as joint direction. All the links of the manipulators
 * have joint at both ends except the end-effector and the base link.
 */
class Link
{
public:
    /** \brief Construct a Link by a two homogeneous transformations (Affine3D) */
    Link(const std::string& link_name, const Eigen::Affine3d& start_frame, const Eigen::Affine3d& end_frame);

    /** \brief Destructor. Clear all memory. */
    // ~Manipulator();

    const std::string getManipulatorName() const  { return link_name_; }
    const Eigen::Affine3d getStartFrame() const  { return start_frame_; }
    const Eigen::Affine3d getEndFrame() const  { return end_frame_; }

    // do I need destructor?
    // do I need copy constructor?  

protected:
    /** \brief The name of the manipulator. */
    std::string link_name_;

    /** \brief The frame at the beginning of the link. */
    Eigen::Affine3d start_frame_;

    /** \brief The frame at the end of the link. */
    Eigen::Affine3d end_frame_;
};

/** \brief Manipulator model including Links. */
class Manipulator
{
public:
    /** \brief Construct a Manipulator by a series of Links */
    Manipulator(const std::string& manipulator_name,
    std::vector<Link>& chain);
   
protected:
    /** \brief The name of the manipulator. */
    std::string manipulator_name_;

    std::vector<Link> chain_;
};


typedef std::shared_ptr<Link> LinkPtr;
typedef std::shared_ptr<const Link> LinkConstPtr;
typedef std::shared_ptr<Manipulator> ManipulatorPtr;
typedef std::shared_ptr<const Manipulator> ManipulatorConstPtr;


enum Error
{
    SUCCESS = 0,
    INVALID_CHAIN = 1
};

}