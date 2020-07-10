/* Author: Omid Heidari */
#pragma once

#include <memory>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "fabrik/robot_model/robot_model.h"
#include "fabrik/util/class_forward.h"

namespace fabrik
{
    
enum ReachingDirection
{
    FORWARD,
    BACKWARD
};


// I want forward decleration for RobotState becuase I predict it will be a heavy class
FABRIK_CLASS_FORWARD(RobotState);

/** \brief This class determines that what state our chain is at. Considering FABRIK
 * algorithm, the state of the robot are the things that we want to update.
 * Joint number and link number are the same. And the numbering starts from
 * 0 so we do not have difficulty in indexing
 */
class RobotState
{
public:
    /** \brief Construct a state at a forward-completed home configuratn. It means:
     * reached_at_ = dof - 1
     * reachin_direction_ = FORWARD
     * joint_values = set to zeros
     */
    RobotState(const RobotModelPtr& robot_model);

    /** \brief Construct a state at a forward-completed by a given configuration. It means:
     * reached_at_ = dof - 1
     * reachin_direction_ = FORWARD
     * joint_values = set to the given configuration
     */
    RobotState(const RobotModelPtr& robot_model, const std::vector<double>& given_configuration);

    // /** \brief Not sure if I need to create any other constructor
    //  */
    // RobotState(const std::vector<fabrik::Link>& chain,
    //            const Eigen::Affine3d& base,
    //            const ReachingDirection& reaching_direction);


    /** \brief This function updates only [s_i e_i] in frames_. We do not change the rest of the 
     * frames start_{i+1} and end_{i+1} ... because this function is used in
     * either forward or backward reaching, meaning it does not
     * calculate a full forward kinematics to reach to the end-effector. It is being disassembled
     * Joint number and linke number are the same. J1 is on s1. s1 and e1 are both on link 1
     * and they start from 0 to n-1, n being the number of degrees of freedom
     * It also updates the joints values first
     */ 
    void updateState(const double& joint_value, const int& joint_number);

    /** \brief Update the last frame in frames_ at the first step of reching backward. 
     * No joint value is needed. The only thing we need is the transformation of the last link
     */
    void updateState(Eigen::Affine3d target);

    int getReachedAt() const
    {
        return reached_at_;
    }

    ReachingDirection getReachingDirection() const
    {
        return reaching_direction_;
    }

    const std::vector<double> getJointsValues() const
    {
        return joints_values_;
    }

    const double getJointsValues(int index) const
    {
        return joints_values_[index];
    }

    const std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> getFrames() const
    {
        return frames_;
    } 

    std::pair<Eigen::Affine3d, Eigen::Affine3d> getFrames(int link_index) const
    {
        return frames_[link_index];
    } 

    const RobotModelPtr getRobotModel() const
    {
        return robot_model_;
    }

    void setReachingDirection(ReachingDirection reaching_direction)
    {
        reaching_direction_ = reaching_direction;
    }

    void setReachingAt(int reaching_at)
    {
        reached_at_ = reaching_at;
    }

    std::string createDashBoard() const;

    /** \brief print out the information of the current state 
     * \param text a text showing where this function is called from and more info if needed
     * \param which_frames which frames to show the transformation for
    */
    void printState(const std::string text, const std::vector<int>& which_frames) const;

private:
    /** \brief An integer showing the link number where the link start and end frame are updated
     * the link is at "reached" state
     */
    int reached_at_;

    /** \brief Reaching direction: forward or backward  */
    ReachingDirection reaching_direction_;

    /** \brief Degrees of freedom  */
    int dof_;

    /** \brief The value of each joint for the current state. Joint number start from 0. */
    std::vector<double> joints_values_;

    /** \brief Chain. The shared pointer is pointing to a const fabrik::Chain
     * because we wont change this chain (it is the structure of the robot)
     */
    RobotModelPtr robot_model_;

    std::vector<Link> chain_;

    /** \brief All frames of all links in order, expressed in world frame:
     * [s_0 e_0 s_1 e_1 .... s_n-1 e_n-1]. Because I want to keep the indecis easily 
     * to read. I will make pair for each start and end:
     * [[s_0 e_0] [s_1 e_1] .... [s_n-1 e_n-1]]
     */
    std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> frames_;

    /** \brief Base frame. {e_b}, the end frame of the base link, expressed
     *  in the world coordinate: {e_b}_w.
     *  to show the frame, the convention will be:
     *  <start/end>_<frame number>_<the frame this frame is expressed in>
     */
    Eigen::Affine3d base_;

};



}