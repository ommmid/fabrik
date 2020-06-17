/* Author: Omid Heidari */
#pragma once

#include <memory>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "fabrik/robot_model/robot_model.h"
#include "fabrik/util/class_forward.h"

namespace robot_state
{
    
enum ReachingDirection
{
    FORWARD,
    FORWARD_COMPLETED,
    BACKWARD,
    BACKWARD_COMPLETED
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
    /** \brief Construct a state at a forward-completed home configuratn means:
     * reaching_at_ = dof
     * reachin_direction_ = FORWARD
     * joint_values = set to zeros
     */
    RobotState(const std::vector<robot_model::Link>& chain, const Eigen::Affine3d& base);

    /** \brief Not sure if I need to create any other constructor
     */
    RobotState(const std::vector<robot_model::Link>& chain,
               const Eigen::Affine3d& base,
               const ReachingDirection& reaching_direction);


    /** \brief This function updates only [s_i e_i]. We do not change the rest of the 
     * frames start_{i+1} and end_{i+1} ... because this function is used in
     * either forward or backward reaching, meaning it does not
     * calculate a full forward kinematics to reach to the end-effector. It is being disassembled
     * Joint number and linke number are the same. J1 is on s1. s1 and e1 are both on link 1
     * and they start from 0 to n-1, n being the number of degrees of freedom
     * It also updates the joints values first
     */ 
    void updateFrame(const double& joint_value, const int& joint_number);

    int getReachingAt() const
    {
        return reaching_at_;
    }

    int getDOF() const
    {
        return dof_;
    }

    ReachingDirection getReachingDirection() const
    {
        return reaching_direction_;
    }

    void setReachingDirection(ReachingDirection& reaching_direction)
    {
        reaching_direction_ = reaching_direction;
    }

    const std::vector<double> getJointsValues() const
    {
        return joints_values_;
    }

    const std::vector<robot_model::Link> getChain() const
    {
        return chain_;
    }

    const std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> getFrames() const
    {
        return frames_;
    } 

    const Eigen::Affine3d getBase() const
    {
        return base_;
    } 

private:
    /** \brief An integer showing the link number trying to reach.
     * If this number is at dof, then it means the robot is at forward complete configuration
     */
    int reaching_at_;

    /** \brief Reaching direction: forward or backward  */
    ReachingDirection reaching_direction_;

    /** \brief Degrees of freedom  */
    int dof_;

    /** \brief The value of each joint for the current state. Joint number start from 0. */
    std::vector<double> joints_values_;

    /** \brief Chain. The shared pointer is pointing to a const robot_model::Chain
     * because we wont change this chain (it is the structure of the robot)
     */
    // std::shared_ptr<const robot_model::Chain> chain_;
    std::vector<robot_model::Link> chain_;
    
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
    // do I need the target in the state too ???
};


}