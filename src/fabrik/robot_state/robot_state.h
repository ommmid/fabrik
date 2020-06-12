/* Author: Omid Heidari */
#pragma once

#include <memory>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "robot_model.h"
#include "fabrik/util/class_forward.h"

namespace robot_state
{
    
enum ReachingDirection
{
    FORWARD = 0,
    BACKWARD = 1
};


// I want forward decleration for RobotState becuase I predict it will be a heavy class
FABRIK_CLASS_FORWARD(RobotState);

/** \brief This class determines that what state our chain is at. Considering FABRIK
 * algorithm, the state of the robot are the things that we want to update
 */
class RobotState
{
public:
    /** \brief Construct a state at a forward-completed home configuratn means:
     * reaching_at_ = dof + 1
     * reachin_direction_ = FORWARD
     * joint_values = set to zeros
     */
    RobotState(const robot_model::Chain& chain);


    void setJointsValues(const std::vector<double>& joints_values);

    void getJointsValues(std::vector<double>& joints_values) const;

private:
    /** \brief An integer showing the link number trying to reach  */
    int reaching_at_;

    /** \brief Reaching direction: forward or backward  */
    ReachingDirection reaching_direction_;

    /** \brief The value of each joint for the current state */
    std::vector<double> joints_values_;

    /** \brief Chain. The shared pointer is pointing to a const robot_model::Chain
     * because we wont change this chain (it is the structure of the robot)
     */
    std::shared_ptr<const robot_model::Chain> chain_;
    
    /** \brief All frames of all links in order, expressed in world frame:
     * [s_1 e_1 s_2 e_2 .... s_n e_n]. Because I want to keep the indecis easily 
     * to read. I will make pair for each start and end:
     * [[s_1 e_1] [s_2 e_2] .... [s_n e_n]]
     */
    std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> frames_;

    /** Base frame. {e0}, the end frame of the base link, expressed
     *  in the world coordinate: {e0}_w.
     *  to show the frame, the convention will be:
     *  <start/end>_<frame number>_<the frame this frame is expressed in>
     */
    Eigen::Affine3d base_;
    // do I need the target in the sate too ???

    /** \brief Update frames_ with new joint values for specific frames. The point is 
     * that this update is not necessary like general forward kinematics because the links
     * can be disassembeld in FABRIK concept. So we do not have to have the values for
     * all joints in the chain to update frames.
     */
    void updateFrames(std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>>& frames,
        , const std::vector<double>& new_joint_value);

};


}