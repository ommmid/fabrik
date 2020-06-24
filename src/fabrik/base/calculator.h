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

FABRIK_CLASS_FORWARD(Calculator);

/** \brief This class includes the problem input, output and the solver
 */
class Calculator
{
public:
    /** \brief Construct a Calculator object */
    // Calculator();
    
    /** \brief Calculate the error between two affine3d frames */
    virtual double calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2) = 0;

    /** \brief Calculates the transformation of the reaching frame based on some criteria  */
    virtual double calculateReach(const Eigen::Affine3d& start_frame_reaching, 
                                const Eigen::Affine3d& end_frame_reaching, 
                                const Eigen::Affine3d& frame_aimed_at) = 0;

protected:

    double error_;


};

class PositionBasedCalculator : public Calculator
{
public:
    PositionBasedCalculator();

    double calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2) override;

    /** \brief Finds the angle to go FROM start_to_end_projected TO start_to_aim_projected */
    double calculateReach(const Eigen::Affine3d& start_frame_reaching,
                          const Eigen::Affine3d& end_frame_reaching,
                          const Eigen::Affine3d& frame_aimed_at) override;

private:

};

class OrientationBasedCalculator : public Calculator
{
public:
    OrientationBasedCalculator();

    double calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2) override;

    double calculateReach(const Eigen::Affine3d& start_frame_reaching,
                          const Eigen::Affine3d& end_frame_reaching,
                          const Eigen::Affine3d& frame_aimed_at) override;
};

class ComboCalculator : public Calculator
{
public:
    ComboCalculator();

    double calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2) override;

    double calculateReach(const Eigen::Affine3d& start_frame_reaching,
                          const Eigen::Affine3d& end_frame_reaching,
                          const Eigen::Affine3d& frame_aimed_at) override;
};

}