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
    Calculator();
    
    /** \brief Calculate the error between two affine3d frames */
    virtual double calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2) = 0;

    /** \brief Calculates the transformation of the reaching frame based on some criteria  */
    virtual bool calculateReach(const Eigen::Affine3d& start_frame_reaching, 
                                const Eigen::Affine3d& end_frame_reaching, 
                                const Eigen::Affine3d& frame_aimed_at) = 0;

protected:

    double error_;


}

class PositionBasedCalculator : Calculator
{
public:
    PositionBasedCalculator();

    double calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2) override;

    double calculateReach(const Eigen::Affine3d& start_frame_reaching,
                          const Eigen::Affine3d& end_frame_reaching,
                          const Eigen::Affine3d& frame_aimed_at) override;

private:

}

// class OrientationBasedCalculator : Calculator
// {
// public:
//     OrientationBasedCalculator();

//     calculateError() override;

//     calculateReach() override;
// }

// class ComboCalculator : Calculator
// {
//     ComboCalculator();

//     calculateError() override;

//     calculateReach() override;
// }

}