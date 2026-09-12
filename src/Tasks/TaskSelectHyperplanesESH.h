/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

namespace SHOT
{

class Constraint;
class NumericConstraint;
class TaskSelectHyperplanesECP;

class TaskSelectHyperplanesESH : public TaskBase
{
public:
    TaskSelectHyperplanesESH(EnvironmentPtr envPtr);
    ~TaskSelectHyperplanesESH() override;

    void run() override;
    virtual void run(std::vector<SolutionPoint> solPoints);

    std::string getType() override;

private:
    std::unique_ptr<TaskSelectHyperplanesECP> tSelectHPPts;
    std::vector<Constraint*> nonlinearConstraints;

    // The point the hyperplane for the constraint is generated in, which is the point from the root search unless
    // the hyperplane there does not cut off the solution point
    VectorDouble selectHyperplanePoint(
        const VectorDouble& externalPoint, const VectorDouble& solutionPoint, NumericConstraint* constraint);
};
} // namespace SHOT