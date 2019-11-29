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
class TaskSelectHyperplanePointsECP;

class TaskSelectHyperplanePointsESH : public TaskBase
{
public:
    TaskSelectHyperplanePointsESH(EnvironmentPtr envPtr);
    ~TaskSelectHyperplanePointsESH() override;

    void run() override;
    virtual void run(std::vector<SolutionPoint> solPoints);

    std::string getType() override;

private:
    std::unique_ptr<TaskSelectHyperplanePointsECP> tSelectHPPts;
    std::vector<Constraint*> nonlinearConstraints;
};
} // namespace SHOT