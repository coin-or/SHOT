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
    virtual ~TaskSelectHyperplanePointsESH();

    virtual void run();
    virtual void run(std::vector<SolutionPoint> solPoints);

    virtual std::string getType();

private:
    std::unique_ptr<TaskSelectHyperplanePointsECP> tSelectHPPts;
    bool hyperplaneSolutionPointStrategyInitialized = false;
    std::vector<Constraint*> nonlinearConstraints;
};
} // namespace SHOT