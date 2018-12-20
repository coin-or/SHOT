/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include "TaskSelectHyperplanePointsECP.h"
#include "../LinesearchMethod/ILinesearchMethod.h"

namespace SHOT
{
class TaskSelectHyperplanePointsESH : public TaskBase
{
  public:
    TaskSelectHyperplanePointsESH(EnvironmentPtr envPtr);
    virtual ~TaskSelectHyperplanePointsESH();

    virtual void run();
    virtual void run(std::vector<SolutionPoint> solPoints);

    virtual std::string getType();

  private:
    TaskSelectHyperplanePointsECP *tSelectHPPts;
    bool hyperplaneSolutionPointStrategyInitialized = false;
    std::vector<Constraint *> nonlinearConstraints;
};
} // namespace SHOT