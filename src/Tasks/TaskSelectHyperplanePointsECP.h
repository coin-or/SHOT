/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include "../OptProblems/OptProblemOriginal.h"

namespace SHOT
{
class TaskSelectHyperplanePointsECP : public TaskBase
{
  public:
    TaskSelectHyperplanePointsECP(EnvironmentPtr envPtr);
    virtual ~TaskSelectHyperplanePointsECP();

    virtual void run();
    virtual void run(std::vector<SolutionPoint> solPoints);

    virtual std::string getType();

  private:
};
} // namespace SHOT