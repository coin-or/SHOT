/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"
#include "../OptProblems/OptProblemOriginal.h"
#include "TaskSelectHyperplanePointsSolution.h"

class TaskSelectHyperplanePointsIndividualLinesearch : public TaskBase
{
  public:
    TaskSelectHyperplanePointsIndividualLinesearch(EnvironmentPtr envPtr);
    virtual ~TaskSelectHyperplanePointsIndividualLinesearch();

    virtual void run();
    virtual void run(std::vector<SolutionPoint> solPoints);

    virtual std::string getType();

  private:
    std::vector<int> nonlinearConstraintIdxs;

    TaskSelectHyperplanePointsSolution *tSelectHPPts;
    bool hyperplaneSolutionPointStrategyInitialized = false;
};
