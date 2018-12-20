/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "TaskBase.h"

#include "../MIPSolver/IMIPSolver.h"

#include "../LinesearchMethod/ILinesearchMethod.h"

namespace SHOT
{
class TaskSolveFixedDualProblem : public TaskBase
{
  public:
    TaskSolveFixedDualProblem(EnvironmentPtr envPtr);
    virtual ~TaskSolveFixedDualProblem();
    virtual void run();
    virtual std::string getType();

  private:
    VectorInteger discreteVariableIndexes;
    std::vector<VectorDouble> testedPoints;

    VectorDouble lastSolution;
    double lastPrimalBound = NAN;

    int totalIters = 0;
};
} // namespace SHOT