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

#include "../MIPSolver/IMIPSolver.h"

class TaskSolveFixedLinearProblem : public TaskBase
{
  public:
    TaskSolveFixedLinearProblem(IMIPSolver *MIPSolver);
    virtual ~TaskSolveFixedLinearProblem();
    virtual void run();
    virtual std::string getType();

  private:
    std::vector<int> discreteVariableIndexes;
    std::vector<std::vector<double>> testedPoints;

    std::vector<double> lastSolution;
    double lastPrimalBound = NAN;
    IMIPSolver *MIPSolver;

    int totalIters = 0;
};
