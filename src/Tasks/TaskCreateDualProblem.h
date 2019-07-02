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
#include "../Model/Problem.h"

namespace SHOT
{

class TaskCreateDualProblem : public TaskBase
{
public:
    TaskCreateDualProblem(EnvironmentPtr envPtr);
    ~TaskCreateDualProblem() override;

    void run() override;
    std::string getType() override;

private:
    bool createProblem(MIPSolverPtr destinationProblem, ProblemPtr sourceProblem);
};
} // namespace SHOT