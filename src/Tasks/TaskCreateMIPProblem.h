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

class TaskCreateMIPProblem : public TaskBase
{
public:
    TaskCreateMIPProblem(EnvironmentPtr envPtr, MIPSolverPtr MIPSolver, ProblemPtr sourceProblem);
    ~TaskCreateMIPProblem() override;

    void run() override;
    std::string getType() override;

private:
    bool createProblem(MIPSolverPtr destinationProblem, ProblemPtr sourceProblem);

    MIPSolverPtr MIPSolver;
    ProblemPtr sourceProblem;
};
} // namespace SHOT