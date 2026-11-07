/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"
#include "../MIPSolver/MIPSolverCapabilities.h"

namespace SHOT
{

class TaskCreateMIPProblem : public TaskBase
{
public:
    TaskCreateMIPProblem(
        EnvironmentPtr envPtr, MIPSolverPtr MIPSolver, ProblemPtr sourceProblem, MIPSolverCapabilities capabilities);
    ~TaskCreateMIPProblem() override;

    void run() override;
    std::string getType() override;

private:
    bool createProblem(MIPSolverPtr destinationProblem, ProblemPtr sourceProblem);

    MIPSolverCapabilities capabilities;
    MIPSolverPtr MIPSolver;
    ProblemPtr sourceProblem;
};
} // namespace SHOT