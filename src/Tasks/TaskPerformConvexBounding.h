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

#include "TaskCreateMIPProblem.h"

namespace SHOT
{

class TaskPerformConvexBounding : public TaskBase
{
public:
    TaskPerformConvexBounding(EnvironmentPtr envPtr);
    ~TaskPerformConvexBounding() override;

    void run() override;
    std::string getType() override;

private:
    std::shared_ptr<TaskCreateMIPProblem> taskCreateMIPProblem;
    int lastNumberOfHyperplanesWithConvexSource = 0;
    int lastNumberOfHyperplanesWithNonconvexSource = 0;
    int idleIterations = 0;
};
} // namespace SHOT