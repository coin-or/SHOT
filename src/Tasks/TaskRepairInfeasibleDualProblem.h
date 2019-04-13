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
class TaskRepairInfeasibleDualProblem : public TaskBase
{
public:
    TaskRepairInfeasibleDualProblem(EnvironmentPtr envPtr, std::string taskIDTrue, std::string taskIDFalse);
    virtual ~TaskRepairInfeasibleDualProblem();

    virtual void run();
    virtual std::string getType();

private:
    std::string taskIDIfTrue;
    std::string taskIDIfFalse;
    int iterLastRepair = 0;
    int mainRepairTries = 0;
};
} // namespace SHOT