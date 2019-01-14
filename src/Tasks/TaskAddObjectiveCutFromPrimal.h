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

namespace SHOT
{
class TaskAddObjectiveCutFromPrimal : public TaskBase
{
public:
    TaskAddObjectiveCutFromPrimal(EnvironmentPtr envPtr, std::string taskIDTrue);
    virtual ~TaskAddObjectiveCutFromPrimal();

    virtual void run();

    virtual std::string getType();

private:
    std::string taskIDIfTrue;

    double previousCutOff = SHOT_DBL_MAX;
    int numCutOff = 0;
    int numWithoutPrimalUpdate = 0;
};
} // namespace SHOT