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
class TaskAddPrimalReductionCut : public TaskBase
{
public:
    TaskAddPrimalReductionCut(EnvironmentPtr envPtr, std::string taskIDTrue, std::string taskIDFalse,
        bool isFinalAttempt = false, bool stopOnlyAtLimits = false);
    ~TaskAddPrimalReductionCut() override;

    void run() override;

    std::string getType() override;

private:
    std::string taskIDIfTrue;
    std::string taskIDIfFalse;

    bool isFinalAttempt;

    // Whether the final attempt runs whatever the search stopped for, except a time, iteration or gap limit or the user
    bool stopOnlyAtLimits;
    int totalReductionCutUpdates = 0;
    double currentLowerBoundForReductionCut = SHOT_DBL_INF;
};
} // namespace SHOT