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
    TaskAddPrimalReductionCut(EnvironmentPtr envPtr, std::string taskIDTrue, std::string taskIDFalse);
    ~TaskAddPrimalReductionCut() override;

    void run() override;

    std::string getType() override;

private:
    std::string taskIDIfTrue;
    std::string taskIDIfFalse;
    int totalReductionCutUpdates = 0;
    double currentLowerBoundForReductionCut = SHOT_DBL_INF;
};
} // namespace SHOT