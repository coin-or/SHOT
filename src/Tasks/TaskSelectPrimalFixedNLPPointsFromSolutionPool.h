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
class TaskSelectPrimalFixedNLPPointsFromSolutionPool : public TaskBase
{
public:
    // isFinalPolish marks the instance added to the finalize sequence, which polishes the solution once the
    // search is over rather than acting as a primal heuristic during it.
    TaskSelectPrimalFixedNLPPointsFromSolutionPool(EnvironmentPtr envPtr, bool isFinalPolish = false);
    ~TaskSelectPrimalFixedNLPPointsFromSolutionPool() override;

    void run() override;
    std::string getType() override;

private:
    bool isFinalPolish;
};
} // namespace SHOT