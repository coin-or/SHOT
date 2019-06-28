/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskAddIntegerCuts.h"

#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"

namespace SHOT
{

TaskAddIntegerCuts::TaskAddIntegerCuts(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskAddIntegerCuts::~TaskAddIntegerCuts() = default;

void TaskAddIntegerCuts::run()
{
    env->timing->startTimer("DualStrategy");

    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    if(env->dualSolver->MIPSolver->integerCutWaitingList.size() == 0)
        return;

    if(!currIter->isMIP() || !env->settings->getSetting<bool>("HyperplaneCuts.Delay", "Dual")
        || !currIter->MIPSolutionLimitUpdated)
    {

        for(size_t j = 0; j < env->dualSolver->MIPSolver->integerCutWaitingList.size(); j++)
        {
            auto [ones, zeroes] = env->dualSolver->MIPSolver->integerCutWaitingList.at(j);

            env->dualSolver->MIPSolver->createIntegerCut(ones, zeroes);
            env->solutionStatistics.numberOfIntegerCuts++;
        }

        env->output->outputDebug("        Added "
            + std::to_string(env->dualSolver->MIPSolver->integerCutWaitingList.size())
            + " integer cut(s) to waiting list.");

        env->dualSolver->MIPSolver->integerCutWaitingList.clear();
    }

    env->timing->stopTimer("DualStrategy");
}

std::string TaskAddIntegerCuts::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT