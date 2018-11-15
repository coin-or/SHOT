/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskAddHyperplanes.h"

namespace SHOT
{

TaskAddHyperplanes::TaskAddHyperplanes(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("DualStrategy");
    itersWithoutAddedHPs = 0;

    env->process->stopTimer("DualStrategy");
}

TaskAddHyperplanes::~TaskAddHyperplanes()
{
}

void TaskAddHyperplanes::run()
{
    env->process->startTimer("DualStrategy");

    auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration

    if (!currIter->isMIP() || !env->settings->getBoolSetting("HyperplaneCuts.Delay", "Dual") || !currIter->MIPSolutionLimitUpdated || itersWithoutAddedHPs > 5)
    {
        int addedHyperplanes = 0;

        for (int k = env->process->hyperplaneWaitingList.size(); k > 0; k--)
        {
            if (addedHyperplanes >= env->settings->getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual"))
                break;

            auto tmpItem = env->process->hyperplaneWaitingList.at(k - 1);

            if (tmpItem.source == E_HyperplaneSource::PrimalSolutionSearchInteriorObjective)
            {
                env->dualSolver->createInteriorHyperplane(tmpItem);
            }
            else
            {
                env->dualSolver->createHyperplane(tmpItem);

                env->process->addedHyperplanes.push_back(tmpItem);

                addedHyperplanes++;
            }
        }

        if (!env->settings->getBoolSetting("TreeStrategy.Multi.Reinitialize", "Dual"))
        {
            env->process->hyperplaneWaitingList.clear();
        }

        itersWithoutAddedHPs = 0;
    }
    else
    {
        itersWithoutAddedHPs++;
    }

    env->process->stopTimer("DualStrategy");
}

std::string TaskAddHyperplanes::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT