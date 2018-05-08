/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskAddIntegerCuts.h"

TaskAddIntegerCuts::TaskAddIntegerCuts(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
}

TaskAddIntegerCuts::~TaskAddIntegerCuts()
{
}

void TaskAddIntegerCuts::run()
{
    env->process->startTimer("DualStrategy");

    auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration

    if (env->process->integerCutWaitingList.size() == 0)
        return;

    if (!currIter->isMIP() || !env->settings->getBoolSetting("HyperplaneCuts.Delay", "Dual") || !currIter->MIPSolutionLimitUpdated)
    {

        for (int j = 0; j < env->process->integerCutWaitingList.size(); j++)
        {
            auto tmpBinaryCombination = env->process->integerCutWaitingList.at(j);
            int numOnes = tmpBinaryCombination.size();

            std::vector<IndexValuePair> elements;

            for (int i = 0; i < numOnes; i++)
            {
                IndexValuePair pair;
                pair.idx = tmpBinaryCombination.at(i);
                pair.value = 1.0;

                elements.push_back(pair);
            }

            env->dualSolver->addLinearConstraint(elements, -(numOnes - 1.0));
            env->process->solutionStatistics.numberOfIntegerCuts++;
        }

        env->output->outputInfo(
            "     Added " + std::to_string(env->process->integerCutWaitingList.size()) + " integer cut(s).                                        ");

        env->process->integerCutWaitingList.clear();
    }

    env->process->stopTimer("DualStrategy");
}

std::string TaskAddIntegerCuts::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
