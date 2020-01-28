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
#include "../Utilities.h"

#include <algorithm> // std::equal

namespace SHOT
{

TaskAddIntegerCuts::TaskAddIntegerCuts(EnvironmentPtr envPtr) : TaskBase(envPtr) {}

TaskAddIntegerCuts::~TaskAddIntegerCuts() = default;

void TaskAddIntegerCuts::run()
{
    env->timing->startTimer("DualStrategy");

    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    if(env->dualSolver->integerCutWaitingList.size() == 0)
        return;

    if(!currIter->isMIP() || !env->settings->getSetting<bool>("HyperplaneCuts.Delay", "Dual")
        || !currIter->MIPSolutionLimitUpdated)
    {
        int numAdded = 0;

        for(size_t j = 0; j < env->dualSolver->integerCutWaitingList.size(); j++)
        {
            auto [ones, zeroes] = env->dualSolver->integerCutWaitingList.at(j);

            bool alreadyAdded = false;

            for(auto [oldOnes, oldZeroes] : env->dualSolver->generatedIntegerCuts)
            {
                bool cutsAreEqual = true;

                if(oldOnes.size() != ones.size())
                    continue;

                if(oldZeroes.size() != zeroes.size())
                    continue;

                for(size_t i = 0; i < oldZeroes.size(); i++)
                {
                    if(zeroes[i] != oldZeroes[i])
                    {
                        cutsAreEqual = false;
                        break;
                    }
                }

                if(!cutsAreEqual)
                    continue;

                for(size_t i = 0; i < oldOnes.size(); i++)
                {
                    if(ones[i] != oldOnes[i])
                    {
                        cutsAreEqual = false;
                        break;
                    }
                }

                if(cutsAreEqual)
                {
                    alreadyAdded = true;
                    break;
                }
            }

            if(alreadyAdded)
                continue;

            env->dualSolver->MIPSolver->createIntegerCut(ones, zeroes);
            env->dualSolver->generatedIntegerCuts.emplace_back(ones, zeroes);
            numAdded++;
        }

        env->output->outputDebug("        Added " + std::to_string(numAdded) + " integer cuts.");

        env->dualSolver->integerCutWaitingList.clear();
    }

    env->timing->stopTimer("DualStrategy");
}

std::string TaskAddIntegerCuts::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT