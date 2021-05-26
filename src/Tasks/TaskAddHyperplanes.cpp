/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskAddHyperplanes.h"

#include "../Model/Constraints.h"
#include "../Model/Problem.h"

#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"

namespace SHOT
{

TaskAddHyperplanes::TaskAddHyperplanes(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("DualStrategy");
    itersWithoutAddedHPs = 0;

    env->timing->stopTimer("DualStrategy");
}

TaskAddHyperplanes::~TaskAddHyperplanes() = default;

void TaskAddHyperplanes::run()
{
    env->timing->startTimer("DualStrategy");

    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    if(!currIter->isMIP() || !env->settings->getSetting<bool>("HyperplaneCuts.Delay", "Dual")
        || !currIter->MIPSolutionLimitUpdated || itersWithoutAddedHPs > 5)
    {
        int addedHyperplanes = 0;

        for(auto k = env->dualSolver->hyperplaneWaitingList.size(); k > 0; k--)
        {
            if(addedHyperplanes >= env->settings->getSetting<int>("HyperplaneCuts.MaxPerIteration", "Dual"))
                break;

            auto tmpItem = env->dualSolver->hyperplaneWaitingList.at(k - 1);

            bool cutAddedSuccessfully = false;

            if(tmpItem.source == E_HyperplaneSource::PrimalSolutionSearchInteriorObjective)
            {
                cutAddedSuccessfully = env->dualSolver->MIPSolver->createInteriorHyperplane(tmpItem);
            }
            else
            {
                cutAddedSuccessfully = env->dualSolver->MIPSolver->createHyperplane(tmpItem);
            }

            if(cutAddedSuccessfully)
            {
                env->dualSolver->addGeneratedHyperplane(tmpItem);
                addedHyperplanes++;
                this->itersWithoutAddedHPs = 0;

                env->output->outputDebug(
                    fmt::format("        Cut added successfully for constraint {}.", tmpItem.sourceConstraintIndex));
            }
            else
            {
                env->output->outputDebug(fmt::format(
                    "        Cut not added successfully for constraint {}.", tmpItem.sourceConstraintIndex));
            }
        }

        if(!env->settings->getSetting<bool>("TreeStrategy.Multi.Reinitialize", "Dual"))
        {
            env->dualSolver->hyperplaneWaitingList.clear();
        }
    }
    else
    {
        this->itersWithoutAddedHPs++;
    }

    env->timing->stopTimer("DualStrategy");
}

std::string TaskAddHyperplanes::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT