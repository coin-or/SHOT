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

#include <algorithm>

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

    auto currIter = env->results->getCurrentIteration();

    if(!currIter->isMIP() || !env->settings->getSetting<bool>("Dual.HyperplaneCuts.Delay")
        || !currIter->MIPSolutionLimitUpdated || itersWithoutAddedHPs > 5)
    {
        int addedHyperplanes = 0;

        // A reinitialized dual problem gets all the hyperplanes of the waiting list again, which have all been
        // generated already
        bool isReinitialized = env->settings->getSetting<bool>("Dual.TreeStrategy.Multi.Reinitialize");

        int maxHyperplanesPerIter = env->settings->getSetting<int>("Dual.HyperplaneCuts.MaxPerIteration");

        // Only the first Dual.HyperplaneCuts.MaxPerIteration hyperplanes are added, and the rest are discarded. The
        // objective and external hyperplanes go first, and the constraint hyperplanes keep the order they were
        // selected in, i.e., those for the best solution points and the most deviating constraints come first. The
        // supporting hyperplanes in feasible solutions of the fixed-integer NLP problem go last, since most of them do
        // not cut away the dual solution, and with many active constraints they would otherwise replace those that do.
        auto& waitingList = env->dualSolver->hyperplaneWaitingList;
        std::stable_partition(waitingList.begin(), waitingList.end(),
            [](const HyperplanePtr& HP) { return (!std::dynamic_pointer_cast<ConstraintHyperplane>(HP)); });
        std::stable_partition(waitingList.begin(), waitingList.end(),
            [](const HyperplanePtr& HP) { return (HP->source != E_HyperplaneSource::FixedIntegerNLPSolution); });

        for(auto& HP : waitingList)
        {
            if(addedHyperplanes >= maxHyperplanesPerIter)
                break;

            // While the hyperplanes are delayed the dual problem is not changed, so it returns the same solution
            // points, and the same hyperplanes are put on the waiting list once per delayed iteration. Only the
            // first one is added, since the others are identical, and adding them would count them as hyperplanes
            // repeated because the dual problem is not making progress.
            if(!isReinitialized && env->dualSolver->hasHyperplaneBeenAdded(HP))
            {
                env->output->outputTrace("        Hyperplane on the waiting list has been added already.");
                continue;
            }

            bool cutAddedSuccessfully = env->dualSolver->MIPSolver->createHyperplane(HP);

            if(cutAddedSuccessfully)
            {
                env->dualSolver->addGeneratedHyperplane(HP);
                addedHyperplanes++;
                this->itersWithoutAddedHPs = 0;
            }

            if(auto constraintHyperplane = std::dynamic_pointer_cast<ConstraintHyperplane>(HP))
            {
                if(cutAddedSuccessfully)
                    env->output->outputDebug(fmt::format(
                        "        Cut added for constraint {}.", constraintHyperplane->sourceConstraint->name));
                else
                    env->output->outputDebug(fmt::format(
                        "        Cut not added for constraint {}.", constraintHyperplane->sourceConstraint->name));
            }
            else if(auto objectiveHyperplane = std::dynamic_pointer_cast<ObjectiveHyperplane>(HP))
            {
                if(cutAddedSuccessfully)
                    env->output->outputDebug(fmt::format("        Cut added for objective constraint."));
                else
                    env->output->outputDebug(fmt::format("        Cut not added for objective constraint."));
            }
            else if(auto externalHyperplane = std::dynamic_pointer_cast<ExternalHyperplane>(HP))
            {
                if(cutAddedSuccessfully)
                    env->output->outputInfo(
                        fmt::format("        Cut added for external constraint {}.", externalHyperplane->description));
                else
                    env->output->outputInfo(fmt::format(
                        "        Cut not added for external constraint {}.", externalHyperplane->description));
            }
        }

        if(!env->settings->getSetting<bool>("Dual.TreeStrategy.Multi.Reinitialize"))
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