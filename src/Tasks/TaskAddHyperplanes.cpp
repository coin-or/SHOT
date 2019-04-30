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

        for(int k = env->dualSolver->MIPSolver->hyperplaneWaitingList.size(); k > 0; k--)
        {
            if(addedHyperplanes >= env->settings->getSetting<int>("HyperplaneCuts.MaxPerIteration", "Dual"))
                break;

            auto tmpItem = env->dualSolver->MIPSolver->hyperplaneWaitingList.at(k - 1);

            if(tmpItem.source == E_HyperplaneSource::PrimalSolutionSearchInteriorObjective)
            {
                env->dualSolver->MIPSolver->createInteriorHyperplane(tmpItem);
            }
            else
            {
                env->dualSolver->MIPSolver->createHyperplane(tmpItem);

                env->dualSolver->MIPSolver->addedHyperplanes.push_back(tmpItem);

                if(tmpItem.isObjectiveHyperplane)
                {
                    bool isObjectiveConvex
                        = env->reformulatedProblem->objectiveFunction->properties.convexity == E_Convexity::Convex
                        || env->reformulatedProblem->objectiveFunction->properties.convexity == E_Convexity::Linear;

                    if(!isObjectiveConvex)
                    {
                        env->results->solutionIsGlobal = false;
                        env->output->outputDebug("Solution is no longer global");
                    }
                }
                else
                {
                    bool isConstraintConvex = tmpItem.sourceConstraint->properties.convexity == E_Convexity::Convex
                        || tmpItem.sourceConstraint->properties.convexity == E_Convexity::Linear;

                    if(!isConstraintConvex)
                    {
                        env->results->solutionIsGlobal = false;
                        env->output->outputDebug("Solution is no longer global");
                    }
                }

                addedHyperplanes++;
            }
        }

        if(!env->settings->getSetting<bool>("TreeStrategy.Multi.Reinitialize", "Dual"))
        {
            env->dualSolver->MIPSolver->hyperplaneWaitingList.clear();
        }

        itersWithoutAddedHPs = 0;
    }
    else
    {
        itersWithoutAddedHPs++;
    }

    env->timing->stopTimer("DualStrategy");
}

std::string TaskAddHyperplanes::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT