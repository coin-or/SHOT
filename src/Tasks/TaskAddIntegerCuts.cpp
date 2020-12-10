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
#include "../Model/Problem.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include <algorithm> // std::equal

namespace SHOT
{

TaskAddIntegerCuts::TaskAddIntegerCuts(EnvironmentPtr envPtr) : TaskBase(envPtr) { }

TaskAddIntegerCuts::~TaskAddIntegerCuts() = default;

void TaskAddIntegerCuts::run()
{
    env->timing->startTimer("DualStrategy");

    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    if(env->dualSolver->integerCutWaitingList.size() == 0)
        return;

    if(env->results->solutionIsGlobal && env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex
        && env->results->getCurrentIteration()->numHyperplanesAdded > 0)
    {
        env->output->outputDebug("        Integer cut not added yet, since problem is nonconvex, solution still "
                                 "global, and other cuts have been added.");
        return;
    }

    if(!currIter->isMIP() || !env->settings->getSetting<bool>("HyperplaneCuts.Delay", "Dual")
        || !currIter->MIPSolutionLimitUpdated)
    {
        int numAdded = 0;

        for(auto& IC : env->dualSolver->integerCutWaitingList)
        {
            if(env->dualSolver->MIPSolver->createIntegerCut(IC))
                env->dualSolver->addGeneratedIntegerCut(IC);

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