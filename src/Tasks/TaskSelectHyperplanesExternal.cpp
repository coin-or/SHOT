/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/
#include "TaskSelectHyperplanesExternal.h"

#include "../DualSolver.h"
#include "../EventHandler.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Utilities.h"
#include "../Timing.h"

#include "../Model/Problem.h"

#include <any>

namespace SHOT
{

TaskSelectHyperplanesExternal::TaskSelectHyperplanesExternal(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("CallbackExternalHyperplaneGeneration");
    env->timing->stopTimer("CallbackExternalHyperplaneGeneration");
}

TaskSelectHyperplanesExternal::~TaskSelectHyperplanesExternal() = default;

void TaskSelectHyperplanesExternal::run() { this->run(env->results->getPreviousIteration()->solutionPoints); }

void TaskSelectHyperplanesExternal::run(std::vector<SolutionPoint> solutionPoints)
{
    env->timing->startTimer("CallbackExternalHyperplaneGeneration");

    env->output->outputDebug("        Selecting cutting planes using external callback functionality:");

    env->events->notify(E_EventType::ExternalHyperplaneSelection, solutionPoints);

    env->timing->stopTimer("CallbackExternalHyperplaneGeneration");
}

std::string TaskSelectHyperplanesExternal::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT