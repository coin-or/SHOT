/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/
#include "TaskSelectHyperplanesExternal.h"

#include "../Callback.h"
#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Utilities.h"
#include "../Timing.h"

#include "../Model/Problem.h"

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
    if(!env->callbacks->isActive(E_CallbackLocation::HyperplaneSelection))
        return;

    env->timing->startTimer("CallbackExternalHyperplaneGeneration");

    env->output->outputDebug("        Selecting cutting planes using external callback functionality:");

    auto context = std::make_shared<HyperplaneSelectionContext>(env, std::move(solutionPoints));
    env->callbacks->invoke(*context);
    auto externalHyperplanes = context->getAddedHyperplanes();
    context->invalidate();

    if(!externalHyperplanes.empty())
    {
        env->output->outputDebug(fmt::format("        Received {} external hyperplanes from callback at iteration {}",
            externalHyperplanes.size(), env->results->getCurrentIteration()->iterationNumber));

        // Add each received hyperplane to the dual solver
        for(const auto& HP : externalHyperplanes)
        {
            env->dualSolver->addHyperplane(std::make_shared<ExternalHyperplane>(HP));
        }

        env->output->outputDebug(fmt::format(
            "        Successfully added {} external hyperplanes to dual solver", externalHyperplanes.size()));
    }

    env->timing->stopTimer("CallbackExternalHyperplaneGeneration");
}

std::string TaskSelectHyperplanesExternal::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT