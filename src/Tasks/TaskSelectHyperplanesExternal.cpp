/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/
#include "TaskSelectHyperplanesExternal.h"

#include "../CallbackData.h"
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
    // Only execute if there are external hyperplane data providers registered
    if(!env->events->hasDataProvider(E_EventType::ExternalHyperplaneSelection))
        return;

    env->timing->startTimer("CallbackExternalHyperplaneGeneration");

    env->output->outputDebug("        Selecting cutting planes using external callback functionality:");

    // Gather current state information for the callback
    bool isMinimization = (env->reformulatedProblem->objectiveFunction->properties.isMinimize);
    int iterationNumber = env->results->getCurrentIteration()->iterationNumber;
    double currentDualBound = env->results->getCurrentDualBound();
    double currentPrimalBound = env->results->getPrimalBound();
    double relativeGap = env->results->getRelativeCurrentObjectiveGap();
    double absoluteGap = env->results->getAbsoluteCurrentObjectiveGap();

    // Check if the objective function is nonlinear
    bool isObjectiveNonlinear = (env->reformulatedProblem->objectiveFunction->properties.classification
        >= E_ObjectiveFunctionClassification::Quadratic);

    // Create callback data with comprehensive context information
    ExternalHyperplaneSelectionCallbackData callbackData(isMinimization, iterationNumber, currentDualBound,
        currentPrimalBound, relativeGap, absoluteGap, solutionPoints, env->problem, env->reformulatedProblem,
        isObjectiveNonlinear, env->solutionStatistics);

    // Request external hyperplanes from registered data provider callbacks
    auto externalHyperplanes = env->events->requestData<std::vector<ExternalHyperplane>>(
        E_EventType::ExternalHyperplaneSelection, callbackData);

    if(externalHyperplanes.has_value() && !externalHyperplanes->empty())
    {
        env->output->outputDebug(fmt::format("        Received {} external hyperplanes from callback at iteration {}",
            externalHyperplanes->size(), iterationNumber));

        // Add each received hyperplane to the dual solver
        for(const auto& HP : *externalHyperplanes)
        {
            env->dualSolver->addHyperplane(std::make_shared<ExternalHyperplane>(HP));
        }

        env->output->outputDebug(fmt::format(
            "        Successfully added {} external hyperplanes to dual solver", externalHyperplanes->size()));
    }

    env->timing->stopTimer("CallbackExternalHyperplaneGeneration");
}

std::string TaskSelectHyperplanesExternal::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT