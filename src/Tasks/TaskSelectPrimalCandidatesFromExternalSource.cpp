/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromExternalSource.h"

#include "../CallbackData.h"
#include "../EventHandler.h"
#include "../Iteration.h"
#include "../Model/Problem.h"
#include "../Results.h"
#include "../PrimalSolver.h"
#include "../Settings.h"
#include "../Timing.h"

namespace SHOT
{

TaskSelectPrimalCandidatesFromExternalSource::TaskSelectPrimalCandidatesFromExternalSource(EnvironmentPtr envPtr)
    : TaskBase(envPtr)
{
}

TaskSelectPrimalCandidatesFromExternalSource::~TaskSelectPrimalCandidatesFromExternalSource() = default;

void TaskSelectPrimalCandidatesFromExternalSource::run()
{
    auto currIter = env->results->getCurrentIteration();

    env->timing->startTimer("PrimalStrategy");

    // Check if there are data providers for external primal solutions
    if(env->events->hasDataProvider(E_EventType::ExternalPrimalSolution))
    {
        // Create callback data with current optimization state
        bool isMinimization = (env->reformulatedProblem->objectiveFunction->properties.isMinimize);
        double currentDualBound = env->results->getCurrentDualBound();
        double currentPrimalBound = env->results->getPrimalBound();
        double relativeGap = env->results->getRelativeCurrentObjectiveGap();
        double absoluteGap = env->results->getAbsoluteCurrentObjectiveGap();
        int iterationNumber = env->results->getNumberOfIterations();

        // Get current best solution if available
        VectorDouble currentSolution;
        if(env->results->hasPrimalSolution())
        {
            currentSolution = env->results->primalSolution;
        }

        ExternalPrimalSolutionCallbackData callbackData(isMinimization, currentDualBound, currentPrimalBound,
            relativeGap, absoluteGap, iterationNumber, currentSolution, env->solutionStatistics);

        auto externalPrimalSolution
            = env->events->requestData<std::vector<VectorDouble>>(E_EventType::ExternalPrimalSolution, callbackData);

        if(externalPrimalSolution.has_value())
        {
            env->primalSolver->addPrimalSolutionCandidates(
                *externalPrimalSolution, E_PrimalSolutionSource::ExternalPrimalSolution, iterationNumber);
        }
    }

    env->timing->stopTimer("PrimalStrategy");
}

std::string TaskSelectPrimalCandidatesFromExternalSource::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT