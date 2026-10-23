/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromExternalSource.h"

#include "../Callback.h"
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
    env->timing->startTimer("PrimalStrategy");

    if(env->callbacks->isActive(E_CallbackLocation::PrimalCandidateSearch))
    {
        auto context = std::make_shared<PrimalCandidateSearchContext>(env);
        env->callbacks->invoke(*context);
        auto externalPrimalSolutions = context->getAddedPrimalSolutions();
        context->invalidate();

        if(!externalPrimalSolutions.empty())
        {
            env->primalSolver->addPrimalSolutionCandidates(externalPrimalSolutions,
                E_PrimalSolutionSource::ExternalPrimalSolution, env->results->getNumberOfIterations());
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