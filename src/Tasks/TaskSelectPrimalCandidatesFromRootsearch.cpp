/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromRootsearch.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Results.h"
#include "../PrimalSolver.h"
#include "../Settings.h"
#include "../Timing.h"

#include "../Model/Problem.h"

#include "../RootsearchMethod/IRootsearchMethod.h"

namespace SHOT
{

TaskSelectPrimalCandidatesFromRootsearch::TaskSelectPrimalCandidatesFromRootsearch(EnvironmentPtr envPtr)
    : TaskBase(envPtr)
{
}

TaskSelectPrimalCandidatesFromRootsearch::~TaskSelectPrimalCandidatesFromRootsearch() = default;

void TaskSelectPrimalCandidatesFromRootsearch::run() { this->run(env->results->getCurrentIteration()->solutionPoints); }

std::string TaskSelectPrimalCandidatesFromRootsearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

void TaskSelectPrimalCandidatesFromRootsearch::run(std::vector<SolutionPoint> solPoints)
{
    auto currIter = env->results->getCurrentIteration();

    if((currIter->isMIP() && env->results->getRelativeGlobalObjectiveGap() > 1e-10)
        || env->results->usedSolutionStrategy == E_SolutionStrategy::NLP)
    {
        env->timing->startTimer("PrimalStrategy");
        env->timing->startTimer("PrimalBoundStrategyRootSearch");

        for(auto& P : solPoints)
        {
            for(auto& IP : env->dualSolver->interiorPts)
            {
                auto xNLP = IP->point;

                auto solPoint = P.point;

                assert(xNLP.size() == solPoint.size());

                for(auto& V : env->reformulatedProblem->binaryVariables)
                {
                    xNLP.at(V->index) = P.point.at(V->index);
                }

                for(auto& V : env->reformulatedProblem->integerVariables)
                {
                    xNLP.at(V->index) = P.point.at(V->index);
                }

                for(auto& V : env->reformulatedProblem->semiintegerVariables)
                {
                    xNLP.at(V->index) = P.point.at(V->index);
                }

                auto maxDevNLP2 = env->reformulatedProblem->getMaxNumericConstraintValue(
                    xNLP, env->reformulatedProblem->numericConstraints);
                auto maxDevMIP = env->reformulatedProblem->getMaxNumericConstraintValue(
                    P.point, env->reformulatedProblem->numericConstraints);

                if(maxDevNLP2.normalizedValue < 0 && maxDevMIP.normalizedValue > 0)
                {
                    std::pair<VectorDouble, VectorDouble> xNewc;

                    try
                    {
                        env->timing->startTimer("PrimalBoundStrategyRootSearch");
                        xNewc = env->rootsearchMethod->findZero(xNLP, P.point,
                            env->settings->getSetting<int>("Rootsearch.MaxIterations", "Subsolver"),
                            env->settings->getSetting<double>("Rootsearch.TerminationTolerance", "Subsolver"), 0,
                            env->reformulatedProblem->nonlinearConstraints, false);

                        env->timing->stopTimer("PrimalBoundStrategyRootSearch");

                        env->primalSolver->addPrimalSolutionCandidate(xNewc.first, E_PrimalSolutionSource::Rootsearch,
                            env->results->getCurrentIteration()->iterationNumber);
                    }
                    catch(std::exception&)
                    {
                        env->output->outputDebug("        Cannot find solution with primal rootsearch.");
                    }
                }
            }

            env->timing->stopTimer("PrimalStrategy");
            env->timing->stopTimer("PrimalBoundStrategyRootSearch");
        }
    }
}
} // namespace SHOT