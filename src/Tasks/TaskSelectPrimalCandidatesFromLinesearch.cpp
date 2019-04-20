/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromLinesearch.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Results.h"
#include "../PrimalSolver.h"
#include "../Settings.h"
#include "../Timing.h"

#include "../Model/Problem.h"

#include "../LinesearchMethod/ILinesearchMethod.h"

namespace SHOT
{

TaskSelectPrimalCandidatesFromLinesearch::TaskSelectPrimalCandidatesFromLinesearch(EnvironmentPtr envPtr)
    : TaskBase(envPtr)
{
}

TaskSelectPrimalCandidatesFromLinesearch::~TaskSelectPrimalCandidatesFromLinesearch() {}

void TaskSelectPrimalCandidatesFromLinesearch::run() { this->run(env->results->getCurrentIteration()->solutionPoints); }

std::string TaskSelectPrimalCandidatesFromLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

void TaskSelectPrimalCandidatesFromLinesearch::run(std::vector<SolutionPoint> solPoints)
{
    auto currIter = env->results->getCurrentIteration();

    if(currIter->isMIP() && env->results->getRelativeGlobalObjectiveGap() > 1e-10
        || env->results->usedSolutionStrategy == E_SolutionStrategy::NLP)
    {
        env->timing->startTimer("PrimalStrategy");
        env->timing->startTimer("PrimalBoundStrategyRootSearch");

        for(int i = 0; i < solPoints.size(); i++)
        {
            for(int j = 0; j < env->dualSolver->MIPSolver->interiorPts.size(); j++)
            {
                auto xNLP = env->dualSolver->MIPSolver->interiorPts.at(j)->point;

                auto solPoint = solPoints.at(i).point;

                for(auto& V : env->problem->binaryVariables)
                {
                    xNLP.at(V->index) = solPoints.at(i).point.at(V->index);
                }

                for(auto& V : env->problem->integerVariables)
                {
                    xNLP.at(V->index) = solPoints.at(i).point.at(V->index);
                }

                auto maxDevNLP2 = env->problem->getMaxNumericConstraintValue(xNLP, env->problem->numericConstraints);
                auto maxDevMIP = env->problem->getMaxNumericConstraintValue(
                    solPoints.at(i).point, env->problem->numericConstraints);

                if(maxDevNLP2.normalizedValue <= 0 && maxDevMIP.normalizedValue > 0)
                {
                    std::pair<VectorDouble, VectorDouble> xNewc;

                    try
                    {
                        env->timing->startTimer("PrimalBoundStrategyRootSearch");
                        xNewc = env->rootsearchMethod->findZero(xNLP, solPoints.at(i).point,
                            env->settings->getSetting<int>("Rootsearch.MaxIterations", "Subsolver"),
                            env->settings->getSetting<double>("Rootsearch.TerminationTolerance", "Subsolver"), 0,
                            env->reformulatedProblem->nonlinearConstraints, false);

                        env->timing->stopTimer("PrimalBoundStrategyRootSearch");

                        env->primalSolver->addPrimalSolutionCandidate(xNewc.first, E_PrimalSolutionSource::Linesearch,
                            env->results->getCurrentIteration()->iterationNumber);
                    }
                    catch(std::exception& e)
                    {
                        env->output->outputWarning("Cannot find solution with primal bound linesearch.");
                    }
                }
            }

            env->timing->stopTimer("PrimalStrategy");
            env->timing->stopTimer("PrimalBoundStrategyRootSearch");
        }
    }
}
} // namespace SHOT