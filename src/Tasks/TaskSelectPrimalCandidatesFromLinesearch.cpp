/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromLinesearch.h"

namespace SHOT
{

TaskSelectPrimalCandidatesFromLinesearch::TaskSelectPrimalCandidatesFromLinesearch(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
}

TaskSelectPrimalCandidatesFromLinesearch::~TaskSelectPrimalCandidatesFromLinesearch()
{
}

void TaskSelectPrimalCandidatesFromLinesearch::run()
{
    this->run(env->process->getCurrentIteration()->solutionPoints);
}

std::string TaskSelectPrimalCandidatesFromLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

void TaskSelectPrimalCandidatesFromLinesearch::run(std::vector<SolutionPoint> solPoints)
{
    auto currIter = env->process->getCurrentIteration();

    if (currIter->isMIP() && env->process->getRelativeObjectiveGap() > 1e-10 || env->process->usedSolutionStrategy == E_SolutionStrategy::NLP)
    {
        env->process->startTimer("PrimalStrategy");
        env->process->startTimer("PrimalBoundStrategyRootSearch");

        for (int i = 0; i < solPoints.size(); i++)
        {
            for (int j = 0; j < env->process->interiorPts.size(); j++)
            {
                auto xNLP = env->process->interiorPts.at(j)->point;

                for (auto &V : env->problem->binaryVariables)
                {
                    xNLP.at(V->index) = solPoints.at(i).point.at(V->index);
                }

                for (auto &V : env->problem->integerVariables)
                {
                    xNLP.at(V->index) = solPoints.at(i).point.at(V->index);
                }

                auto maxDevNLP2 = env->problem->getMaxNumericConstraintValue(xNLP, env->problem->numericConstraints);
                auto maxDevMIP = env->problem->getMaxNumericConstraintValue(solPoints.at(i).point, env->problem->numericConstraints);

                if (maxDevNLP2.normalizedValue <= 0 && maxDevMIP.normalizedValue > 0)
                {
                    std::pair<VectorDouble, VectorDouble> xNewc;

                    try
                    {
                        env->process->startTimer("PrimalBoundStrategyRootSearch");
                        xNewc = env->process->linesearchMethod->findZero(xNLP, solPoints.at(i).point,
                                                                         env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                         env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
                                                                         0, env->reformulatedProblem->nonlinearConstraints);

                        env->process->stopTimer("PrimalBoundStrategyRootSearch");

                        env->process->addPrimalSolutionCandidate(xNewc.first,
                                                                 E_PrimalSolutionSource::Linesearch,
                                                                 env->process->getCurrentIteration()->iterationNumber);
                    }
                    catch (std::exception &e)
                    {
                        env->output->outputWarning("Cannot find solution with primal bound linesearch.");
                    }
                }
            }

            env->process->stopTimer("PrimalStrategy");
            env->process->stopTimer("PrimalBoundStrategyRootSearch");
        }
    }
}
} // namespace SHOT