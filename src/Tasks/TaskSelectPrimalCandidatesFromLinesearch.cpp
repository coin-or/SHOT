/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromLinesearch.h"

TaskSelectPrimalCandidatesFromLinesearch::TaskSelectPrimalCandidatesFromLinesearch(EnvironmentPtr envPtr): TaskBase(envPtr)
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

    if (currIter->isMIP() && env->process->getRelativeObjectiveGap() > 1e-10)
    {
        env->process->startTimer("PrimalStrategy");
        env->process->startTimer("PrimalBoundStrategyRootSearch");

        for (int i = 0; i < solPoints.size(); i++)
        {
            for (int j = 0; j < env->process->interiorPts.size(); j++)
            {
                auto xNLP = env->process->interiorPts.at(j)->point;

                auto varTypes = env->process->originalProblem->getVariableTypes();

                std::vector<double> xNLP2(xNLP.size());
                for (int k = 0; k < env->process->originalProblem->getNumberOfVariables(); k++)
                {
                    if (varTypes.at(k) == 'I' || varTypes.at(k) == 'B')
                    {
                        xNLP2.at(k) = solPoints.at(i).point.at(k);
                    }
                    else
                    {
                        xNLP2.at(k) = xNLP.at(k);
                    }
                }

                auto maxDevNLP2 = env->process->originalProblem->getMostDeviatingAllConstraint(xNLP2);
                auto maxDevMIP = env->process->originalProblem->getMostDeviatingAllConstraint(solPoints.at(i).point);

                if (maxDevNLP2.value <= 0 && maxDevMIP.value > 0)
                {
                    std::pair<std::vector<double>, std::vector<double>> xNewc;

                    try
                    {
                        env->process->startTimer("PrimalBoundStrategyRootSearch");
                        xNewc = env->process->linesearchMethod->findZero(xNLP2, solPoints.at(i).point,
                                                                                      env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                                      env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"), 0);

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
