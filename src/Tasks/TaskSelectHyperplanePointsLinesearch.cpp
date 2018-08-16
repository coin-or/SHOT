/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsLinesearch.h"

TaskSelectHyperplanePointsLinesearch::TaskSelectHyperplanePointsLinesearch(EnvironmentPtr envPtr): TaskBase(envPtr)
{
}

TaskSelectHyperplanePointsLinesearch::~TaskSelectHyperplanePointsLinesearch()
{
    if (hyperplaneSolutionPointStrategyInitialized)
    {
        delete tSelectHPPts;
    }
}

void TaskSelectHyperplanePointsLinesearch::run()
{
    this->run(env->process->getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsLinesearch::run(std::vector<SolutionPoint> solPoints)
{
    env->process->startTimer("DualCutGenerationRootSearch");

    int addedHyperplanes = 0;

    auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration

    auto originalProblem = env->model->originalProblem;

    int prevHPnum = env->process->hyperplaneWaitingList.size();

    if (env->process->interiorPts.size() == 0)
    {
        if (!hyperplaneSolutionPointStrategyInitialized)
        {
            tSelectHPPts = new TaskSelectHyperplanePointsSolution(env);
            hyperplaneSolutionPointStrategyInitialized = true;
        }

        env->output->outputWarning("     Adding cutting plane since no interior point is known.");
        tSelectHPPts->run(solPoints);

        env->process->stopTimer("DualCutGenerationRootSearch");
        return;
    }

    for (int i = 0; i < solPoints.size(); i++)
    {
        if (originalProblem->isConstraintsFulfilledInPoint(solPoints.at(i).point))
        {
        }
        else
        {
            for (int j = 0; j < env->process->interiorPts.size(); j++)
            {
                if (addedHyperplanes >= env->settings->getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual"))
                {
                    env->process->stopTimer("DualCutGenerationRootSearch");
                    return;
                }
                auto xNLP = env->process->interiorPts.at(j)->point;

                DoubleVector externalPoint;
                DoubleVector internalPoint;

                try
                {

                    env->process->startTimer("DualCutGenerationRootSearch");
                    auto xNewc = env->process->linesearchMethod->findZero(xNLP, solPoints.at(i).point,
                                                                                       env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                                       env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
                                                                                       env->settings->getDoubleSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver"));

                    env->process->stopTimer("DualCutGenerationRootSearch");
                    internalPoint = xNewc.first;
                    externalPoint = xNewc.second;
                }
                catch (std::exception &e)
                {
                    env->process->stopTimer("DualCutGenerationRootSearch");
                    externalPoint = solPoints.at(i).point;

                    env->output->outputWarning(
                        "     Cannot find solution with linesearch, using solution point instead.");
                }

                auto tmpMostDevConstr = env->model->originalProblem->getMostDeviatingConstraint(externalPoint);

                if (tmpMostDevConstr.value >= 0)
                {
                    Hyperplane hyperplane;
                    hyperplane.sourceConstraintIndex = tmpMostDevConstr.idx;
                    hyperplane.generatedPoint = externalPoint;

                    if (i == 0 && currIter->isMIP())
                    {
                        hyperplane.source = E_HyperplaneSource::MIPOptimalLinesearch;
                    }
                    else if (currIter->isMIP())
                    {
                        hyperplane.source = E_HyperplaneSource::MIPSolutionPoolLinesearch;
                    }
                    else
                    {
                        hyperplane.source = E_HyperplaneSource::LPRelaxedLinesearch;
                    }

                    env->process->hyperplaneWaitingList.push_back(hyperplane);
                    addedHyperplanes++;

                    env->output->outputInfo(
                        "     Added hyperplane to waiting list with deviation: " + UtilityFunctions::toString(tmpMostDevConstr.value));
                }
                else
                {
                    env->output->outputAlways("     Could not add hyperplane to waiting list.");
                }
            }
        }
    }

    env->process->stopTimer("DualCutGenerationRootSearch");
}

std::string TaskSelectHyperplanePointsLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
