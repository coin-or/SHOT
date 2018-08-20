/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsIndividualLinesearch.h"

TaskSelectHyperplanePointsIndividualLinesearch::TaskSelectHyperplanePointsIndividualLinesearch(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("DualCutGenerationRootSearch");

    nonlinearConstraintIdxs = env->model->originalProblem->getNonlinearConstraintIndexes();

    env->process->stopTimer("DualCutGenerationRootSearch");
}

TaskSelectHyperplanePointsIndividualLinesearch::~TaskSelectHyperplanePointsIndividualLinesearch()
{
    if (hyperplaneSolutionPointStrategyInitialized)
    {
        delete tSelectHPPts;
    }

    nonlinearConstraintIdxs.clear();
}

void TaskSelectHyperplanePointsIndividualLinesearch::run()
{
    this->run(env->process->getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsIndividualLinesearch::run(std::vector<SolutionPoint> solPoints)
{
    env->process->startTimer("DualCutGenerationRootSearch");

    int addedHyperplanes = 0;

    bool useUniqueConstraints = env->settings->getBoolSetting("ESH.Linesearch.IndividualConstraints.Unique", "Dual");

    auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration

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

    // Contains boolean array that indicates if a constraint has been added or not
    std::vector<bool> hyperplaneAddedToConstraint(
        env->model->originalProblem->getNumberOfNonlinearConstraints(), false);

    for (int i = 0; i < solPoints.size(); i++)
    {
        auto maxDevConstr = env->model->originalProblem->getMostDeviatingConstraint(
            solPoints.at(i).point);

        if (maxDevConstr.value <= 0)
        {
            continue;
        }
        else
        {
            for (int j = 0; j < env->process->interiorPts.size(); j++)
            {
                auto xNLP = env->process->interiorPts.at(j)->point;

                for (int k = 0; k < nonlinearConstraintIdxs.size(); k++)
                {
                    int currConstrIdx = nonlinearConstraintIdxs.at(k);

                    // Check if max hyperplanes per iteration counter has been reached
                    if (addedHyperplanes >= env->settings->getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual"))
                        return;

                    // Do not add hyperplane if one has been added for this constraint already
                    if (useUniqueConstraints && ((currConstrIdx != -1 && hyperplaneAddedToConstraint.at(k)) || ((currConstrIdx == -1) && hyperplaneAddedToConstraint.back())))
                        continue;

                    auto constrDevExterior =
                        env->model->originalProblem->calculateConstraintFunctionValue(currConstrIdx,
                                                                                      solPoints.at(i).point);

                    if (isnan(constrDevExterior))
                    {
                        continue;
                    }

                    // Do not add hyperplane if less than this tolerance or negative
                    if (constrDevExterior < env->settings->getDoubleSetting("ESH.Linesearch.ConstraintTolerance", "Dual"))
                        continue;

                    // Do not add hyperplane if constraint value is much less than largest
                    if (constrDevExterior < env->settings->getDoubleSetting("ESH.Linesearch.ConstraintFactor", "Dual") * maxDevConstr.value)
                        continue;

                    VectorDouble externalPoint;
                    VectorDouble internalPoint;

                    VectorInteger currentIndexes;

                    currentIndexes.push_back(currConstrIdx);

                    try
                    {
                        env->process->startTimer("DualCutGenerationRootSearch");
                        auto xNewc = env->process->linesearchMethod->findZero(xNLP, solPoints.at(i).point,
                                                                              env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                              env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
                                                                              env->settings->getDoubleSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver"),
                                                                              currentIndexes);

                        env->process->stopTimer("DualCutGenerationRootSearch");
                        internalPoint = xNewc.first;
                        externalPoint = xNewc.second;
                    }
                    catch (std::exception &e)
                    {
                        env->process->stopTimer("DualCutGenerationRootSearch");
                        externalPoint = solPoints.at(i).point;

                        env->output->outputError(
                            "     Cannot find solution with linesearch. Interior value: " + std::to_string(env->process->interiorPts.at(j)->maxDevatingConstraint.value) + " exterior value: " + std::to_string(constrDevExterior));
                    }

                    auto constrDevBoundary =
                        env->model->originalProblem->calculateConstraintFunctionValue(currConstrIdx,
                                                                                      externalPoint);

                    if (constrDevBoundary >= 0)
                    {
                        Hyperplane hyperplane;
                        hyperplane.sourceConstraintIndex = currConstrIdx;
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

                        if (currConstrIdx != -1)
                            hyperplaneAddedToConstraint.at(k) = true;
                        else
                            hyperplaneAddedToConstraint.at(hyperplaneAddedToConstraint.back()) = true;

                        env->output->outputInfo(
                            "     Added hyperplane to constraint " + std::to_string(currConstrIdx) + " original dev: " + std::to_string(constrDevExterior));

                        hyperplane.generatedPoint.clear();
                    }

                    externalPoint.clear();
                    internalPoint.clear();
                    currentIndexes.clear();
                }

                xNLP.clear();
            }
        }
    }

    hyperplaneAddedToConstraint.clear();
}

std::string TaskSelectHyperplanePointsIndividualLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
