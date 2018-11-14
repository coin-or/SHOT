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
    env->process->stopTimer("DualCutGenerationRootSearch");
}

TaskSelectHyperplanePointsIndividualLinesearch::~TaskSelectHyperplanePointsIndividualLinesearch()
{
    if (hyperplaneSolutionPointStrategyInitialized)
    {
        delete tSelectHPPts;
    }
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
    std::vector<bool> hyperplaneAddedToConstraint(env->reformulatedProblem->properties.numberOfNonlinearConstraints, false);

    for (int i = 0; i < solPoints.size(); i++)
    {
        auto maxDevConstr = env->reformulatedProblem->getMostDeviatingNumericConstraint(solPoints.at(i).point);

        if (maxDevConstr)
        {
            continue;
        }
        else
        {
            for (int j = 0; j < env->process->interiorPts.size(); j++)
            {
                auto xNLP = env->process->interiorPts.at(j)->point;

                int constraintCounter = 0;
                for (auto C : env->reformulatedProblem->nonlinearConstraints)
                {
                    // Check if max hyperplanes per iteration counter has been reached
                    if (addedHyperplanes >= env->settings->getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual"))
                        return;

                    // Do not add hyperplane if one has been added for this constraint already
                    if (useUniqueConstraints && hyperplaneAddedToConstraint.at(constraintCounter))
                        continue;

                    auto constrDevExterior = C->calculateNumericValue(solPoints.at(i).point);

                    if (isnan(constrDevExterior.functionValue))
                    {
                        continue;
                    }

                    // Do not add hyperplane if less than this tolerance or negative
                    if (constrDevExterior.error < env->settings->getDoubleSetting("ESH.Linesearch.ConstraintTolerance", "Dual"))
                        continue;

                    // Do not add hyperplane if constraint value is much less than largest
                    if (constrDevExterior.error < env->settings->getDoubleSetting("ESH.Linesearch.ConstraintFactor", "Dual") * maxDevConstr.value)
                        continue;

                    VectorDouble externalPoint;
                    VectorDouble internalPoint;

                    try
                    {
                        env->process->startTimer("DualCutGenerationRootSearch");
                        auto xNewc = env->process->linesearchMethod->findZero(xNLP, solPoints.at(i).point,
                                                                              env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                              env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
                                                                              env->settings->getDoubleSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver"),
                                                                              env->reformulatedProblem->nonlinearConstraints);

                        env->process->stopTimer("DualCutGenerationRootSearch");
                        internalPoint = xNewc.first;
                        externalPoint = xNewc.second;
                    }
                    catch (std::exception &e)
                    {
                        env->process->stopTimer("DualCutGenerationRootSearch");
                        externalPoint = solPoints.at(i).point;

                        env->output->outputError(
                            "     Cannot find solution with linesearch. Interior value: " + std::to_string(env->process->interiorPts.at(j)->maxDevatingConstraint.value) + " exterior value: " + std::to_string(constrDevExterior.normalizedValue));
                    }

                    auto constrDevBoundary = C->calculateNumericValue(externalPoint);

                    if (constrDevBoundary.error >= 0)
                    {
                        Hyperplane hyperplane;
                        hyperplane.sourceConstraint = constrDevBoundary.constraint;
                        hyperplane.sourceConstraintIndex = C->index;
                        hyperplane.generatedPoint = externalPoint;

                        if (solPoints.at(i).isRelaxedPoint)
                        {
                            hyperplane.source = E_HyperplaneSource::MIPCallbackRelaxed;
                        }
                        else if (i == 0 && currIter->isMIP())
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

                        hyperplaneAddedToConstraint.at(constraintCounter) = true;

                        env->output->outputInfo(
                            "     Added hyperplane to constraint " + std::to_string(C->index) + " original dev: " + std::to_string(constrDevExterior.error));

                        hyperplane.generatedPoint.clear();
                        constraintCounter++;
                    }

                    externalPoint.clear();
                    internalPoint.clear();
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
