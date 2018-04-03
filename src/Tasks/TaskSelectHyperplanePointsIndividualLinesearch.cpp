/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsIndividualLinesearch.h"

TaskSelectHyperplanePointsIndividualLinesearch::TaskSelectHyperplanePointsIndividualLinesearch()
{
    nonlinearConstraintIdxs = ProcessInfo::getInstance().originalProblem->getNonlinearConstraintIndexes();

    ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
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
    this->run(ProcessInfo::getInstance().getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsIndividualLinesearch::run(vector<SolutionPoint> solPoints)
{
    int addedHyperplanes = 0;

    bool useUniqueConstraints = Settings::getInstance().getBoolSetting("ESH.Linesearch.IndividualConstraints.Unique", "Dual");

    auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

    if (ProcessInfo::getInstance().interiorPts.size() == 0)
    {
        if (!hyperplaneSolutionPointStrategyInitialized)
        {
            tSelectHPPts = new TaskSelectHyperplanePointsSolution();
            hyperplaneSolutionPointStrategyInitialized = true;
        }

        Output::getInstance().outputWarning("     Adding cutting plane since no interior point is known.");

        tSelectHPPts->run(solPoints);

        return;
    }

    // Contains boolean array that indicates if a constraint has been added or not
    std::vector<bool> hyperplaneAddedToConstraint(
        ProcessInfo::getInstance().originalProblem->getNumberOfNonlinearConstraints(), false);

    for (int i = 0; i < solPoints.size(); i++)
    {
        auto maxDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(
            solPoints.at(i).point);

        if (maxDevConstr.value <= 0)
        {
            continue;
        }
        else
        {
            for (int j = 0; j < ProcessInfo::getInstance().interiorPts.size(); j++)
            {
                auto xNLP = ProcessInfo::getInstance().interiorPts.at(j)->point;

                for (int k = 0; k < nonlinearConstraintIdxs.size(); k++)
                {
                    int currConstrIdx = nonlinearConstraintIdxs.at(k);

                    // Check if max hyperplanes per iteration counter has been reached
                    if (addedHyperplanes >= Settings::getInstance().getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual"))
                        return;

                    // Do not add hyperplane if one has been added for this constraint already
                    if (useUniqueConstraints && ((currConstrIdx != -1 && hyperplaneAddedToConstraint.at(k)) || ((currConstrIdx == -1) && hyperplaneAddedToConstraint.back())))
                        continue;

                    auto constrDevExterior =
                        ProcessInfo::getInstance().originalProblem->calculateConstraintFunctionValue(currConstrIdx,
                                                                                                     solPoints.at(i).point);

                    if (isnan(constrDevExterior))
                    {
                        continue;
                    }

                    // Do not add hyperplane if less than this tolerance or negative
                    if (constrDevExterior < Settings::getInstance().getDoubleSetting("ESH.Linesearch.ConstraintTolerance", "Dual"))
                        continue;

                    // Do not add hyperplane if constraint value is much less than largest
                    if (constrDevExterior < Settings::getInstance().getDoubleSetting("ESH.Linesearch.ConstraintFactor", "Dual") * maxDevConstr.value)
                        continue;

                    std::vector<double> externalPoint;
                    std::vector<double> internalPoint;

                    std::vector<int> currentIndexes;

                    currentIndexes.push_back(currConstrIdx);

                    try
                    {
                        ProcessInfo::getInstance().startTimer("HyperplaneLinesearch");
                        auto xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(xNLP, solPoints.at(i).point,
                                                                                           Settings::getInstance().getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                                           Settings::getInstance().getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
                                                                                           Settings::getInstance().getDoubleSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver"),
                                                                                           currentIndexes);

                        ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
                        internalPoint = xNewc.first;
                        externalPoint = xNewc.second;
                    }
                    catch (std::exception &e)
                    {
                        ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
                        externalPoint = solPoints.at(i).point;

                        Output::getInstance().Output::getInstance().outputError(
                            "     Cannot find solution with linesearch. Interior value: " + to_string(ProcessInfo::getInstance().interiorPts.at(j)->maxDevatingConstraint.value) + " exterior value: " + to_string(constrDevExterior));
                    }

                    auto constrDevBoundary =
                        ProcessInfo::getInstance().originalProblem->calculateConstraintFunctionValue(currConstrIdx,
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

                        ProcessInfo::getInstance().hyperplaneWaitingList.push_back(hyperplane);
                        addedHyperplanes++;

                        if (currConstrIdx != -1)
                            hyperplaneAddedToConstraint.at(k) = true;
                        else
                            hyperplaneAddedToConstraint.at(hyperplaneAddedToConstraint.back()) = true;

                        Output::getInstance().outputInfo(
                            "     Added hyperplane to constraint " + to_string(currConstrIdx) + " original dev: " + to_string(constrDevExterior));

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
