/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsLinesearch.h"

TaskSelectHyperplanePointsLinesearch::TaskSelectHyperplanePointsLinesearch()
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
    this->run(ProcessInfo::getInstance().getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsLinesearch::run(vector<SolutionPoint> solPoints)
{
    ProcessInfo::getInstance().startTimer("DualCutGenerationRootSearch");

    int addedHyperplanes = 0;

    auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

    auto originalProblem = ProcessInfo::getInstance().originalProblem;

    int prevHPnum = ProcessInfo::getInstance().hyperplaneWaitingList.size();

    if (ProcessInfo::getInstance().interiorPts.size() == 0)
    {
        if (!hyperplaneSolutionPointStrategyInitialized)
        {
            tSelectHPPts = new TaskSelectHyperplanePointsSolution();
            hyperplaneSolutionPointStrategyInitialized = true;
        }

        Output::getInstance().outputWarning("     Adding cutting plane since no interior point is known.");
        tSelectHPPts->run(solPoints);

        ProcessInfo::getInstance().stopTimer("DualCutGenerationRootSearch");
        return;
    }

    for (int i = 0; i < solPoints.size(); i++)
    {
        if (originalProblem->isConstraintsFulfilledInPoint(solPoints.at(i).point))
        {
        }
        else
        {
            for (int j = 0; j < ProcessInfo::getInstance().interiorPts.size(); j++)
            {
                if (addedHyperplanes >= Settings::getInstance().getIntSetting("HyperplaneCuts.MaxPerIteration", "Dual"))
                {
                    ProcessInfo::getInstance().stopTimer("DualCutGenerationRootSearch");
                    return;
                }
                auto xNLP = ProcessInfo::getInstance().interiorPts.at(j)->point;

                std::vector<double> externalPoint;
                std::vector<double> internalPoint;

                try
                {

                    ProcessInfo::getInstance().startTimer("DualCutGenerationRootSearch");
                    auto xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(xNLP, solPoints.at(i).point,
                                                                                       Settings::getInstance().getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                                       Settings::getInstance().getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
                                                                                       Settings::getInstance().getDoubleSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver"));

                    ProcessInfo::getInstance().stopTimer("DualCutGenerationRootSearch");
                    internalPoint = xNewc.first;
                    externalPoint = xNewc.second;
                }
                catch (std::exception &e)
                {
                    ProcessInfo::getInstance().stopTimer("DualCutGenerationRootSearch");
                    externalPoint = solPoints.at(i).point;

                    Output::getInstance().outputWarning(
                        "     Cannot find solution with linesearch, using solution point instead.");
                }

                auto tmpMostDevConstr = originalProblem->getMostDeviatingConstraint(externalPoint);

                if (tmpMostDevConstr.value >= 0)
                {
                    Hyperplane hyperplane;
                    hyperplane.sourceConstraintIndex = tmpMostDevConstr.idx;
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

                    ProcessInfo::getInstance().hyperplaneWaitingList.push_back(hyperplane);
                    addedHyperplanes++;

                    Output::getInstance().outputInfo(
                        "     Added hyperplane to waiting list with deviation: " + UtilityFunctions::toString(tmpMostDevConstr.value));
                }
                else
                {
                    Output::getInstance().outputAlways("     Could not add hyperplane to waiting list.");
                }
            }
        }
    }

    ProcessInfo::getInstance().stopTimer("DualCutGenerationRootSearch");
}

std::string TaskSelectHyperplanePointsLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}