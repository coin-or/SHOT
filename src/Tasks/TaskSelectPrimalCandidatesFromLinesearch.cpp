/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalCandidatesFromLinesearch.h"

TaskSelectPrimalCandidatesFromLinesearch::TaskSelectPrimalCandidatesFromLinesearch()
{
}

TaskSelectPrimalCandidatesFromLinesearch::~TaskSelectPrimalCandidatesFromLinesearch()
{
}

void TaskSelectPrimalCandidatesFromLinesearch::run()
{
    this->run(ProcessInfo::getInstance().getCurrentIteration()->solutionPoints);
}

std::string TaskSelectPrimalCandidatesFromLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

void TaskSelectPrimalCandidatesFromLinesearch::run(vector<SolutionPoint> solPoints)
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    if (currIter->isMIP() && ProcessInfo::getInstance().getRelativeObjectiveGap() > 1e-10)
    {
        ProcessInfo::getInstance().startTimer("PrimalStrategy");
        ProcessInfo::getInstance().startTimer("PrimalBoundStrategyRootSearch");

        for (int i = 0; i < solPoints.size(); i++)
        {
            for (int j = 0; j < ProcessInfo::getInstance().interiorPts.size(); j++)
            {
                auto xNLP = ProcessInfo::getInstance().interiorPts.at(j)->point;

                auto varTypes = ProcessInfo::getInstance().originalProblem->getVariableTypes();

                std::vector<double> xNLP2(xNLP.size());
                for (int k = 0; k < ProcessInfo::getInstance().originalProblem->getNumberOfVariables(); k++)
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

                auto maxDevNLP2 = ProcessInfo::getInstance().originalProblem->getMostDeviatingAllConstraint(xNLP2);
                auto maxDevMIP = ProcessInfo::getInstance().originalProblem->getMostDeviatingAllConstraint(solPoints.at(i).point);

                if (maxDevNLP2.value <= 0 && maxDevMIP.value > 0)
                {
                    std::pair<std::vector<double>, std::vector<double>> xNewc;

                    try
                    {
                        ProcessInfo::getInstance().startTimer("PrimalBoundStrategyRootSearch");
                        xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(xNLP2, solPoints.at(i).point,
                                                                                      Settings::getInstance().getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                                      Settings::getInstance().getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"), 0);

                        ProcessInfo::getInstance().stopTimer("PrimalBoundStrategyRootSearch");

                        ProcessInfo::getInstance().addPrimalSolutionCandidate(xNewc.first,
                                                                              E_PrimalSolutionSource::Linesearch,
                                                                              ProcessInfo::getInstance().getCurrentIteration()->iterationNumber);
                    }
                    catch (std::exception &e)
                    {
                        Output::getInstance().outputWarning("Cannot find solution with primal bound linesearch.");
                    }
                }
            }

            ProcessInfo::getInstance().stopTimer("PrimalStrategy");
            ProcessInfo::getInstance().stopTimer("PrimalBoundStrategyRootSearch");
        }
    }
}
