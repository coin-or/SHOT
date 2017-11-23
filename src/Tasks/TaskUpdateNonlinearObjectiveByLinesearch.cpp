/*
 * TaskSelectPrimalCandidatesFromLinesearch.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include "TaskUpdateNonlinearObjectiveByLinesearch.h"

TaskUpdateNonlinearObjectiveByLinesearch::TaskUpdateNonlinearObjectiveByLinesearch()
{

}

TaskUpdateNonlinearObjectiveByLinesearch::~TaskUpdateNonlinearObjectiveByLinesearch()
{
	// TODO Auto-generated destructor stub
}

void TaskUpdateNonlinearObjectiveByLinesearch::run()
{
	ProcessInfo::getInstance().startTimer("ObjectiveLinesearch");

	ProcessInfo::getInstance().setObjectiveUpdatedByLinesearch(false);

	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->isMILP())
	{
		bool isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();
		std::vector<int> constrIdxs;
		constrIdxs.push_back(-1);

		auto allSolutions = currIter->solutionPoints;

		for (int i = 0; i < allSolutions.size(); i++)
		{
			auto dualSol = allSolutions.at(i);

			auto oldObjVal = allSolutions.at(i).objectiveValue;

			if (dualSol.maxDeviation.value < 0) continue;

			double mu = dualSol.objectiveValue;
			double error = ProcessInfo::getInstance().originalProblem->calculateConstraintFunctionValue(-1,
					dualSol.point);

			vector<double> tmpPoint(dualSol.point);
			tmpPoint.back() = mu + 1.05 * error;

			std::vector<double> internalPoint;
			std::vector<double> externalPoint;

			try
			{
				auto xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(tmpPoint, dualSol.point,
						Settings::getInstance().getIntSetting("LinesearchMaxIter", "Linesearch"),
						Settings::getInstance().getDoubleSetting("LinesearchLambdaEps", "Linesearch"), 0, constrIdxs);

				internalPoint = xNewc.first;
				externalPoint = xNewc.second;

				auto mostDevInner = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(
						internalPoint);
				auto mostDevOuter = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(
						externalPoint);

				allSolutions.at(i).maxDeviation = mostDevOuter;
				allSolutions.at(i).objectiveValue =
						ProcessInfo::getInstance().originalProblem->calculateOriginalObjectiveValue(externalPoint);
				allSolutions.at(i).point.back() = externalPoint.back();

				auto diffobj = abs(oldObjVal - allSolutions.at(i).objectiveValue);

				if (diffobj > Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm"))
				{
					Hyperplane hyperplane;
					hyperplane.sourceConstraintIndex = mostDevOuter.idx;
					hyperplane.generatedPoint = externalPoint;
					hyperplane.source = E_HyperplaneSource::PrimalSolutionSearch;
					ProcessInfo::getInstance().hyperplaneWaitingList.push_back(hyperplane);
				}

				// Update the iteration solution as well (for i==0)
				if (i == 0 && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
				{
					currIter->maxDeviation = mostDevOuter.value;
					currIter->maxDeviationConstraint = mostDevOuter.idx;
					currIter->objectiveValue = allSolutions.at(0).objectiveValue;

					ProcessInfo::getInstance().setObjectiveUpdatedByLinesearch(true);
					ProcessInfo::getInstance().outputInfo(
							"     Obj. for sol. # 0 upd. by l.s." + to_string(oldObjVal) + " -> "
									+ to_string(allSolutions.at(i).objectiveValue) + " (diff:" + to_string(diffobj)
									+ ")  #");
					// Change the status of the solution if it has been updated much
					if (diffobj > Settings::getInstance().getDoubleSetting("GapTermTolAbsolute", "Algorithm"))
					{
						if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
						{
							currIter->solutionStatus = E_ProblemSolutionStatus::SolutionLimit;
						}
					}
				}
				else
				{
					ProcessInfo::getInstance().outputInfo(
							"     Obj. for sol. #" + std::to_string(i) + " upd. by l.s." + to_string(oldObjVal) + " -> "
									+ to_string(allSolutions.at(i).objectiveValue) + " (diff:" + to_string(diffobj)
									+ ")  #");
				}

				ProcessInfo::getInstance().addPrimalSolutionCandidate(internalPoint, E_PrimalSolutionSource::Linesearch,
						ProcessInfo::getInstance().getCurrentIteration()->iterationNumber);

			}
			catch (std::exception &e)
			{
				ProcessInfo::getInstance().outputWarning(
						"     Cannot find solution with linesearch for updating nonlinear objective.");
			}
		}
	}

	ProcessInfo::getInstance().stopTimer("ObjectiveLinesearch");
}

std::string TaskUpdateNonlinearObjectiveByLinesearch::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
