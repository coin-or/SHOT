/*
 * TaskSelectPrimalCandidatesFromLinesearch.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include "TaskUpdateNonlinearObjectiveByLinesearch.h"

TaskUpdateNonlinearObjectiveByLinesearch::TaskUpdateNonlinearObjectiveByLinesearch()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskUpdateNonlinearObjectiveByLinesearch::~TaskUpdateNonlinearObjectiveByLinesearch()
{
	// TODO Auto-generated destructor stub
}

void TaskUpdateNonlinearObjectiveByLinesearch::run()
{
	processInfo->startTimer("ObjectiveLinesearch");

	processInfo->setObjectiveUpdatedByLinesearch(false);

	auto currIter = processInfo->getCurrentIteration();

	if (currIter->isMILP())
	{
		bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();
		std::vector<int> constrIdxs;
		constrIdxs.push_back(-1);

		auto allSolutions = currIter->solutionPoints;

		for (int i = 0; i < allSolutions.size(); i++)
		{
			auto dualSol = allSolutions.at(i);

			auto oldObjVal = allSolutions.at(i).objectiveValue;

			if (dualSol.maxDeviation.value < 0) continue;

			double mu = dualSol.objectiveValue;
			double error = processInfo->originalProblem->calculateConstraintFunctionValue(-1, dualSol.point);

			vector<double> tmpPoint(dualSol.point);
			tmpPoint.back() = mu + 1.05 * error;

			std::vector<double> internalPoint;
			std::vector<double> externalPoint;

			try
			{
				auto xNewc = processInfo->linesearchMethod->findZero(tmpPoint, dualSol.point,
						settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
						settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"), 0, constrIdxs);

				internalPoint = xNewc.first;
				externalPoint = xNewc.second;

				auto mostDevInner = processInfo->originalProblem->getMostDeviatingConstraint(internalPoint);
				auto mostDevOuter = processInfo->originalProblem->getMostDeviatingConstraint(externalPoint);

				allSolutions.at(i).maxDeviation = mostDevOuter;
				allSolutions.at(i).objectiveValue = processInfo->originalProblem->calculateOriginalObjectiveValue(
						externalPoint);
				allSolutions.at(i).point.back() = externalPoint.back();

				auto diffobj = abs(oldObjVal - allSolutions.at(i).objectiveValue);

				if (diffobj > settings->getDoubleSetting("GapTermTolAbsolute", "Algorithm"))
				{
					Hyperplane hyperplane;
					hyperplane.sourceConstraintIndex = mostDevOuter.idx;
					hyperplane.generatedPoint = externalPoint;
					hyperplane.source = E_HyperplaneSource::PrimalSolutionSearch;
					processInfo->hyperplaneWaitingList.push_back(hyperplane);
				}

				// Update the iteration solution as well (for i==0)
				if (i == 0 && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
				{
					currIter->maxDeviation = mostDevOuter.value;
					currIter->maxDeviationConstraint = mostDevOuter.idx;
					currIter->objectiveValue = allSolutions.at(0).objectiveValue;

					processInfo->setObjectiveUpdatedByLinesearch(true);
					processInfo->outputInfo(
							"     Obj. for sol. # 0 upd. by l.s." + to_string(oldObjVal) + " -> "
									+ to_string(allSolutions.at(i).objectiveValue) + " (diff:" + to_string(diffobj)
									+ ")  #");
					// Change the status of the solution if it has been updated much
					if (diffobj > settings->getDoubleSetting("GapTermTolAbsolute", "Algorithm"))
					{
						if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
						{
							currIter->solutionStatus = E_ProblemSolutionStatus::SolutionLimit;
						}
					}
				}
				else
				{
					processInfo->outputInfo(
							"     Obj. for sol. #" + std::to_string(i) + " upd. by l.s." + to_string(oldObjVal) + " -> "
									+ to_string(allSolutions.at(i).objectiveValue) + " (diff:" + to_string(diffobj)
									+ ")  #");
				}

				processInfo->addPrimalSolutionCandidate(internalPoint, E_PrimalSolutionSource::Linesearch,
						processInfo->getCurrentIteration()->iterationNumber);

			}
			catch (std::exception &e)
			{
				processInfo->outputWarning("Cannot find solution with linesearch for updating nonlinear objective.");
			}
		}
	}

	processInfo->stopTimer("ObjectiveLinesearch");
}

std::string TaskUpdateNonlinearObjectiveByLinesearch::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
