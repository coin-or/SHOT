/*
 * TaskSelectPrimalCandidatesFromLinesearch.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include <TaskUpdateNonlinearObjectiveByLinesearch.h>

TaskUpdateNonlinearObjectiveByLinesearch::TaskUpdateNonlinearObjectiveByLinesearch()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("ObjectiveLinesearch");
	if (settings->getIntSetting("LinesearchMethod", "Linesearch") == static_cast<int>(ES_LinesearchMethod::Boost))
	{
		processInfo->logger.message(2) << "Boost linesearch implementation selected for primal heuristics"
				<< CoinMessageEol;
		linesearchMethod = new LinesearchMethodBoost();
	}
	else if (settings->getIntSetting("LinesearchMethod", "Linesearch")
			== static_cast<int>(ES_LinesearchMethod::Bisection))
	{
		processInfo->logger.message(2) << "Bisection linesearch selected primal heuristics" << CoinMessageEol;
		linesearchMethod = new LinesearchMethodBisection();
	}

	processInfo->stopTimer("ObjectiveLinesearch");
}

TaskUpdateNonlinearObjectiveByLinesearch::~TaskUpdateNonlinearObjectiveByLinesearch()
{
	// TODO Auto-generated destructor stub
}

void TaskUpdateNonlinearObjectiveByLinesearch::run()
{
	processInfo->startTimer("ObjectiveLinesearch");

	auto currIter = processInfo->getCurrentIteration();

	if (currIter->isMILP() && processInfo->getRelativeObjectiveGap() > 1e-10)
	{
		bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();
		std::vector<int> constrIdxs;
		constrIdxs.push_back(-1);

		auto allSolutions = currIter->solutionPoints;

		for (int i = 0; i < allSolutions.size(); i++)
		{
			auto dualSol = allSolutions.at(i);

			auto oldObjVal = allSolutions.at(i).objectiveValue;

			//std::cout << dualSol.maxDeviation.value << std::endl;

			if (dualSol.maxDeviation.value < 0) continue;

			double mu = dualSol.objectiveValue;
			double error = processInfo->originalProblem->calculateConstraintFunctionValue(-1, dualSol.point);

			vector<double> tmpPoint(dualSol.point);
			tmpPoint.back() = mu + 1.05 * error;

			std::vector<double> internalPoint;
			std::vector<double> externalPoint;

			try
			{
				auto xNewc = linesearchMethod->findZero(tmpPoint, dualSol.point,
						settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
						settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"), 0, constrIdxs);

				internalPoint = xNewc.first;
				externalPoint = xNewc.second;

				auto mostDevInner = processInfo->originalProblem->getMostDeviatingConstraint(internalPoint);
				auto mostDevOuter = processInfo->originalProblem->getMostDeviatingConstraint(externalPoint);

				Hyperplane hyperplane;
				hyperplane.sourceConstraintIndex = mostDevOuter.idx;
				hyperplane.generatedPoint = externalPoint;
				hyperplane.source = E_HyperplaneSource::PrimalSolutionSearch;

				processInfo->hyperplaneWaitingList.push_back(hyperplane);

				allSolutions.at(i).maxDeviation = mostDevOuter;
				allSolutions.at(i).objectiveValue = processInfo->originalProblem->calculateOriginalObjectiveValue(
						externalPoint);
				allSolutions.at(i).point.back() = externalPoint.back();

				//UtilityFunctions::displayVector(externalPoint);
				//UtilityFunctions::displayVector(currIter->solutionPoints.at(i).point);

				if (i == 0)
				{
					currIter->maxDeviation = mostDevOuter.value;
					currIter->maxDeviationConstraint = mostDevOuter.idx;
					currIter->objectiveValue = allSolutions.at(i).objectiveValue;

					//std::cout << "New objective: " << currIter->objectiveValue << std::endl;

					/*
					 if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
					 {
					 currIter->solutionStatus = E_ProblemSolutionStatus::SolutionLimit;
					 }*/
				}

				//std::cout << "PT: " << ptNew.at(ptNew.size() - 1) << std::endl;

				/*Can never add a dual solution for the point is not optimal
				 * if (i == 0 && currIter->solutionStatus != E_ProblemSolutionStatus::Optimal) // Do not have the point, only the objective bound
				 {
				 DualSolution sol =
				 { externalPoint, E_DualSolutionSource::MILPSolutionOptimal,
				 currIter->solutionPoints.at(i).objectiveValue, currIter->iterationNumber };
				 processInfo->addDualSolutionCandidate(sol);
				 }*/

				processInfo->addPrimalSolutionCandidate(internalPoint, E_PrimalSolutionSource::Linesearch,
						processInfo->getCurrentIteration()->iterationNumber);
				//processInfo->logger.message(2) << "    Obj. var." << oldObjVal << "->"
				//<< currIter->solutionPoints.at(i).objectiveValue << CoinMessageEol;

			}
			catch (std::exception &e)
			{
				processInfo->logger.message(0)
						<< "Cannot find solution with linesearch for updating nonlinear objective: "
						<< CoinMessageNewline << e.what() << CoinMessageEol;
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
