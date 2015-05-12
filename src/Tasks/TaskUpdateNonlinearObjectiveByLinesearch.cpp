/*
 * TaskSelectPrimalCandidatesFromLinesearch.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include <TaskUpdateNonlinearObjectiveByLinesearch.h>

class Test3
{
	private:
		OptProblemOriginal *originalProblem;
		//std::vector<char> varTypes;

	public:
		std::vector<double> firstPt;
		std::vector<double> secondPt;
		Test3(OptProblemOriginal *prob)
		{
			originalProblem = prob;
		}

		double operator()(const double x)
		{
			int length = firstPt.size();
			std::vector<double> ptNew(length);

			for (int i = 0; i < length; i++)
			{
				ptNew.at(i) = x * firstPt.at(i) + (1 - x) * secondPt.at(i);
			}

			auto value = originalProblem->calculateConstraintFunctionValue(-1, ptNew);

			return (value);
		}
};

class TerminationCondition3
{
	private:
		double tol;

	public:
		TerminationCondition3(double tolerance)
		{
			tol = tolerance;
		}

		bool operator()(double min, double max)
		{
			return (abs(min - max) <= tol);
		}
};

TaskUpdateNonlinearObjectiveByLinesearch::TaskUpdateNonlinearObjectiveByLinesearch()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("PrimalBoundTotal");
	processInfo->startTimer("PrimalBoundLinesearch");
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

	processInfo->stopTimer("PrimalBoundLinesearch");
	processInfo->stopTimer("PrimalBoundTotal");
}

TaskUpdateNonlinearObjectiveByLinesearch::~TaskUpdateNonlinearObjectiveByLinesearch()
{
	// TODO Auto-generated destructor stub
}

void TaskUpdateNonlinearObjectiveByLinesearch::run()
{
	auto currIter = processInfo->getCurrentIteration();

	if (processInfo->originalProblem->isObjectiveFunctionNonlinear())
	{
		//processInfo->startTimer("PrimalBoundTotal");
		//processInfo->startTimer("PrimalBoundLinesearch");

		//auto allSolutions = processInfo->MILPSolver->getAllVariableSolutions();
		auto allSolutions = processInfo->getCurrentIteration()->solutionPoints;
		//if (settings->getBoolSetting("UseObjectiveLinesearch", "PrimalBound")
		//		&& processInfo->originalProblem->isObjectiveFunctionNonlinear())
		//{
		Test3 t(processInfo->originalProblem);
		for (int i = 0; i < currIter->solutionPoints.size(); i++)
		{
			//auto dualSol = currIter->solutionPoints.at(i);

			if (currIter->solutionPoints.at(i).maxDeviation.value < 0) continue;
			if (currIter->solutionPoints.at(i).maxDeviation.value > 1e6) continue;

			double mu = currIter->solutionPoints.at(i).objectiveValue;
			double error = processInfo->originalProblem->calculateConstraintFunctionValue(-1,
					currIter->solutionPoints.at(i).point);

			vector<double> tmpPoint(currIter->solutionPoints.at(i).point);
			tmpPoint.back() = mu + 1.1 * error;

			//std::cout << "Error is " << error << std::endl;

			//std::cout << "adding primal solution candidate" << std::endl;

			int numVar = processInfo->originalProblem->getNumberOfVariables();
			//std::vector<double> ptA(numVar);
			//std::vector<double> ptB(numVar);
			std::vector<double> ptNew(numVar);

			t.firstPt = currIter->solutionPoints.at(i).point;
			t.secondPt = tmpPoint;

			typedef std::pair<double, double> Result;
			boost::uintmax_t max_iter = 100;
			Result r1 = boost::math::tools::toms748_solve(t, 0.0, 1.0, TerminationCondition3(1e-18), max_iter);

			for (int j = 0; j < numVar; j++)
			{
				ptNew.at(j) = r1.second * currIter->solutionPoints.at(i).point.at(j) + (1 - r1.second) * tmpPoint.at(j);
			}

			auto error2 = processInfo->originalProblem->getMostDeviatingConstraint(ptNew);

			processInfo->addPrimalSolutionCandidate(ptNew, E_PrimalSolutionSource::ObjectiveConstraint,
					currIter->iterationNumber);

			for (int j = 0; j < numVar; j++)
			{
				ptNew.at(j) = r1.first * currIter->solutionPoints.at(i).point.at(j) + (1 - r1.first) * tmpPoint.at(j);
			}
			currIter->solutionPoints.at(i).maxDeviation = processInfo->originalProblem->getMostDeviatingConstraint(
					ptNew);

			currIter->solutionPoints.at(i).objectiveValue =
					processInfo->originalProblem->calculateOriginalObjectiveValue(ptNew);

			currIter->solutionPoints.at(i).point = ptNew;

			std::cout << "Maxiter: " << max_iter << std::endl;
			std::cout << "New obj value: " << currIter->objectiveValue << std::endl;
			std::cout << "Max deviation: " << currIter->maxDeviation << std::endl;

			/*auto mostDev = processInfo->originalProblem->getMostDeviatingConstraint(ptNew);

			 processInfo->addDualSolutionCandidate(ptNew, E_DualSolutionSource::ObjectiveConstraint,
			 currIter->iterationNumber);

			 std::pair<int, std::vector<double>> tmpItem;
			 tmpItem.first = mostDev.idx;
			 tmpItem.second = ptNew;
			 processInfo->hyperplaneWaitingList.push_back(tmpItem);
			 */
			//}
		}

		//processInfo->stopTimer("PrimalBoundLinesearch");
		//processInfo->stopTimer("PrimalBoundTotal");
	}
}

std::string TaskUpdateNonlinearObjectiveByLinesearch::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
