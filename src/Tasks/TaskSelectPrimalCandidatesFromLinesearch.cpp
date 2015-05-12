/*
 * TaskSelectPrimalCandidatesFromLinesearch.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include <TaskSelectPrimalCandidatesFromLinesearch.h>

class Test2
{
	private:
		OptProblemOriginal *originalProblem;
		//std::vector<char> varTypes;

	public:
		std::vector<double> firstPt;
		std::vector<double> secondPt;
		Test2(OptProblemOriginal *prob)
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

			return value;
		}
};

class TerminationCondition2
{
	private:
		double tol;

	public:
		TerminationCondition2(double tolerance)
		{
			tol = tolerance;
		}

		bool operator()(double min, double max)
		{
			return abs(min - max) <= tol;
		}
};

TaskSelectPrimalCandidatesFromLinesearch::TaskSelectPrimalCandidatesFromLinesearch()
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

TaskSelectPrimalCandidatesFromLinesearch::~TaskSelectPrimalCandidatesFromLinesearch()
{
	// TODO Auto-generated destructor stub
}

void TaskSelectPrimalCandidatesFromLinesearch::run()
{
	auto currIter = processInfo->getCurrentIteration();

	if (currIter->isMILP() && processInfo->getRelativeObjectiveGap() > 1e-10)
	{
		processInfo->startTimer("PrimalBoundTotal");
		processInfo->startTimer("PrimalBoundLinesearch");

		auto allSolutions = processInfo->getCurrentIteration()->solutionPoints;
		if (settings->getBoolSetting("UseObjectiveLinesearch", "PrimalBound")
				&& processInfo->originalProblem->isObjectiveFunctionNonlinear())
		{
			Test2 t(processInfo->originalProblem);

			double tol = settings->getDoubleSetting("LinesearchEps", "Linesearch");
			boost::uintmax_t N = settings->getIntSetting("LinesearchMaxIter", "Linesearch");

			for (int i = 0; i < currIter->solutionPoints.size(); i++)
			{
				auto dualSol = currIter->solutionPoints.at(i);

				if (dualSol.maxDeviation.value < 0) continue;

				double mu = dualSol.objectiveValue;
				double error = processInfo->originalProblem->calculateConstraintFunctionValue(-1, dualSol.point);

				vector<double> tmpPoint(dualSol.point);
				tmpPoint.back() = mu + 1.2 * error;

				int numVar = processInfo->originalProblem->getNumberOfVariables();

				std::vector<double> ptNew(numVar);

				t.firstPt = dualSol.point;
				t.secondPt = tmpPoint;

				typedef std::pair<double, double> Result;
				boost::uintmax_t max_iter = N;

				Result r1 = boost::math::tools::toms748_solve(t, 0.0, 1.0, TerminationCondition2(tol), max_iter);

				if (max_iter == N)
				{
					processInfo->logger.message(1)
							<< "Warning, number of line search iterations reached for primal bound search!"
							<< CoinMessageEol;
				}

				for (int i = 0; i < numVar; i++)
				{
					ptNew.at(i) = r1.second * dualSol.point.at(i) + (1 - r1.second) * tmpPoint.at(i);
				}

				auto error2 = processInfo->originalProblem->getMostDeviatingConstraint(ptNew);

				processInfo->addPrimalSolutionCandidate(ptNew, E_PrimalSolutionSource::ObjectiveConstraint,
						currIter->iterationNumber);

				for (int i = 0; i < numVar; i++)
				{
					ptNew.at(i) = r1.first * dualSol.point.at(i) + (1 - r1.first) * tmpPoint.at(i);
				}

				auto mostDev = processInfo->originalProblem->getMostDeviatingConstraint(ptNew);

				processInfo->addDualSolutionCandidate(ptNew, E_DualSolutionSource::ObjectiveConstraint,
						currIter->iterationNumber);

				std::pair<int, std::vector<double>> tmpItem;
				tmpItem.first = mostDev.idx;
				tmpItem.second = ptNew;
				processInfo->hyperplaneWaitingList.push_back(tmpItem);

			}
		}

		for (int i = 0; i < allSolutions.size(); i++)
		{
			for (int j = 0; j < processInfo->interiorPts.size(); j++)
			{
				auto xNLP = processInfo->interiorPts.at(j).point;

				auto varTypes = processInfo->originalProblem->getVariableTypes();

				std::vector<double> xNLP2(xNLP.size());
				for (int k = 0; k < processInfo->originalProblem->getNumberOfVariables(); k++)
				{
					if (varTypes.at(k) == 'I' || varTypes.at(k) == 'B')
					{
						xNLP2.at(k) = allSolutions.at(i).point.at(k);

					}
					else
					{
						xNLP2.at(k) = xNLP.at(k);
					}
				}

				auto maxDevNLP2 = processInfo->originalProblem->getMostDeviatingAllConstraint(xNLP2);
				auto maxDevMILP = processInfo->originalProblem->getMostDeviatingAllConstraint(allSolutions.at(i).point);

				if (maxDevNLP2.value <= 0 && maxDevMILP.value >= 0)
				{
					auto xNewc2 = linesearchMethod->findZero(xNLP2, allSolutions.at(i).point,
							settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
							settings->getDoubleSetting("LinesearchEps", "Linesearch"));

					processInfo->addPrimalSolutionCandidate(xNewc2, E_PrimalSolutionSource::LinesearchFixedIntegers,
							processInfo->getCurrentIteration()->iterationNumber);
				}
			}
		}

		processInfo->stopTimer("PrimalBoundLinesearch");
		processInfo->stopTimer("PrimalBoundTotal");
	}
}

std::string TaskSelectPrimalCandidatesFromLinesearch::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
