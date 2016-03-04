/*
 * TaskSelectPrimalCandidatesFromLinesearch.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include <TaskSelectPrimalCandidatesFromLinesearch.h>

/*
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
 };*/

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

		/*
		 if (processInfo->originalProblem->isObjectiveFunctionNonlinear()
		 && settings->getBoolSetting("UseObjectiveLinesearch", "PrimalBound"))
		 {
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

		 auto xNewc = linesearchMethod->findZero(dualSol.point, allSolutions.at(i).point,
		 settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
		 settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"),
		 settings->getDoubleSetting("LinesearchConstrEps", "Linesearch"));

		 auto mostDev = processInfo->originalProblem->getMostDeviatingConstraint(ptNew);

		 Hyperplane hyperplane;
		 hyperplane.sourceConstraintIndex = mostDev.idx;
		 hyperplane.generatedPoint = ptNew;
		 hyperplane.source = E_HyperplaneSource::PrimalSolutionSearch;

		 processInfo->hyperplaneWaitingList.push_back(hyperplane);
		 }
		 }*/

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

				if (maxDevNLP2.value <= 0 && maxDevMILP.value > 0)
				{
					try
					{

						processInfo->startTimer("PrimalBoundLinesearch");
						auto xNewc = linesearchMethod->findZero(xNLP2, allSolutions.at(i).point,
								settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
								settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"), 0);

						processInfo->stopTimer("PrimalBoundLinesearch");

						processInfo->addPrimalSolutionCandidate(xNewc.first, E_PrimalSolutionSource::Linesearch,
								processInfo->getCurrentIteration()->iterationNumber);

						//processInfo->addPrimalSolutionCandidate(xNewc.second,
						//		E_PrimalSolutionSource::LinesearchFixedIntegers,
						//		processInfo->getCurrentIteration()->iterationNumber);
					}
					catch (std::exception &e)
					{

						processInfo->logger.message(1) << "Cannot find solution with primal bound linesearch: "
								<< CoinMessageNewline << e.what() << CoinMessageEol;
						processInfo->stopTimer("PrimalBoundLinesearch");
					}

					//auto tmpMostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(externalPoint);

					//auto xNewc = linesearchMethod->findZero(xNLP2, allSolutions.at(i).point,
					//		settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
					//		settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"), 10 ^ (-17));

					/*processInfo->addPrimalSolutionCandidate(xNewc.first,
					 E_PrimalSolutionSource::LinesearchFixedIntegers,
					 processInfo->getCurrentIteration()->iterationNumber);
					 */

				}
			}

			processInfo->stopTimer("PrimalBoundTotal");
		}
	}
}

std::string TaskSelectPrimalCandidatesFromLinesearch::getType()
{
	std::string type = typeid(this).name();
	return (type);
}

