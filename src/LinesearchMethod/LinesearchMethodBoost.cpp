#include "LinesearchMethodBoost.h"

std::vector<int> activeConstraints;
double lastActiveConstraintUpdateValue;

Test::Test()
{

}

void Test::determineActiveConstraints(double constrTol)
{
	valFirstPt = -DBL_MAX;
	valSecondPt = -DBL_MAX;

	auto allNonlinearConstrIdxs = originalProblem->getNonlinearConstraintIndexes();

	clearActiveConstraints();

	for (auto I : allNonlinearConstrIdxs)
	{
		auto tmpValFirstPt = originalProblem->calculateConstraintFunctionValue(I, firstPt);
		auto tmpValSecondPt = originalProblem->calculateConstraintFunctionValue(I, secondPt);

		if ((tmpValFirstPt > constrTol && tmpValSecondPt <= 0) || (tmpValFirstPt <= 0 && tmpValSecondPt > constrTol))
		{
			addActiveConstraint(I);
		}

		// For reuse of the function value
		if (tmpValFirstPt > valFirstPt) valFirstPt = tmpValFirstPt;
		if (tmpValSecondPt > valSecondPt) valSecondPt = tmpValSecondPt;

	}

	lastActiveConstraintUpdateValue = DBL_MAX;
}

void Test::addActiveConstraint(int constrIdx)
{
	activeConstraints.push_back(constrIdx);
}

void Test::clearActiveConstraints()
{
	activeConstraints.clear();
}

void Test::setActiveConstraints(std::vector<int> constrIdxs)
{
	activeConstraints = constrIdxs;
}

std::vector<int> Test::getActiveConstraints()
{
	return (activeConstraints);
}

double Test::operator()(const double x)
{
	int length = firstPt.size();
	std::vector<double> ptNew(length);

	for (int i = 0; i < length; i++)
	{
		ptNew.at(i) = x * firstPt.at(i) + (1 - x) * secondPt.at(i);
	}

	auto tmpActiveConstraints = getActiveConstraints();
	//auto tmpActiveConstraints = originalProblem->getNonlinearConstraintIndexes();
	auto mostDevConstr = originalProblem->getMostDeviatingConstraint(ptNew, tmpActiveConstraints);

	double validNewPt = mostDevConstr.first.value;

	//std::cout << " Deviation " << validNewPt << " <= " << lastActiveConstraintUpdateValue << std::endl;

	if (validNewPt > 0 && validNewPt <= lastActiveConstraintUpdateValue
			&& mostDevConstr.second.size() < tmpActiveConstraints.size())
	{

		//std::cout << " Deviation " << validNewPt << " <= " << lastActiveConstraintUpdateValue << " new size "
		//		<< mostDevConstr.second.size() << std::endl;

		setActiveConstraints(mostDevConstr.second);
		lastActiveConstraintUpdateValue = validNewPt;
	}

	return (validNewPt);
}

LinesearchMethodBoost::LinesearchMethodBoost()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	test = new Test();
	test->originalProblem = (processInfo->originalProblem);

}

LinesearchMethodBoost::~LinesearchMethodBoost()
{
	delete test;
}

std::pair<std::vector<double>, std::vector<double>> LinesearchMethodBoost::findZero(std::vector<double> ptA,
		std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol)
{
	std::vector<int> tmpVector;
	return (findZero(ptA, ptB, Nmax, lambdaTol, constrTol, tmpVector));
}

std::pair<std::vector<double>, std::vector<double> > LinesearchMethodBoost::findZero(std::vector<double> ptA,
		std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol, std::vector<int> constrIdxs)
{
	if (ptA.size() != ptB.size())
	{
		processInfo->logger.message(1) << " Linesearch error: sizes of points vary" << std::to_string(ptB.size())
				<< " and " << std::to_string(ptA.size()) << CoinMessageEol;
	}

	int length = ptA.size();
	std::vector<double> ptNew(length);
	std::vector<double> ptNew2(length);

	typedef std::pair<double, double> Result;
	boost::uintmax_t max_iter = Nmax;

	test->firstPt = ptA;
	test->secondPt = ptB;

	if (constrIdxs.size() == 0)
	{
		test->determineActiveConstraints(constrTol);
	}
	else
	{
		test->setActiveConstraints(constrIdxs);
		test->valFirstPt = processInfo->originalProblem->getMostDeviatingConstraint(ptA).value;
		test->valSecondPt = processInfo->originalProblem->getMostDeviatingConstraint(ptB).value;
	}

	//processInfo->logger.message(1) << " ====== Active constraints before "
	//<< (double) test->getActiveConstraints().size() << " / "
	//<< processInfo->originalProblem->getNumberOfNonlinearConstraints() << CoinMessageEol;

	if (test->getActiveConstraints().size() == 0) // All constraints are fulfilled.
	{
		if (test->valFirstPt > test->valSecondPt)
		{
			std::pair<std::vector<double>, std::vector<double>> tmpPair(ptB, ptA);

			return (tmpPair);
		}

		std::pair<std::vector<double>, std::vector<double>> tmpPair(ptA, ptB);

		return (tmpPair);
	}

//try
//{
	int tempFEvals = processInfo->numFunctionEvals;
	Result r1 = boost::math::tools::toms748_solve(*test, 0.0, 1.0, TerminationCondition(lambdaTol), max_iter);

	//processInfo->logger.message(1) << " ======  Active constraints after"
	//<< (double) test->getActiveConstraints().size() << " / "
	//<< processInfo->originalProblem->getNumberOfNonlinearConstraints() << CoinMessageEol;

	int resFVals = processInfo->numFunctionEvals - tempFEvals;
	if (max_iter == Nmax)
	{
		processInfo->logger.message(1) << "    Warning, number of line search iterations " << (double) max_iter
				<< " reached!" << CoinMessageEol;
	}
	else
	{
		processInfo->logger.message(3) << "    Line search iterations: " << (double) max_iter
				<< "   function evaluations: " << (double) resFVals << CoinMessageEol;
	}

//if (r1.first < 0.00001)
//	r1.first = 0;

	for (int i = 0; i < length; i++)
	{
		ptNew.at(i) = r1.first * ptA.at(i) + (1 - r1.first) * ptB.at(i);
		ptNew2.at(i) = r1.second * ptA.at(i) + (1 - r1.second) * ptB.at(i);
	}

//if (r1.first > 1-0.5)
//std::cout << "Line search value: " << r1.first << std::endl;

//if (max_iter > settings->getIntSetting("LinesearchMaxIter", "Linesearch")) processInfo->logger.message(1)
//		<< "Warning maximal number of line search iterations reached: " << (int) max_iter << CoinMessageEol;

	auto validNewPt = processInfo->originalProblem->isConstraintsFulfilledInPoint(ptNew);

	if (!validNewPt) // ptNew Outside feasible region
	{
		//processInfo->addPrimalSolutionCandidate(ptNew2, E_PrimalSolutionSource::Linesearch,
		//		processInfo->getCurrentIteration()->iterationNumber);

		processInfo->addPrimalSolutionCandidate(ptNew2, E_PrimalSolutionSource::Linesearch,
				processInfo->getCurrentIteration()->iterationNumber);

		std::pair<std::vector<double>, std::vector<double>> tmpPair(ptNew2, ptNew);
		return (tmpPair);
	}
	else
	{
		processInfo->addPrimalSolutionCandidate(ptNew, E_PrimalSolutionSource::Linesearch,
				processInfo->getCurrentIteration()->iterationNumber);

		std::pair<std::vector<double>, std::vector<double>> tmpPair(ptNew, ptNew2);
		return (tmpPair);
	}
	/*}
	 catch (std::exception &e)
	 {

	 processInfo->logger.message(0) << "Boost error while doing linesearch: " << e.what() << CoinMessageEol;
	 //processInfo->logger.message(0) << "Returning solution point instead. " << CoinMessageEol;
	 //std::vector<double> ptEmpty(0);

	 std::pair<std::vector<double>, std::vector<double>> tmpPair(ptB, ptA);

	 std::cout << "NEJ" << std::endl;
	 return (tmpPair);

	 if (!processInfo->originalProblem->isConstraintsFulfilledInPoint(ptA))
	 {
	 std::pair<std::vector<double>, std::vector<double>> tmpPair(ptB, ptA);

	 std::cout << "NEJ" << std::endl;
	 return (tmpPair);
	 }

	 if (!processInfo->originalProblem->isConstraintsFulfilledInPoint(ptB))
	 {
	 std::pair<std::vector<double>, std::vector<double>> tmpPair(ptA, ptB);

	 std::cout << "NEJ2" << std::endl;
	 return (tmpPair);
	 }

	 }
	 catch (...)
	 {
	 processInfo->logger.message(0) << "Boost error while doing linesearch." << CoinMessageEol;
	 //processInfo->logger.message(0) << "Returning solution point instead. " << CoinMessageEol;

	 if (!processInfo->originalProblem->isConstraintsFulfilledInPoint(ptA)) //Returns the NLP point if not on the interior

	 if (!processInfo->originalProblem->isConstraintsFulfilledInPoint(ptA))
	 {

	 std::pair<std::vector<double>, std::vector<double>> tmpPair(ptNew, ptA);
	 return (tmpPair);
	 }

	 std::pair<std::vector<double>, std::vector<double>> tmpPair(ptNew, ptB);
	 return (tmpPair);
	 }*/
}

