#include "LinesearchMethodBoost.h"

std::vector<int> activeConstraints;
double lastActiveConstraintUpdateValue;

Test::Test()
{

}

void Test::determineActiveConstraints(double constrTol)
{
	valFirstPt = -OSDBL_MAX;
	valSecondPt = -OSDBL_MAX;

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

	lastActiveConstraintUpdateValue = OSDBL_MAX;
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
	auto mostDevConstr = originalProblem->getMostDeviatingConstraint(ptNew, tmpActiveConstraints);

	double validNewPt = mostDevConstr.first.value;

	if (validNewPt > 0 && validNewPt <= lastActiveConstraintUpdateValue
			&& mostDevConstr.second.size() < tmpActiveConstraints.size())
	{
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
		processInfo->outputError(
				"     Linesearch error: sizes of points vary: " + std::to_string(ptA.size()) + " != "
						+ std::to_string(ptB.size()));
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

	/*
	 if (test->valFirstPt == test->valSecondPt)
	 {
	 throw new ErrorClass(
	 "Exterior point and interior point has the same value " + to_string(test->valFirstPt) + " exterior "
	 + to_string(test->valSecondPt));
	 }
	 else if (test->valFirstPt > test->valSecondPt)
	 {
	 throw new ErrorClass(
	 "Exterior point has greater value than interior: interior " + to_string(test->valFirstPt) + " exterior "
	 + to_string(test->valSecondPt));
	 }*/

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

	int tempFEvals = processInfo->numFunctionEvals;
	Result r1 = boost::math::tools::toms748_solve(*test, 0.0, 1.0, TerminationCondition(lambdaTol), max_iter);

	int resFVals = processInfo->numFunctionEvals - tempFEvals;
	if (max_iter == Nmax)
	{
		processInfo->outputWarning(
				"     Warning, number of line search iterations " + to_string(max_iter) + " reached!");
	}
	else
	{
		processInfo->outputInfo(
				"     Line search iterations: " + to_string(max_iter) + ". Function evaluations: "
						+ to_string(resFVals));
	}

	for (int i = 0; i < length; i++)
	{
		ptNew.at(i) = r1.first * ptA.at(i) + (1 - r1.first) * ptB.at(i);
		ptNew2.at(i) = r1.second * ptA.at(i) + (1 - r1.second) * ptB.at(i);
	}

	auto validNewPt = processInfo->originalProblem->isConstraintsFulfilledInPoint(ptNew);

	if (!validNewPt) // ptNew Outside feasible region
	{
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
}

