/*
 * PrimalSolutionStrategyBase.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: alundell
 */

#include <PrimalSolutionStrategy/PrimalSolutionStrategyBase.h>

PrimalSolutionStrategyBase::PrimalSolutionStrategyBase()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

PrimalSolutionStrategyBase::~PrimalSolutionStrategyBase()
{
}

bool PrimalSolutionStrategyBase::checkPoints(std::vector<PrimalSolution> primalSols)
{
	bool updated = false;

	for (auto primalSol : primalSols)
	{
		updated = updated || checkPoint(primalSol);
	}

	return (updated);
}

bool PrimalSolutionStrategyBase::checkPoint(PrimalSolution primalSol)
{
	std::string sourceDesc;

	std::vector<double> tmpPoint(primalSol.point);
	double tmpObjVal = primalSol.objValue;

	bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();

	char HPadded = ' ';

	bool isLinConstrFulfilled = false;
	bool isNonLinConstrFulfilled = false;
	IndexValuePair mostDev;

	switch (primalSol.sourceType)
	{
		case E_PrimalSolutionSource::Linesearch:
			sourceDesc = "line search";
			break;
		case E_PrimalSolutionSource::LinesearchFixedIntegers:
			sourceDesc = "line search fixed";
			break;
		case E_PrimalSolutionSource::NLPFixedIntegers:
			sourceDesc = "NLP fixed";
			break;
		case E_PrimalSolutionSource::NLPRelaxed:
			sourceDesc = "NLP relaxed";
			break;
		case E_PrimalSolutionSource::MILPSolutionPool:
			sourceDesc = "MILP sol. pool";
			break;
		case E_PrimalSolutionSource::ObjectiveConstraint:
			sourceDesc = "Obj. constr.";
			break;
		case E_PrimalSolutionSource::LPFixedIntegers:
			sourceDesc = "LP fixed";
			break;
		case E_PrimalSolutionSource::LazyConstraintCallback:
			sourceDesc = "Lazy constraint callback";
			break;
		case E_PrimalSolutionSource::HeuristicCallback:
			sourceDesc = "Heuristic constraint callback";
			break;
		default:
			break;
	}

	if (std::isnan(primalSol.objValue))
	{
		tmpObjVal = processInfo->originalProblem->calculateOriginalObjectiveValue(primalSol.point);
	}

	if (primalSol.sourceType == E_PrimalSolutionSource::MILPSolutionPool
			|| primalSol.sourceType == E_PrimalSolutionSource::NLPFixedIntegers/*
			 || primalSol.sourceType == E_PrimalSolutionSource::LazyConstraintCallback*/)
	{
		isLinConstrFulfilled = true;
		mostDev = primalSol.maxDevatingConstraint;
	}
	else
	{
		bool isRounded = false;

		auto discreteVarIndexes = processInfo->originalProblem->getDiscreteVariableIndices();

		std::vector<double> ptRounded(tmpPoint);

		for (int i = 0; i < discreteVarIndexes.size(); i++)
		{
			int idx = discreteVarIndexes.at(i);
			double rounded = round(tmpPoint.at(idx));

			if (abs(rounded - tmpPoint.at(idx)) > 0)
			{
				ptRounded.at(idx) = rounded;
				isRounded = true;
			}
		}

		if (isRounded)
		{
			tmpPoint = ptRounded;
			tmpObjVal = processInfo->originalProblem->calculateOriginalObjectiveValue(ptRounded);

			if (processInfo->originalProblem->isObjectiveFunctionNonlinear())
			{
				tmpPoint.at(processInfo->originalProblem->getNonlinearObjectiveVariableIdx()) = tmpObjVal;
			}

		}

		isLinConstrFulfilled = processInfo->originalProblem->isLinearConstraintsFulfilledInPoint(tmpPoint,
				settings->getDoubleSetting("PrimalBoundLinearTolerance", "PrimalBound"));
	}

	isNonLinConstrFulfilled = processInfo->originalProblem->isConstraintsFulfilledInPoint(tmpPoint,
			settings->getDoubleSetting("PrimalBoundNonlinearTolerance", "PrimalBound"));
	/*if (processInfo->originalProblem->isObjectiveFunctionNonlinear())
	 {

	 }*/

	/*if (primalSol.sourceType == E_PrimalSolutionSource::ObjectiveConstraint)
	 {
	 std::cout << "Objective constraint value " << tmpObjVal << " and error " << mostDev.value << std::endl;
	 }*/

	bool isMostDevConstrNonlinear = processInfo->originalProblem->isConstraintNonlinear(mostDev.idx);

	if (primalSol.sourceType != E_PrimalSolutionSource::MILPSolutionPool && isMostDevConstrNonlinear
			&& ((mostDev.value > 0
					&& mostDev.value < 10000 * settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"))
					|| primalSol.sourceType == E_PrimalSolutionSource::ObjectiveConstraint)) // Add point as hyperplane
	{
		Hyperplane hyperplane;
		hyperplane.sourceConstraintIndex = mostDev.idx;
		hyperplane.generatedPoint = tmpPoint;
		hyperplane.source = E_HyperplaneSource::PrimalSolutionSearch;

		processInfo->hyperplaneWaitingList.push_back(hyperplane);

		HPadded = '*';

	}

	bool updatePrimal = isLinConstrFulfilled && isNonLinConstrFulfilled;

	/*std::cout << "Fulfilled linear: " << (isLinConstrFulfilled == true) << std::endl;
	 std::cout << "Fulfilled nonlinear: " << (isNonLinConstrFulfilled == true) << " " << mostDev.value << std::endl;
	 std::cout << std::endl;
	 */
	updatePrimal = updatePrimal
			&& ((isMinimization && tmpObjVal < processInfo->currentObjectiveBounds.second)
					|| (!isMinimization && tmpObjVal > processInfo->currentObjectiveBounds.second));

	//if (((isMinimization && tmpObjVal < processInfo->currentObjectiveBounds.second)
	//		|| (!isMinimization && tmpObjVal > processInfo->currentObjectiveBounds.second)))
	if (updatePrimal)
	{
		char HPobjadded = ' ';

		if (settings->getBoolSetting("UsePrimalObjectiveCut", "MILP")
				&& processInfo->originalProblem->isObjectiveFunctionNonlinear())
		{
			auto objConstrVal = processInfo->originalProblem->calculateConstraintFunctionValue(-1, primalSol.point)
					- primalSol.point.back();

			if (objConstrVal < 0)
			{

				Hyperplane hyperplane;
				hyperplane.sourceConstraintIndex = -1;
				hyperplane.generatedPoint = tmpPoint;
				hyperplane.source = E_HyperplaneSource::PrimalSolutionSearchInteriorObjective;

				processInfo->hyperplaneWaitingList.push_back(hyperplane);

				HPobjadded = '#';
			}
		}

		processInfo->currentObjectiveBounds.second = tmpObjVal;

		auto tmpLine = boost::format("    New primal bound %1% with dev. %2% (%3%) %4% %5%") % tmpObjVal % mostDev.value
				% sourceDesc % HPadded % HPobjadded;

		processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;

		primalSol.objValue = tmpObjVal;
		primalSol.point = tmpPoint;
		primalSol.maxDevatingConstraint = mostDev;

		processInfo->primalSolutions.push_back(primalSol);

		processInfo->primalSolution = tmpPoint;

		if (settings->getBoolSetting("AddPrimalBoundAsInteriorPoint", "Algorithm") && mostDev.value < -1e-11)
		{
			InteriorPoint tmpIP;

			tmpIP.point = tmpPoint;

			processInfo->logger.message(1) << "    Primal solution point added as interior point." << CoinMessageEol;

			if (processInfo->interiorPts.size() == processInfo->numOriginalInteriorPoints)
			{
				processInfo->interiorPts.push_back(tmpIP);
			}
			else
			{
				processInfo->interiorPts.back() = tmpIP;
			}

			/*UtilityFunctions::displayVector(processInfo->interiorPts.at(0).point);
			 UtilityFunctions::displayVector(processInfo->interiorPts.at(1).point);*/

		}
		return (true);
	}

	/*auto tmpLine = boost::format("    No new primal bound %1% with dev. %2% (%3%) %4%") % tmpObjVal % mostDev.value
	 % sourceDesc % HPadded;

	 processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;
	 */
	return (false);
}

