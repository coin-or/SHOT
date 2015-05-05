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
		default:
			break;
	}

	if (primalSol.sourceType == E_PrimalSolutionSource::MILPSolutionPool)
	{
		isLinConstrFulfilled = true;
		mostDev = processInfo->originalProblem->getMostDeviatingConstraint(tmpPoint);
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

		isLinConstrFulfilled = processInfo->originalProblem->isLinearConstraintsFulfilledInPoint(tmpPoint, 0.000001);

		mostDev = processInfo->originalProblem->getMostDeviatingAllConstraint(primalSol.point);

	}

	isNonLinConstrFulfilled = processInfo->originalProblem->isConstraintsFulfilledInPoint(tmpPoint, 0.0001);

	if (std::isnan(primalSol.objValue))
	{
		tmpObjVal = processInfo->originalProblem->calculateOriginalObjectiveValue(primalSol.point);
	}

	/*if (processInfo->originalProblem->isObjectiveFunctionNonlinear())
	 {

	 }*/

	if (((isMinimization && tmpObjVal < processInfo->currentObjectiveBounds.second)
			|| (!isMinimization && tmpObjVal > processInfo->currentObjectiveBounds.second)) && isLinConstrFulfilled
			&& isNonLinConstrFulfilled)
	{
		if (mostDev.value >= 0) // Add point as hyperplane
		{
			processInfo->currentObjectiveBounds.second = tmpObjVal;
			std::pair<int, std::vector<double>> tmpItem;
			tmpItem.first = mostDev.idx;
			tmpItem.second = tmpPoint;
			processInfo->hyperplaneWaitingList.push_back(tmpItem);

			HPadded = '*';

		}
		else
		{
			processInfo->currentObjectiveBounds.second = tmpObjVal;
		}

		auto tmpLine = boost::format("    New primal bound %1% with dev. %2% (%3%) %4%") % tmpObjVal % mostDev.value
				% sourceDesc % HPadded;

		processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;

		primalSol.objValue = tmpObjVal;
		primalSol.point = tmpPoint;

		processInfo->primalSolutions.push_back(primalSol);

		processInfo->primalSolution = tmpPoint;

		return (true);
	}

	/*auto tmpLine = boost::format("    No new primal bound %1% with dev. %2% (%3%) %4%") % tmpObjVal % mostDev.value
	 % sourceDesc % HPadded;

	 processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;
	 */
	return (false);
}

