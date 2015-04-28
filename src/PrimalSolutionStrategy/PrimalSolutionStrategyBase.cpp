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

	if (primalSol.sourceType != E_PrimalSolutionSource::MILPSolutionPool)
	{
		bool isRounded = false;

		auto discreteVarIndexes = processInfo->originalProblem->getDiscreteVariableIndices();

		std::vector<double> ptRounded(primalSol.point);

		for (int i = 0; i < discreteVarIndexes.size(); i++)
		{
			int idx = discreteVarIndexes.at(i);
			double rounded = round(primalSol.point.at(idx));

			if (abs(rounded - primalSol.point.at(idx)) > 0)
			{
				ptRounded.at(idx) = rounded;
				isRounded = true;
			}
		}

		if (isRounded)
		{
			primalSol.point = ptRounded;
			primalSol.objValue = processInfo->originalProblem->calculateOriginalObjectiveValue(ptRounded);
		}
	}

	if (std::isnan(primalSol.objValue))
	{
		primalSol.objValue = processInfo->originalProblem->calculateOriginalObjectiveValue(primalSol.point);
	}

	auto tmpMaxDev = processInfo->originalProblem->getMostDeviatingAllConstraint(primalSol.point);

	if (processInfo->originalProblem->isObjectiveFunctionNonlinear())
	{
		primalSol.point.at(processInfo->originalProblem->getNonlinearObjectiveVariableIdx()) = primalSol.objValue;
	}

	bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();

	//auto tmpMostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(ptRounded);

	char HPadded = ' ';

	bool isLinConstrFulfilled = processInfo->originalProblem->isLinearConstraintsFulfilledInPoint(primalSol.point,
			0.000001);
	bool isNonLinConstrFulfilled = processInfo->originalProblem->isConstraintsFulfilledInPoint(primalSol.point,
			0.00001);

	if (((isMinimization && primalSol.objValue < processInfo->currentObjectiveBounds.second)
			|| (!isMinimization && primalSol.objValue > processInfo->currentObjectiveBounds.second))
			&& isLinConstrFulfilled && isNonLinConstrFulfilled)
	{
		processInfo->currentObjectiveBounds.second = primalSol.objValue;

		if (tmpMaxDev.value >= 0) // Add point as hyperplane
		{
			std::pair<int, std::vector<double>> tmpItem;
			tmpItem.first = tmpMaxDev.idx;
			tmpItem.second = primalSol.point;

			processInfo->hyperplaneWaitingList.push_back(tmpItem);

			HPadded = '*';

		}

		auto tmpLine = boost::format("    New primal bound %1% with dev. %2% (%3%) %4%") % primalSol.objValue
				% tmpMaxDev.value % sourceDesc % HPadded;

		processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;

		processInfo->primalSolutions.push_back(primalSol);

		return (true);
	}

	return (false);
}

