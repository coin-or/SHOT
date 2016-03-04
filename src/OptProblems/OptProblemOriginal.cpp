#include "OptProblemOriginal.h"

OptProblemOriginal::OptProblemOriginal()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

OptProblemOriginal::~OptProblemOriginal()
{

}

double OptProblemOriginal::calculateConstraintFunctionValue(int idx, std::vector<double> point)
{
	double tmpVal = 0.0;

	tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);
	processInfo->numFunctionEvals++;

	if (idx == -1)
	{
		processInfo->logger.message(1) << "Objective function constraint should not be calculated here! "
				<< CoinMessageEol;
	}
	else if (getProblemInstance()->getConstraintTypes()[idx] == 'L')
	{
		auto tmpUB = getProblemInstance()->instanceData->constraints->con[idx]->ub;
		tmpVal = tmpVal - tmpUB;
	}
	else if (getProblemInstance()->getConstraintTypes()[idx] == 'G')
	{
		auto tmpLB = getProblemInstance()->instanceData->constraints->con[idx]->lb;
		tmpVal = -tmpVal + tmpLB;
	}
	else if (getProblemInstance()->getConstraintTypes()[idx] == 'E')
	{
		auto tmpUB = getProblemInstance()->instanceData->constraints->con[idx]->ub;
		tmpVal = tmpVal - tmpUB;
	}
	else
	{
		processInfo->logger.message(1) << "Constraint with index " << idx << " of type "
				<< getProblemInstance()->getConstraintTypes()[idx] << " is not supported! " << CoinMessageEol;
	}

	return tmpVal;
}

SparseVector* OptProblemOriginal::calculateConstraintFunctionGradient(int idx, std::vector<double> point)
{
	SparseVector* tmpVector;
	tmpVector = getProblemInstance()->calculateConstraintFunctionGradient(&point.at(0), idx, true);
	processInfo->numGradientEvals++;

	int number = tmpVector->number;

	if (getProblemInstance()->getConstraintTypes()[idx] == 'G')
	{
		for (int i = 0; i < number; i++)
		{
			tmpVector->values[i] = -tmpVector->values[i];
		}
	}

	return tmpVector;
}
