#include "OptProblemOriginal.h"

OptProblemOriginal::OptProblemOriginal()
{

}

OptProblemOriginal::~OptProblemOriginal()
{

}

double OptProblemOriginal::calculateConstraintFunctionValue(int idx, std::vector<double> point)
{
	double tmpVal = 0.0;

	tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);
	ProcessInfo::getInstance().numFunctionEvals++;

	if (idx == -1)
	{
		ProcessInfo::getInstance().outputWarning("Objective function constraint should not be calculated here! ");
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
		ProcessInfo::getInstance().outputWarning(
				"Constraint with index " + to_string(idx) + " of type "
						+ to_string(getProblemInstance()->getConstraintTypes()[idx]) + " is not supported!");
	}

	return tmpVal;
}

SparseVector* OptProblemOriginal::calculateConstraintFunctionGradient(int idx, std::vector<double> point)
{
	SparseVector* tmpVector;
	tmpVector = getProblemInstance()->calculateConstraintFunctionGradient(&point.at(0), idx, true);
	ProcessInfo::getInstance().numGradientEvals++;

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
