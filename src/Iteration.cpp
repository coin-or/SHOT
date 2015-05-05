#include "Iteration.h"

Iteration::Iteration()
{
}

Iteration::~Iteration()
{
}

bool Iteration::isMILP()
{

	return (this->type == E_IterationProblemType::MIP);
}

SolutionPoint Iteration::getSolutionPointWithSmallestDeviation()
{
	double tmpVal = -DBL_MAX;
	int tmpIdx = 0;

	for (int i = 0; i < solutionPoints.size(); i++)
	{
		if (solutionPoints.at(i).maxDeviation.value > tmpVal)
		{
			tmpIdx = i;
			tmpVal = solutionPoints.at(i).maxDeviation.value;
		}
	}

	return (solutionPoints.at(tmpIdx));
}
