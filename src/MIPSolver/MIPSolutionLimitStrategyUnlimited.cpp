#include "MIPSolutionLimitStrategyUnlimited.h"

MIPSolutionLimitStrategyUnlimited::MIPSolutionLimitStrategyUnlimited(IMIPSolver *MIPSolver)
{
	this->MIPSolver = MIPSolver;
}

MIPSolutionLimitStrategyUnlimited::~MIPSolutionLimitStrategyUnlimited()
{
}

bool MIPSolutionLimitStrategyUnlimited::updateLimit()
{
	return false;
}

int MIPSolutionLimitStrategyUnlimited::getNewLimit()
{
	return MIPSolver->getSolutionLimit();
}

int MIPSolutionLimitStrategyUnlimited::getInitialLimit()
{
	auto tmpVal = MIPSolver->getSolutionLimit();
	return tmpVal;
}
