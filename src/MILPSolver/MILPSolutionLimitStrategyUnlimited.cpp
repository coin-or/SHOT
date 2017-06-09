#include "MILPSolutionLimitStrategyUnlimited.h"

MILPSolutionLimitStrategyUnlimited::MILPSolutionLimitStrategyUnlimited(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

MILPSolutionLimitStrategyUnlimited::~MILPSolutionLimitStrategyUnlimited()
{
}

bool MILPSolutionLimitStrategyUnlimited::updateLimit()
{
	return false;
}

int MILPSolutionLimitStrategyUnlimited::getNewLimit()
{
	return MILPSolver->getSolutionLimit();
}

int MILPSolutionLimitStrategyUnlimited::getInitialLimit()
{
	auto tmpVal = MILPSolver->getSolutionLimit();
	return tmpVal;
}
