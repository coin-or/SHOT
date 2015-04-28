#include "MILPSolutionLimitStrategyUnlimited.h"

MILPSolutionLimitStrategyUnlimited::MILPSolutionLimitStrategyUnlimited()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	//MILPSolver = solver;

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
	return processInfo->MILPSolver->getSolutionLimit();
}

int MILPSolutionLimitStrategyUnlimited::getInitialLimit()
{
	auto tmpVal = processInfo->MILPSolver->getSolutionLimit();
	return tmpVal;
}
