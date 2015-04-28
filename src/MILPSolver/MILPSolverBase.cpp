#include "MILPSolverBase.h"

MILPSolverBase::MILPSolverBase()
{
}

MILPSolverBase::~MILPSolverBase()
{
}

void MILPSolverBase::startTimer()
{
	if (discreteVariablesActivated) processInfo->startTimer("MILP");
	else processInfo->startTimer("LP");
}

void MILPSolverBase::stopTimer()
{
	if (discreteVariablesActivated) processInfo->stopTimer("MILP");
	else processInfo->stopTimer("LP");
}

bool MILPSolverBase::getDiscreteVariableStatus()
{
	return (discreteVariablesActivated);
}
