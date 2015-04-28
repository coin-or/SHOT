#include "RelaxationStrategyNone.h"

RelaxationStrategyNone::RelaxationStrategyNone()
{
	//relaxationActive = false;

	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

RelaxationStrategyNone::~RelaxationStrategyNone()
{
}

void RelaxationStrategyNone::setInitial()
{
	setInactive();
}

void RelaxationStrategyNone::executeStrategy()
{
	processInfo->getCurrentIteration()->type = E_IterationProblemType::MIP;
}

void RelaxationStrategyNone::setActive()
{

}

void RelaxationStrategyNone::setInactive()
{

}

E_IterationProblemType RelaxationStrategyNone::getProblemType()
{
	if (processInfo->MILPSolver->getDiscreteVariableStatus()) return E_IterationProblemType::MIP;
	else return E_IterationProblemType::Relaxed;
}
