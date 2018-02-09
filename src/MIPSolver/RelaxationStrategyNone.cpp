#include "RelaxationStrategyNone.h"

RelaxationStrategyNone::RelaxationStrategyNone(IMIPSolver *MIPSolver)
{
	this->MIPSolver = MIPSolver;
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
	ProcessInfo::getInstance().getCurrentIteration()->type = E_IterationProblemType::MIP;
}

void RelaxationStrategyNone::setActive()
{

}

void RelaxationStrategyNone::setInactive()
{

}

E_IterationProblemType RelaxationStrategyNone::getProblemType()
{
	if (MIPSolver->getDiscreteVariableStatus()) return E_IterationProblemType::MIP;
	else return E_IterationProblemType::Relaxed;
}