#include "OptProblemOriginalLinearObjective.h"

OptProblemOriginalLinearObjective::OptProblemOriginalLinearObjective()
{
}

OptProblemOriginalLinearObjective::~OptProblemOriginalLinearObjective()
{
}

bool OptProblemOriginalLinearObjective::setProblem(OSInstance *instance)
{
	this->setObjectiveFunctionType(E_ObjectiveFunctionType::Linear);
	this->setProblemInstance(instance);
	this->setTypeOfObjectiveMinimize(instance->instanceData->objectives->obj[0]->maxOrMin == "min");
	this->setObjectiveFunctionNonlinear(false);
	if (!isObjectiveFunctionNonlinear())
	{
		this->setNonlinearObjectiveConstraintIdx(-COIN_INT_MAX);
		this->setNonlinearObjectiveVariableIdx(-COIN_INT_MAX);
	}

	this->repairNonboundedObjectiveVariable(instance);
	this->setNonlinearConstraintIndexes();

	if (this->getNonlinearConstraintIndexes().size() == 0)
	{
		Settings::getInstance().updateSetting("Relaxation.IterationLimit", "Dual", 0);
		Settings::getInstance().updateSetting("MIP.SolutionLimit.Initial", "Dual", 1000);
	}

	ProcessInfo::getInstance().setOriginalProblem(this);

	this->setVariableBoundsTightened(std::vector<bool>(getProblemInstance()->getVariableNumber(), false));

	instance->getJacobianSparsityPattern();

	return true;
}
