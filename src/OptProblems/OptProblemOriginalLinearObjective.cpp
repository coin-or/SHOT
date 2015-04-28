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
	this->setObjectiveFunctionNonlinear(isConstraintNonlinear(-1));
	this->repairNonboundedObjectiveVariable(instance);
	this->setNonlinearConstraintIndexes();

	if (this->getNonlinearConstraintIndexes().size() == 0)
	{
		settings->updateSetting("IterLimitLP", "Algorithm", 0);
		settings->updateSetting("MILPSolLimitInitial", "MILP", 1000);
	}


	processInfo->setOriginalProblem(this);
	
	instance->getJacobianSparsityPattern();

	return true;
}
