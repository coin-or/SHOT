#include "OptProblemOriginalQuadraticObjective.h"

OptProblemOriginalQuadraticObjective::OptProblemOriginalQuadraticObjective()
{
}

OptProblemOriginalQuadraticObjective::~OptProblemOriginalQuadraticObjective()
{
}

bool OptProblemOriginalQuadraticObjective::setProblem(OSInstance *instance)
{
	this->setObjectiveFunctionType(E_ObjectiveFunctionType::Quadratic);
	this->setProblemInstance(instance);
	this->setTypeOfObjectiveMinimize(instance->instanceData->objectives->obj[0]->maxOrMin == "min");
	this->setObjectiveFunctionNonlinear(false);
	this->setNonlinearConstraintIndexes();

	if (this->getNonlinearConstraintIndexes().size() == 0)
	{
		Settings::getInstance().updateSetting("IterLimitLP", "Algorithm", 0);
		Settings::getInstance().updateSetting("MILPSolLimitInitial", "MILP", 1000);
	}

	ProcessInfo::getInstance().setOriginalProblem(this);

	this->setVariableBoundsTightened(std::vector<bool>(getProblemInstance()->getVariableNumber(), false));

	instance->getJacobianSparsityPattern();

	return true;
}
