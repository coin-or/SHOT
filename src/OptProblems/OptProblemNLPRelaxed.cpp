#include "OptProblemNLPRelaxed.h"

OptProblemNLPRelaxed::OptProblemNLPRelaxed()
{
	//problemInstance = NULL;

}

OptProblemNLPRelaxed::~OptProblemNLPRelaxed()
{
	//delete problemInstance;
}

void OptProblemNLPRelaxed::reformulate(OSInstance *originalInstance)
{
	OSInstance *newInstance = NULL;
	newInstance = new OSInstance();

	this->setObjectiveFunctionNonlinear(isConstraintNonlinear(originalInstance, -1));
	this->setTypeOfObjectiveMinimize(originalInstance->instanceData->objectives->obj[0]->maxOrMin == "min");

	this->copyVariables(originalInstance, newInstance, true);

	this->copyObjectiveFunction(originalInstance, newInstance);

	this->copyConstraints(originalInstance, newInstance);

	this->copyLinearTerms(originalInstance, newInstance);

	this->copyQuadraticTerms(originalInstance, newInstance);

	this->copyNonlinearExpressions(originalInstance, newInstance);

	this->setProblemInstance(newInstance);

	this->setNonlinearConstraintIndexes();

	if (this->isObjectiveFunctionNonlinear())
	{
		setNonlinearObjectiveConstraintIdx(-1);	// Sets a virtual constraint

		setNonlinearObjectiveVariableIdx(originalInstance->getVariableNumber());
	}

	newInstance->getJacobianSparsityPattern();

}

IndexValuePair OptProblemNLPRelaxed::getMostDeviatingConstraint(std::vector<double> point)
{
	IndexValuePair valpair;

	std::vector<int> idxNLCs = this->getNonlinearOrQuadraticConstraintIndexes();

	if (idxNLCs.size() == 0)	//Only a quadratic objective function and quadratic constraints
	{
		valpair.idx = -1;
		valpair.value = 0.0;
	}
	else
	{
		std::vector<double> constrDevs(idxNLCs.size());

		for (int i = 0; i < idxNLCs.size(); i++)
		{
			constrDevs.at(i) = calculateConstraintFunctionValue(idxNLCs.at(i), point);
		}

		auto biggest = std::max_element(std::begin(constrDevs), std::end(constrDevs));
		valpair.idx = idxNLCs.at(std::distance(std::begin(constrDevs), biggest));
		valpair.value = *biggest;
	}

	return valpair;
}

bool OptProblemNLPRelaxed::isConstraintsFulfilledInPoint(std::vector<double> point, double eps)
{
	std::vector<int> idxNLCs = this->getNonlinearOrQuadraticConstraintIndexes();

	for (int i = 0; i < getNumberOfNonlinearConstraints(); i++)
	{
		double tmpVal = calculateConstraintFunctionValue(idxNLCs.at(i), point);
		if (tmpVal > eps) return false;
	}

	return true;
}
