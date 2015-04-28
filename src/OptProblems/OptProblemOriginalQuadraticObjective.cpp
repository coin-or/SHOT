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
		settings->updateSetting("IterLimitLP", "Algorithm", 0);
		settings->updateSetting("MILPSolLimitInitial", "MILP", 1000);
	}

	processInfo->setOriginalProblem(this);
	
	instance->getJacobianSparsityPattern();

	return true;
}

/*
IndexValuePair OptProblemOriginalQuadraticObjective::getMostDeviatingConstraint(std::vector<double> point)
{
	IndexValuePair valpair;

	std::vector<int> idxNLCs = this->getNonlinearConstraintIndexes();

	if (idxNLCs.size() == 0)	//Only a quadratic objective function
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
}*/

/*
void OptProblemOriginalQuadraticObjective::setQuadraticConstraintIndexes()
{
	std::vector<bool> isQuadratic(this->getProblemInstance()->getConstraintNumber(), false);

	int numNonlinExpr = this->getProblemInstance()->instanceData->nonlinearExpressions == NULL ? 0 : this->getProblemInstance()->getNumberOfNonlinearExpressions();
	int numQuadTerms = this->getProblemInstance()->instanceData->quadraticCoefficients == NULL ? 0 : this->getProblemInstance()->getNumberOfQuadraticTerms();

	for (int i = 0; i < numQuadTerms; i++)
	{
		int tmpIndex = this->getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

		if (tmpIndex != -1)
		{
			isQuadratic.at(tmpIndex) = true;
		}
	}

	for (int i = 0; i < numNonlinExpr; i++)
	{
		int tmpIndex = this->getProblemInstance()->instanceData->nonlinearExpressions->nl[i]->idx;

		if (tmpIndex != -1)
		{
			isQuadratic.at(tmpIndex) = false;
		}
	}

	std::vector<int> quadraticIndexes;

	for (int i = 0; i < isQuadratic.size(); i++)
	{
		if (isQuadratic.at(i))	quadraticIndexes.push_back(i);
	}

	setQuadraticConstraints(quadraticIndexes);
}


void OptProblemOriginalQuadraticObjective::setNonlinearConstraintIndexes()
{
	std::vector<bool> isNonlinear(this->getProblemInstance()->getConstraintNumber(), false);
	int numNonlinExpr = this->getProblemInstance()->instanceData->nonlinearExpressions == NULL ? 0 : this->getProblemInstance()->getNumberOfNonlinearExpressions();
	int numQuadTerms = this->getProblemInstance()->instanceData->quadraticCoefficients == NULL ? 0 : this->getProblemInstance()->getNumberOfQuadraticTerms();

	for (int i = 0; i < numNonlinExpr; i++)
	{
		int tmpIndex = this->getProblemInstance()->instanceData->nonlinearExpressions->nl[i]->idx;

		if (tmpIndex != -1)
		{
			isNonlinear.at(tmpIndex) = true;
		}
	}
	
	std::vector<int> NLCIndexes;

	for (int i = 0; i < isNonlinear.size(); i++)
	{
		if (isNonlinear.at(i))	NLCIndexes.push_back(i);
	}

	setNonlinearConstraints(NLCIndexes);
}*/
