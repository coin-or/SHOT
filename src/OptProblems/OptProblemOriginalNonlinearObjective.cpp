#include "OptProblemOriginalNonlinearObjective.h"

OptProblemOriginalNonlinearObjective::OptProblemOriginalNonlinearObjective()
{

}

OptProblemOriginalNonlinearObjective::~OptProblemOriginalNonlinearObjective()
{
}

bool OptProblemOriginalNonlinearObjective::setProblem(OSInstance *instance)
{
	this->setObjectiveFunctionType(E_ObjectiveFunctionType::GeneralNonlinear);
	this->setProblemInstance(instance);

	this->setTypeOfObjectiveMinimize(instance->instanceData->objectives->obj[0]->maxOrMin == "min");

	this->setObjectiveFunctionNonlinear(isConstraintNonlinear(-1));

	this->repairNonboundedObjectiveVariable(instance);

	processInfo->setOriginalProblem(this);

	this->setNonlinearConstraintIndexes();

	if (this->getNonlinearConstraintIndexes().size() == 0)
	{
		settings->updateSetting("IterLimitLP", "Algorithm", 0);
		settings->updateSetting("MILPSolLimitInitial", "MILP", 1000);
	}

	this->addedConstraintName = "objconstr";
	this->setNonlinearObjectiveConstraintIdx(getProblemInstance()->getConstraintNumber());

	this->addedObjectiveVariableName = "addobjvar";
	this->addedObjectiveVariableLowerBound = -settings->getDoubleSetting("NLPObjectiveBound", "NLP");
	//this->addedObjectiveVariableLowerBound = 117916.288;
	this->addedObjectiveVariableUpperBound = settings->getDoubleSetting("NLPObjectiveBound", "NLP");
	;
	this->setNonlinearObjectiveVariableIdx(getProblemInstance()->getVariableNumber());

	instance->getJacobianSparsityPattern();

	return true;
}

double OptProblemOriginalNonlinearObjective::calculateConstraintFunctionValue(int idx, std::vector<double> point)
{
	double tmpVal = 0.0;

	if (idx != -1 && idx != this->getNonlinearObjectiveConstraintIdx())	// Not the objective function
	{
		tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);

		if (getProblemInstance()->getConstraintTypes()[idx] == 'L')
		{
			tmpVal = tmpVal - getProblemInstance()->instanceData->constraints->con[idx]->ub; // -problemInstance->getConstraintConstants()[idx];
		}
		else if (getProblemInstance()->getConstraintTypes()[idx] == 'G')
		{
			tmpVal = -tmpVal + getProblemInstance()->instanceData->constraints->con[idx]->lb; // +problemInstance->getConstraintConstants()[idx];
			//std::cout << "Lin value is: "<< tmpVal << std::endl;
		}
		else if (getProblemInstance()->getConstraintTypes()[idx] == 'E')
		{
			tmpVal = tmpVal - getProblemInstance()->instanceData->constraints->con[idx]->lb; // +problemInstance->getConstraintConstants()[idx];
			//std::cout << "Lin value is: "<< tmpVal << std::endl;
		}
		/*
		 if (getProblemInstance()->getConstraintTypes()[idx] == 'L')
		 {
		 auto tmpUB = getProblemInstance()->instanceData->constraints->con[idx]->ub;
		 tmpVal = tmpVal - tmpUB; // -problemInstance->getConstraintConstants()[idx];
		 std::cout << "Lin value2 is: "<< tmpVal << std::endl;
		 }
		 else if (getProblemInstance()->getConstraintTypes()[idx] == 'G')
		 {
		 auto tmpLB = getProblemInstance()->instanceData->constraints->con[idx]->lb;
		 tmpVal = -tmpVal + tmpLB; // +problemInstance->getConstraintConstants()[idx];
		 //std::cout << "Lin value is: "<< tmpVal << std::endl;
		 }
		 else if (getProblemInstance()->getConstraintTypes()[idx] == 'E')
		 {
		 auto tmpLB = getProblemInstance()->instanceData->constraints->con[idx]->lb;
		 tmpVal = tmpVal -tmpLB;
		 //std::cout << "Lin value is: "<< tmpVal << std::endl;
		 }*/
		else
		{
			processInfo->logger.message(1) << "Constraint with index " << idx << " of type "
					<< getProblemInstance()->getConstraintTypes()[idx] << " is not supported! " << CoinMessageEol;
		}
	}
	else // The nonlinear objective function constraint
	{
		tmpVal = getProblemInstance()->calculateFunctionValue(-1, &point.at(0), true);

		//std::cout << "tmpval before: " << tmpVal << std::endl;

		tmpVal = tmpVal - point.at(this->getNonlinearObjectiveVariableIdx());

		//std::cout << "nonlin var index: " <<this->getNonlinearObjectiveVariableIdx() << std::endl;
		//std::cout << "point :" << point.at(this->getNonlinearObjectiveVariableIdx());
		//std::cout << "tmpval after: " << tmpVal << std::endl;
	}

	return tmpVal;
}

SparseVector* OptProblemOriginalNonlinearObjective::calculateConstraintFunctionGradient(int idx,
		std::vector<double> point)
{
	int number;
	SparseVector* tmpVector;

	if (idx == -1)
	{
		auto tmpArray = getProblemInstance()->calculateObjectiveFunctionGradient(&point.at(0), -1, true);
		number = getProblemInstance()->getVariableNumber();
		tmpVector = new SparseVector(number);
		std::vector<int> tmpIndexes;
		std::vector<double> tmpValues;

		int nonZeroVals = 0;
		for (int i = 0; i < number; i++)
		{
			if (tmpArray[i] != 0)
			{
				tmpIndexes.push_back(i);
				tmpValues.push_back(tmpArray[i]);
				nonZeroVals++;
			}
		}

		tmpIndexes.push_back(this->getNonlinearObjectiveVariableIdx());
		tmpValues.push_back(-1);

		for (int i = 0; i < nonZeroVals + 1; i++)
		{
			tmpVector->indexes[i] = tmpIndexes.at(i);
			tmpVector->values[i] = tmpValues.at(i);
		}

		delete tmpArray;

		return tmpVector;
	}
	else
	{
		tmpVector = getProblemInstance()->calculateConstraintFunctionGradient(&point.at(0), idx, true);

		/*if (processInfo->originalProblem->getProblemInstance()->getInstanceName() == "synthes1")
		 {
		 if (idx == 0)
		 {
		 tmpVector->indexes[0] = 0;
		 tmpVector->indexes[1] = 1;
		 tmpVector->indexes[2] = 2;

		 tmpVector->values[0] = 0.96 / (point.at(0) - point.at(1) + 1);
		 tmpVector->values[1] = -0.96 / (point.at(0) - point.at(1) + 1) + 0.8 / (point.at(1) + 1);
		 tmpVector->values[2] = -0.8;
		 }
		 else if (idx == 1)
		 {
		 tmpVector->indexes[0] = 0;
		 tmpVector->indexes[1] = 1;
		 tmpVector->indexes[2] = 2;

		 tmpVector->values[0] = 1.2 / (point.at(0) - point.at(1) + 1);
		 tmpVector->values[1] = -1.2 / (point.at(0) - point.at(1) + 1) + 1.0 / (point.at(1) + 1);
		 tmpVector->values[2] = -1.0;
		 tmpVector->values[5] = -2.0;
		 }
		 }
		 else if (processInfo->originalProblem->getProblemInstance()->getInstanceName() == "nvs12")
		 {


		 if (idx == 0)
		 {
		 tmpVector->indexes[0] = 0;
		 tmpVector->indexes[1] = 1;
		 tmpVector->indexes[2] = 2;
		 tmpVector->indexes[3] = 3;

		 tmpVector->values[0] = -18 * point.at(0) - 10 * point.at(1) - 6 * point.at(2) - 10 * point.at(3);
		 tmpVector->values[1] = -10 * point.at(0) - 16 * point.at(1) - 10 * point.at(2) - 6 * point.at(3);
		 tmpVector->values[2] = -6 * point.at(0) - 10 * point.at(1) - 10 * point.at(2) - 2 * point.at(3);
		 tmpVector->values[3] = -10 * point.at(0) - 6 * point.at(1) - 2 * point.at(2) - 14 * point.at(3);
		 }
		 else if (idx == 1)
		 {
		 tmpVector->indexes[0] = 0;
		 tmpVector->indexes[1] = 1;
		 tmpVector->indexes[2] = 2;
		 tmpVector->indexes[3] = 3;

		 tmpVector->values[0] = -12 * point.at(0) - 8 * point.at(1) - 2 * point.at(2) + 2 * point.at(3);
		 tmpVector->values[1] = -8 * point.at(0) - 12 * point.at(1) - 2 * point.at(2) + 10 * point.at(3);
		 tmpVector->values[2] = -2 * point.at(0) - 2 * point.at(1) - 8 * point.at(2);
		 tmpVector->values[3] = 2 * point.at(0) + 10 * point.at(1) - 16 * point.at(3);
		 }
		 else if (idx == 2)
		 {
		 tmpVector->indexes[0] = 0;
		 tmpVector->indexes[1] = 1;
		 tmpVector->indexes[2] = 2;
		 tmpVector->indexes[3] = 3;

		 tmpVector->values[0] = -18 * point.at(0) + 2 * point.at(1) + 4 * point.at(3);
		 tmpVector->values[1] = 2 * point.at(0) - 12 * point.at(1) + 2 * point.at(2) + 4 * point.at(3);
		 tmpVector->values[2] = 2 * point.at(1) - 16 * point.at(2) - 2 * point.at(3);
		 tmpVector->values[3] = 4 * point.at(0) + 4 * point.at(1) - 2 * point.at(2) - 12 * point.at(3);
		 }
		 else if (idx == 3)
		 {
		 tmpVector->indexes[0] = 0;
		 tmpVector->indexes[1] = 1;
		 tmpVector->indexes[2] = 2;
		 tmpVector->indexes[3] = 3;

		 tmpVector->values[0] = -16 * point.at(0) - 2 * point.at(1) - 2 * point.at(2) + 6 * point.at(3);
		 tmpVector->values[1] = -2 * point.at(0) - 8 * point.at(1) - 4 * point.at(2) + 2 * point.at(3);
		 tmpVector->values[2] = -2 * point.at(0) - 4 * point.at(1) - 18 * point.at(2) - 2 * point.at(3);
		 tmpVector->values[3] = 6 * point.at(0) + 2 * point.at(1) - 2 * point.at(2) - 14 * point.at(3);
		 }
		 }*/

		number = tmpVector->number;

		if (getProblemInstance()->getConstraintTypes()[idx] == 'G')
		{
			for (int i = 0; i < number; i++)
			{
				tmpVector->values[i] = -tmpVector->values[i];
			}
		}

		return tmpVector;
	}
}

// For all objectives except the additional nonlinear objective constraint
IndexValuePair OptProblemOriginalNonlinearObjective::getMostDeviatingAllConstraint(std::vector<double> point)
{

	IndexValuePair valpair;

	int numConstr = this->getNumberOfConstraints();

	std::vector<double> constrDevs(numConstr);

	for (int i = 0; i < numConstr; i++)
	{
		if (i == this->getNonlinearObjectiveConstraintIdx())
		{
			constrDevs.at(i) = 0.0;
		}
		else
		{
			constrDevs.at(i) = calculateConstraintFunctionValue(i, point);
		}
	}

	//UtilityFunctions::displayVector(constrDevs);

	auto biggest = std::max_element(std::begin(constrDevs), std::end(constrDevs));
	valpair.idx = std::distance(std::begin(constrDevs), biggest);
	valpair.value = *biggest;

	return valpair;
}

int OptProblemOriginalNonlinearObjective::getNumberOfNonlinearConstraints()
{
	/*	int ctr = 0;

	 std::vector<bool> isNonlinear(getProblemInstance()->getConstraintNumber(), false);

	 for (int i = 0; i < getProblemInstance()->getNumberOfNonlinearExpressions(); i++)
	 {
	 int tmpIndex = getProblemInstance()->instanceData->nonlinearExpressions->nl[i]->idx;

	 if (tmpIndex != -1) isNonlinear.at(tmpIndex) = true;
	 }


	 if (!settings->getBoolSetting("UseQuadraticProgramming", "Algorithm"))
	 {
	 for (int i = 0; i < getProblemInstance()->getNumberOfQuadraticTerms(); i++)
	 {
	 int tmpIndex = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

	 if (tmpIndex != -1) isNonlinear.at(tmpIndex) = true;
	 }
	 }

	 for (int i = 0; i < isNonlinear.size(); i++)
	 {
	 if (isNonlinear.at(i))	ctr++;
	 }*/

	int ctr = OptProblem::getNumberOfNonlinearConstraints();

	ctr++; //Nonlinear objective function constraint

	return ctr;
}

int OptProblemOriginalNonlinearObjective::getNumberOfConstraints()
{
	return getProblemInstance()->getConstraintNumber() + 1;
}

std::vector<std::string> OptProblemOriginalNonlinearObjective::getConstraintNames()
{
	std::string* tmpArray = getProblemInstance()->getConstraintNames();

	std::vector < std::string > tmpVector;

	for (int i = 0; i < getProblemInstance()->getConstraintNumber(); i++)
	{
		tmpVector.push_back(tmpArray[i]);
	}

	tmpVector.push_back(this->addedConstraintName);

	return tmpVector;
}

int OptProblemOriginalNonlinearObjective::getNumberOfVariables()
{
	return getProblemInstance()->getVariableNumber() + 1;
}

int OptProblemOriginalNonlinearObjective::getNumberOfRealVariables()
{
	return getProblemInstance()->getVariableNumber() - getProblemInstance()->getNumberOfBinaryVariables()
			- getProblemInstance()->getNumberOfIntegerVariables() + 1;
}

std::vector<std::string> OptProblemOriginalNonlinearObjective::getVariableNames()
{
	std::string* tmpArray = getProblemInstance()->getVariableNames();
	std::vector < std::string > tmpVector;

	for (int i = 0; i < getProblemInstance()->getVariableNumber(); i++)
	{
		tmpVector.push_back(tmpArray[i]);
	}

	tmpVector.push_back(this->addedObjectiveVariableName);

	return tmpVector;
}

std::vector<char> OptProblemOriginalNonlinearObjective::getVariableTypes()
{
	char* tmpArray = getProblemInstance()->getVariableTypes();

	std::vector<char> tmpVector;

	for (int i = 0; i < getProblemInstance()->getVariableNumber(); i++)
	{
		tmpVector.push_back(tmpArray[i]);
	}

	tmpVector.push_back('C');

	return tmpVector;
}

std::vector<double> OptProblemOriginalNonlinearObjective::getVariableLowerBounds()
{
	double* tmpArray = getProblemInstance()->getVariableLowerBounds();

	std::vector<double> tmpVector;

	for (int i = 0; i < getProblemInstance()->getVariableNumber(); i++)
	{
		tmpVector.push_back(tmpArray[i]);
	}

	tmpVector.push_back(this->addedObjectiveVariableLowerBound);

	return tmpVector;
}

std::vector<double> OptProblemOriginalNonlinearObjective::getVariableUpperBounds()
{
	double* tmpArray = getProblemInstance()->getVariableUpperBounds();

	std::vector<double> tmpVector;

	for (int i = 0; i < getProblemInstance()->getVariableNumber(); i++)
	{
		tmpVector.push_back(tmpArray[i]);
	}

	tmpVector.push_back(this->addedObjectiveVariableUpperBound);

	return tmpVector;
}

std::vector<std::pair<int, double>> OptProblemOriginalNonlinearObjective::getObjectiveFunctionVarCoeffPairs()
{
	std::vector<std::pair<int, double>> tmpVector;

	std::pair<int, double> tmpPair;

	tmpPair.first = getNonlinearObjectiveVariableIdx();
	tmpPair.second = 1.0;

	tmpVector.push_back(tmpPair);

	return tmpVector;
}

double OptProblemOriginalNonlinearObjective::getObjectiveConstant()
{
	return 0.0;
}

void OptProblemOriginalNonlinearObjective::setNonlinearConstraintIndexes()
{
	OptProblem::setNonlinearConstraintIndexes();

	auto NLCIndexes = getNonlinearConstraintIndexes();

	std::vector<int>::iterator it;
	it = NLCIndexes.begin();

	NLCIndexes.insert(it, -1);

	setNonlinearConstraints(NLCIndexes);
}
