#include "OptProblemNLPSHOTMinimax.h"
#include "OSExpressionTree.h"
#include <vector>

OptProblemNLPSHOTMinimax::OptProblemNLPSHOTMinimax()
{
	//problemInstance = NULL;
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

OptProblemNLPSHOTMinimax::~OptProblemNLPSHOTMinimax()
{
	//delete problemInstance;
}

void OptProblemNLPSHOTMinimax::reformulate(OSInstance *originalInstance)
{
	OSInstance *newInstance = NULL;
	newInstance = new OSInstance();

	this->setObjectiveFunctionNonlinear(isConstraintNonlinear(originalInstance, -1));

	this->setTypeOfObjectiveMinimize(true);

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
		int tmpVal = originalInstance->instanceData->constraints->numberOfConstraints;
		setNonlinearObjectiveConstraintIdx(tmpVal);	// Sets a virtual constraint

		setNonlinearObjectiveVariableIdx(originalInstance->getVariableNumber());
	}

	newInstance->getJacobianSparsityPattern();
	newInstance->initForAlgDiff();
}

void OptProblemNLPSHOTMinimax::copyVariables(OSInstance *source, OSInstance *destination, bool integerRelaxed)
{
	int numVar = source->getVariableNumber();

	if (this->isObjectiveFunctionNonlinear())
	{
		destination->setVariableNumber(numVar + 2);
	}
	else
	{
		destination->setVariableNumber(numVar + 1);
	}

	auto varNames = source->getVariableNames();
	auto varTypes = source->getVariableTypes();
	auto varLBs = source->getVariableLowerBounds();
	auto varUBs = source->getVariableUpperBounds();

	//int numVar = source->instanceData->variables->numberOfVariables;

	if (destination->getVariableNumber() == 0)
	{
		destination->setVariableNumber(numVar);
	}

	if (integerRelaxed)
	{
		for (int i = 0; i < numVar; i++)
		{
			std::string name = varNames[i];
			double lb = varLBs[i];
			double ub = varUBs[i];
			char type = 'C';
			/*
			 if (lb < -1000000000000)
			 {
			 lb = -10000000000;
			 }

			 if (ub > 10000000000000)
			 {
			 ub = 10000000000;
			 }
			 */
			destination->addVariable(i, name, lb, ub, type);
		}
	}
	else
	{
		for (int i = 0; i < numVar; i++)
		{

			std::string name = varNames[i];
			double lb = varLBs[i];
			double ub = varUBs[i];
			char type = varTypes[i];

			destination->addVariable(i, name, lb, ub, type);

		}
	}

	double tmpObjBound = settings->getDoubleSetting("NLPObjectiveBound", "NLP");
	double tmpConstrFeas = settings->getDoubleSetting("InteriorPointFeasEps", "NLP");

	if (this->isObjectiveFunctionNonlinear())
	{
		destination->addVariable(numVar, "mu", -tmpObjBound, tmpObjBound, 'C');
		destination->addVariable(numVar + 1, "tempobjvar", -tmpObjBound, tmpObjBound, 'C');
	}
	else
	{
		destination->addVariable(numVar, "tempobjvar", -tmpObjBound, tmpObjBound, 'C');
	}

	destination->bVariablesModified = true;
}

void OptProblemNLPSHOTMinimax::copyObjectiveFunction(OSInstance *source, OSInstance *destination)
{
	int numVar = source->getVariableNumber();

	destination->setObjectiveNumber(1);

	SparseVector * newobjcoeff = new SparseVector(1);

	if (this->isObjectiveFunctionNonlinear())
	{
		newobjcoeff->indexes[0] = numVar + 1;
	}
	else
	{
		newobjcoeff->indexes[0] = numVar;
	}

	newobjcoeff->values[0] = 1;

	destination->addObjective(-1, "newobj", "min", 0.0, 1.0, newobjcoeff);
	delete newobjcoeff;

	destination->bObjectivesModified = true;
}

void OptProblemNLPSHOTMinimax::copyConstraints(OSInstance *source, OSInstance *destination)
{
	int numCon = source->getConstraintNumber();

	if (this->isObjectiveFunctionNonlinear())
	{
		destination->setConstraintNumber(numCon + 1);
	}
	else
	{
		destination->setConstraintNumber(numCon);
	}

	for (int i = 0; i < numCon; i++)
	{
		std::string name = source->instanceData->constraints->con[i]->name;
		double lb = source->instanceData->constraints->con[i]->lb;
		double ub = source->instanceData->constraints->con[i]->ub;
		double constant = source->instanceData->constraints->con[i]->constant;

		destination->addConstraint(i, name, lb, ub, constant);
	}

	if (this->isObjectiveFunctionNonlinear())
	{
		//double tmpObjBound = settings->getDoubleSetting("NLPObjectiveBound", "NLP");
		destination->addConstraint(numCon, "objconstr", -DBL_MAX, -source->instanceData->objectives->obj[0]->constant,
				0.0);
	}

	destination->bConstraintsModified = true;
}

void OptProblemNLPSHOTMinimax::copyLinearTerms(OSInstance *source, OSInstance *destination)
{
	int row_nonz = 0;
	int obj_nonz = 0;
	int varIdx = 0;

	int rowNum = source->getConstraintNumber();
	int varNum = source->getVariableNumber();
	int elemNum = source->getLinearConstraintCoefficientNumber();

	int nonlinConstrs = getNumberOfNonlinearConstraints(source);

	SparseMatrix *m_linearConstraintCoefficientsInRowMajor = source->getLinearConstraintCoefficientsInRowMajor();

	/*int *rowIndices, *colIndices;
	 double * elements;*/

	int numTotalElements = elemNum + nonlinConstrs;

	if (this->isObjectiveFunctionNonlinear())
	{
		numTotalElements = numTotalElements + 1 + source->instanceData->objectives->obj[0]->numberOfObjCoef;
	}

	std::vector<int> rowIndices;
	std::vector<int> colIndices;
	std::vector<double> elements;

	rowIndices.reserve(numTotalElements);
	colIndices.reserve(numTotalElements);
	elements.reserve(numTotalElements);

	for (int rowIdx = 0; rowIdx < rowNum; rowIdx++)
	{
		if (m_linearConstraintCoefficientsInRowMajor != NULL && m_linearConstraintCoefficientsInRowMajor->valueSize > 0)
		{
			row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1]
					- m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];

			for (int j = 0; j < row_nonz; j++)
			{
				varIdx =
						m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx]
								+ j];

				double tmpVal =
						m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx]
								+ j];

				rowIndices.push_back(rowIdx);
				colIndices.push_back(
						m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx]
								+ j]);
				elements.push_back(
						m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx]
								+ j]);
			}
		}
		// Inserts the objective function variable into nonlinear constraints
		if (isConstraintNonlinear(source, rowIdx))
		{
			rowIndices.push_back(rowIdx);

			if (this->isObjectiveFunctionNonlinear())
			{
				colIndices.push_back(varNum + 1);
			}
			else
			{
				colIndices.push_back(varNum);
			}

			if (source->getConstraintTypes()[rowIdx] == 'L')
			{
				elements.push_back(-1.0);
			}
			else if (source->getConstraintTypes()[rowIdx] == 'G')
			{
				elements.push_back(1.0);
			}

			std::cout << "Row: " << rowIdx << std::endl;
		}
	}

	if (this->isObjectiveFunctionNonlinear())
	{
		auto objCoeffs = source->getObjectiveCoefficients()[0];
		int numObjCoeffs = source->getObjectiveCoefficientNumbers()[0];

		for (int i = 0; i < numObjCoeffs; i++)
		{
			rowIndices.push_back(rowNum);
			colIndices.push_back(objCoeffs->indexes[i]);
			elements.push_back(objCoeffs->values[i]);
		}

		rowIndices.push_back(rowNum);
		colIndices.push_back(varNum);
		elements.push_back(-1.0);

		rowIndices.push_back(rowNum);
		colIndices.push_back(varNum + 1);
		elements.push_back(-1.0);

		std::cout << "Row obj: " << rowNum << std::endl;
	}

	CoinPackedMatrix *matrix = new CoinPackedMatrix(false, &rowIndices.at(0), &colIndices.at(0), &elements.at(0),
			numTotalElements);

	int numnonz = matrix->getNumElements();
	int valuesBegin = 0;
	int valuesEnd = numnonz - 1;
	int startsBegin = 0;
	int indexesBegin = 0;
	int indexesEnd = numnonz - 1;

	int startsEnd = this->isObjectiveFunctionNonlinear() ? varNum + 1 : varNum;
	//int startsEnd = this->isObjectiveFunctionNonlinear() ? varNum : varNum - 1;

	auto tmp = const_cast<int*>(matrix->getVectorStarts());

	destination->setLinearConstraintCoefficients(numnonz, matrix->isColOrdered(),
			const_cast<double*>(matrix->getElements()), valuesBegin, valuesEnd, const_cast<int*>(matrix->getIndices()),
			indexesBegin, indexesEnd, const_cast<int*>(matrix->getVectorStarts()), startsBegin, startsEnd);

	//destination->bConstraintsModified = true;

}

void OptProblemNLPSHOTMinimax::copyQuadraticTerms(OSInstance *source, OSInstance *destination)
{
	int numQuadTerms = source->getNumberOfQuadraticTerms();

	if (numQuadTerms > 0)
	{
		auto quadTerms = source->getQuadraticTerms();
		auto varRowIndexes = quadTerms->rowIndexes;

		std::vector<int> varOneIndexes;
		std::vector<int> varTwoIndexes;
		std::vector<double> coefficients;
		std::vector<int> rowIndexes;

		varOneIndexes.reserve(numQuadTerms);
		varTwoIndexes.reserve(numQuadTerms);
		coefficients.reserve(numQuadTerms);
		rowIndexes.reserve(numQuadTerms);

		for (int i = 0; i < numQuadTerms; i++)
		{
			varOneIndexes.push_back(quadTerms->varOneIndexes[i]);
			varTwoIndexes.push_back(quadTerms->varTwoIndexes[i]);
			coefficients.push_back(quadTerms->coefficients[i]);

			if (varRowIndexes[i] != -1)
			{
				rowIndexes.push_back(quadTerms->rowIndexes[i]);
			}
			else
			{
				rowIndexes.push_back(source->getConstraintNumber());
			}
		}

		if (varOneIndexes.size() > 0)
		{
			destination->setQuadraticCoefficients(varOneIndexes.size(), &rowIndexes.at(0), &varOneIndexes.at(0),
					&varTwoIndexes.at(0), &coefficients.at(0), 0, varOneIndexes.size() - 1);
		}
	}
}

void OptProblemNLPSHOTMinimax::copyNonlinearExpressions(OSInstance *source, OSInstance *destination)
{
	int numNonlinearExpressions = source->getNumberOfNonlinearExpressions();
	destination->instanceData->nonlinearExpressions = new NonlinearExpressions();
	destination->instanceData->nonlinearExpressions->numberOfNonlinearExpressions = numNonlinearExpressions;

	if (numNonlinearExpressions > 0)
	{
		//destination->instanceData->nonlinearExpressions = new NonlinearExpressions();
		destination->instanceData->nonlinearExpressions->nl = new Nl*[numNonlinearExpressions];

		for (int i = 0; i < numNonlinearExpressions; i++)
		{
			int rowIdx = source->instanceData->nonlinearExpressions->nl[i]->idx;

			//int valsAdded = 0;

			auto nlNodeVec = source->getNonlinearExpressionTreeInPrefix(rowIdx);

			destination->instanceData->nonlinearExpressions->nl[i] = new Nl();
			destination->instanceData->nonlinearExpressions->nl[i]->osExpressionTree = new ScalarExpressionTree(
					*source->instanceData->nonlinearExpressions->nl[i]->osExpressionTree);
			//auto tmp = ((OSnLNode*)nlNodeVec[0])->createExpressionTreeFromPrefix(nlNodeVec);
			//destination->instanceData->nonlinearExpressions->nl[i]->osExpressionTree->m_treeRoot = tmp;

			if (rowIdx != -1)
			{
				destination->instanceData->nonlinearExpressions->nl[i]->idx = rowIdx;
			}
			else
			{
				destination->instanceData->nonlinearExpressions->nl[i]->idx = source->getConstraintNumber();
			}

			nlNodeVec.clear();
			//valsAdded++;

		}
	}
}

double OptProblemNLPSHOTMinimax::calculateConstraintFunctionValue(int idx, std::vector<double> point)
{
	double tmpVal;

	tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);
	processInfo->numFunctionEvals++;

	if (getProblemInstance()->getConstraintTypes()[idx] == 'L')
	{
		tmpVal = tmpVal - getProblemInstance()->instanceData->constraints->con[idx]->ub; // -problemInstance->getConstraintConstants()[idx];
		//std::cout << "Lin value is: "<< tmpVal << std::endl;
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

	return tmpVal;
}

SparseVector* OptProblemNLPSHOTMinimax::calculateConstraintFunctionGradient(int idx, std::vector<double> point)
{
	auto tmpVector = getProblemInstance()->calculateConstraintFunctionGradient(&point.at(0), idx, true);
	processInfo->numGradientEvals++;
	/*if (getProblemInstance()->getConstraintTypes()[idx] == 'L')
	 {
	 }
	 else*/
	if (getProblemInstance()->getConstraintTypes()[idx] == 'G')
	{
		for (int i = 0; i < tmpVector->number; i++)
		{
			tmpVector->values[i] = -tmpVector->values[i];
		}
	}
	return (tmpVector);
}

IndexValuePair OptProblemNLPSHOTMinimax::getMostDeviatingConstraint(std::vector<double> point)
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

vector<IndexValuePair> OptProblemNLPSHOTMinimax::getMostDeviatingConstraints(std::vector<double> point,
		double tolerance)
{
	vector < IndexValuePair > valpairs;

	std::vector<int> idxNLCs = this->getNonlinearOrQuadraticConstraintIndexes();

	if (idxNLCs.size() == 0)	//Only a quadratic objective function and quadratic constraints
	{
		IndexValuePair valpair;
		valpair.idx = -1;
		valpair.value = 0.0;

		valpairs.push_back(valpair);
	}
	else
	{
		if (tolerance < 0) tolerance = 0;
		if (tolerance > 1) tolerance = 1;

		std::vector<double> constrDevs(idxNLCs.size());

		for (int i = 0; i < idxNLCs.size(); i++)
		{
			constrDevs.at(i) = calculateConstraintFunctionValue(idxNLCs.at(i), point);
		}

		IndexValuePair valpair;
		auto biggest = std::max_element(std::begin(constrDevs), std::end(constrDevs));
		valpair.idx = idxNLCs.at(std::distance(std::begin(constrDevs), biggest));
		valpair.value = *biggest;
		valpairs.push_back(valpair);

		double compVal;
		if (valpair.value >= 0)
		{
			compVal = valpair.value * (1 - tolerance);
		}
		else
		{
			compVal = valpair.value * (1 + tolerance);
		}

		for (int i = 0; i < idxNLCs.size(); i++)
		{
			if (idxNLCs.at(i) != valpair.idx)
			{
				if (constrDevs.at(i) >= compVal)
				{
					IndexValuePair tmpPair
					{ idxNLCs.at(i), constrDevs.at(i) };

					valpairs.push_back(tmpPair);
				}
			}
		}

	}

	return valpairs;
}

bool OptProblemNLPSHOTMinimax::isConstraintsFulfilledInPoint(std::vector<double> point, double eps)
{
	std::vector<int> idxNLCs = this->getNonlinearOrQuadraticConstraintIndexes();

	for (int i = 0; i < getNumberOfNonlinearConstraints(); i++)
	{
		double tmpVal = calculateConstraintFunctionValue(idxNLCs.at(i), point);
		if (tmpVal > eps) return false;
	}

	return true;
}
