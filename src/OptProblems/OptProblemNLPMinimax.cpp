#include "OptProblemNLPMinimax.h"
#include "OSExpressionTree.h"
#include <vector>

OptProblemNLPMinimax::OptProblemNLPMinimax()
{
	//problemInstance = NULL;
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

OptProblemNLPMinimax::~OptProblemNLPMinimax()
{
	//delete problemInstance;
}

void OptProblemNLPMinimax::reformulate(OSInstance *originalInstance)
{

	processInfo->logger.message(3) << "Starting minimax NLP problem definition" << CoinMessageEol;

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
		int tmpVal = originalInstance->instanceData->constraints->numberOfConstraints;
		setNonlinearObjectiveConstraintIdx(tmpVal);	// Sets a virtual constraint

		setNonlinearObjectiveVariableIdx(originalInstance->getVariableNumber());
	}

	newInstance->getJacobianSparsityPattern();
	newInstance->initForAlgDiff();

	processInfo->logger.message(3) << "Finished minimax NLP problem definition" << CoinMessageEol;
}

void OptProblemNLPMinimax::copyVariables(OSInstance *source, OSInstance *destination, bool integerRelaxed)
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

			if (lb < -1000000000000)
			{
				lb = -10000000000;
			}

			if (ub > 10000000000000)
			{
				ub = 10000000000;
			}

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

void OptProblemNLPMinimax::copyObjectiveFunction(OSInstance *source, OSInstance *destination)
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

void OptProblemNLPMinimax::copyConstraints(OSInstance *source, OSInstance *destination)
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

void OptProblemNLPMinimax::copyLinearTerms(OSInstance *source, OSInstance *destination)
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
		numTotalElements = numTotalElements + 2 + source->instanceData->objectives->obj[0]->numberOfObjCoef;
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

	auto tmp = const_cast<int*>(matrix->getVectorStarts());

	destination->setLinearConstraintCoefficients(numnonz, matrix->isColOrdered(),
			const_cast<double*>(matrix->getElements()), valuesBegin, valuesEnd, const_cast<int*>(matrix->getIndices()),
			indexesBegin, indexesEnd, const_cast<int*>(matrix->getVectorStarts()), startsBegin, startsEnd);

	//destination->bConstraintsModified = true;

}

void OptProblemNLPMinimax::copyQuadraticTerms(OSInstance *source, OSInstance *destination)
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

void OptProblemNLPMinimax::copyNonlinearExpressions(OSInstance *source, OSInstance *destination)
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

/*
 double OptProblemNLPMinimax::calculateConstraintFunctionValue(int idx, std::vector<double> point)
 {
 double tmpVal;
 if (idx != getNonlinearObjectiveConstraintIdx())	// Not the objective function
 {
 tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);

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
 }
 else
 {
 tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);
 tmpVal = tmpVal - getProblemInstance()->instanceData->constraints->con[idx]->ub; // -problemInstance->getConstraintConstants()[idx];
 std::cout << " hahaaa" << std::endl;
 }

 //else if (idx == -1)
 //{
 //	tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);
 //}

 return tmpVal;
 }*/

/*
 double OptProblemNLPMinimax::calculateConstraintFunctionValue(int idx, std::vector<double> point)
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
 else
 {
 processInfo->logger.message(1) << "Constraint with index " << idx << " of type "
 << getProblemInstance()->getConstraintTypes()[idx] << " is not supported! " << CoinMessageEol;
 }
 }
 else
 {
 tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);
 tmpVal = tmpVal - getProblemInstance()->instanceData->constraints->con[idx]->ub;
 }

 return tmpVal;
 }*/

/*
 double OptProblemNLPMinimax::calculateConstraintFunctionValue(int idx, std::vector<double> point)
 {
 double tmpVal = 0.0;

 int objConstr = getNonlinearObjectiveConstraintIdx();
 if (idx != objConstr)	// Not the objective function
 {
 tmpVal = getProblemInstance()->calculateFunctionValue(idx,&point[0], true);

 if (getProblemInstance()->getConstraintTypes()[idx] == 'L')
 {
 tmpVal = tmpVal; // -problemInstance->getConstraintConstants()[idx];
 }
 else if (getProblemInstance()->getConstraintTypes()[idx] == 'G')
 {
 tmpVal = -tmpVal; // +problemInstance->getConstraintConstants()[idx];
 }
 }
 else if (this->isObjectiveFunctionNonlinear())	// The nonlinear objective function constraint
 {
 //assert(point.size() == problemInstance->getVariableNumber() + 1);
 tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point[0], true);

 tmpVal = tmpVal;// -point[this->addedObjectiveVariableIdx1] - point[this->addedObjectiveVariableIdx2];
 }

 return tmpVal;
 }*/

/*void OptProblemNLPMinimax::reformulate(OSInstance *originalInstance)
 {
 std::vector<double> scalingFactor(originalInstance->getNumberOfNonlinearExpressionTreeIndexes(), 1.0);

 reformulate(originalInstance, scalingFactor)
 }*/

//void OptProblemNLPMinimax::reformulate(OSInstance *originalInstance)
//{
//	//originalInstance->addQTermsToExpressionTree();
//	//bool nonlinobjfunct = false;
//	int numVar = originalInstance->getVariableNumber();
//	auto varNames = originalInstance->getVariableNames();
//	auto varTypes = originalInstance->getVariableTypes();
//	auto varLBs = originalInstance->getVariableLowerBounds();
//	auto varUBs = originalInstance->getVariableUpperBounds();
//
//	int numCon = originalInstance->getConstraintNumber();
//	auto conNames = originalInstance->getConstraintNames();
//	auto conLBs = originalInstance->getConstraintLowerBounds();
//	auto conUBs = originalInstance->getConstraintUpperBounds();
//	auto conConsts = originalInstance->getConstraintConstants();
//
//	int nunNonlinearConstrs = originalInstance->getNumberOfNonlinearConstraints();
//
//	auto nonlinearConstrs = originalInstance->getNonlinearExpressions();
//	int nunLinearConstrs = originalInstance->getConstraintNumber() - nunNonlinearConstrs;
//
//	bool isObjMin = originalInstance->getObjectiveMaxOrMins()[0] == "min";
//
//	OSInstance *newinstance = NULL;
//	newinstance = new OSInstance();
//
//
//	newinstance->setVariableNumber(numVar+1);
//	newinstance->setConstraintNumber(numCon);
//
//	for (int i = 0; i < numVar; i++)
//	{
//		newinstance->addVariable(i, varNames[i], varLBs[i], varUBs[i], 'C');
//	}
//	double tmpObjBound = settings->getDoubleSetting("NLPObjectiveBound", "NLP");
//
//	newinstance->addVariable(numVar, "tempobjvar", -tmpObjBound, tmpObjBound, 'C');
//	newinstance->bVariablesModified = true;
//
//
//	for (int i = 0; i < numCon; i++)
//	{
//		newinstance->addConstraint(i,
//			originalInstance->instanceData->constraints->con[i]->name,
//			originalInstance->instanceData->constraints->con[i]->lb,
//			originalInstance->instanceData->constraints->con[i]->ub,
//			originalInstance->instanceData->constraints->con[i]->constant);
//	}
//
//	newinstance->bConstraintsModified = true;
//
//	newinstance->setObjectiveNumber(1);
//
//
//	auto originalObjectiveWeight = settings->getDoubleSetting("OriginalObjectiveWeight", "NLP");
//
//	if (originalObjectiveWeight != 0.0)
//	{
//		int numCoeff = originalInstance->instanceData->objectives->obj[0]->numberOfObjCoef;
//
//
//		auto oldobjcoeff = originalInstance->getObjectiveCoefficients()[0];
//		auto oldobjcoeffnum = originalInstance->getObjectiveCoefficientNumbers()[0];
//
//		SparseVector * newobjcoeff = new SparseVector(oldobjcoeffnum + 1);
//
//		for (int i = 0; i < oldobjcoeffnum; i++)
//		{
//			newobjcoeff->indexes[i] = oldobjcoeff->indexes[i];
//			newobjcoeff->values[i] = originalObjectiveWeight * oldobjcoeff->values[i];
//		}
//
//		newobjcoeff->indexes[oldobjcoeffnum] = numVar;
//		newobjcoeff->values[oldobjcoeffnum] = 1;
//
//		newinstance->addObjective(-1, "newobj", "min", 0.0, 1.0, newobjcoeff);
//		delete newobjcoeff;
//	}
//	else
//	{
//		SparseVector * newobjcoeff = new SparseVector(1);
//		newobjcoeff->indexes[0] = numVar;
//		newobjcoeff->values[0] = 1;
//
//		newinstance->addObjective(-1, "newobj", "min", 0.0, 1.0, newobjcoeff);
//		delete newobjcoeff;
//	}
//
//	newinstance->bObjectivesModified = true;
//
//	OSnLNodeVariable *nlNodeVariablePoint;
//	OSnLNodeNumber *nlNodeNumberPoint;
//	std::vector<OSnLNode*> nlNodeVec;
//	OSnLNode *nlNodePoint;
//
//	newinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions = nunNonlinearConstrs;
//
//	newinstance->instanceData->nonlinearExpressions->nl = new Nl*[nunNonlinearConstrs];
//
//	// Copies the linear constraints
//	int row_nonz = 0;
//	int obj_nonz = 0;
//	int varIdx = 0;
//	SparseMatrix *m_linearConstraintCoefficientsInRowMajor = originalInstance->getLinearConstraintCoefficientsInRowMajor();
//
//	CoinPackedMatrix *matrix = new CoinPackedMatrix(false, 0, 0);
//	matrix->setDimensions(0, numVar);
//
//	std::vector<double>rowLBs;
//	std::vector<double>rowUBs;
//
//	// Sets a boolean indicating nonlinear rows
//
//	std::vector<bool> isNonlinear(originalInstance->getConstraintNumber(), false);
//
//	for (int i = 0; i < originalInstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions; i++)
//	{
//		int tmpIndex = originalInstance->instanceData->nonlinearExpressions->nl[i]->idx;
//
//		isNonlinear[tmpIndex] = true;
//	}
//
//	for (int i = 0; i < originalInstance->instanceData->quadraticCoefficients->numberOfQuadraticTerms; i++)
//	{
//		int tmpIndex = originalInstance->instanceData->quadraticCoefficients->qTerm[i]->idx;
//
//		isNonlinear[tmpIndex] = true;
//	}
//
//
//	for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
//	{
//		if (originalInstance->instanceData->linearConstraintCoefficients != NULL &&
//			originalInstance->instanceData->linearConstraintCoefficients->numberOfValues > 0)
//		{
//			row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1] - m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];
//
//			std::vector<int> varIndexes;
//			std::vector<double> varElements;
//
//			CoinPackedVector row;
//
//			for (int j = 0; j < row_nonz; j++)
//			{
//				varIdx = m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
//				row.insert(varIdx, m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j]);
//			}
//
//			// Inserts the objective function variable into nonlinear constraints
//			if (isNonlinear[rowIdx])
//			{
//				row.insert(numVar, -1.0);
//			}
//
//			matrix->appendRow(row);
//
//			// Adds the bounds of the rows
//			rowLBs.push_back(originalInstance->instanceData->constraints->con[rowIdx]->lb);
//			rowUBs.push_back(originalInstance->instanceData->constraints->con[rowIdx]->ub);
//		}
//	}
//
//	int numnonz = originalInstance->instanceData->linearConstraintCoefficients->numberOfValues;
//
//	int valuesBegin = 0;
//	int valuesEnd = numnonz - 1;
//	int startsBegin = 0;
//	int indexesBegin = 0;
//	int indexesEnd = numnonz - 1;
//	int startsEnd = matrix->isColOrdered() ? numVar + 1 : numCon  ;
//	newinstance->setLinearConstraintCoefficients(numnonz, matrix->isColOrdered(),
//		const_cast<double*>(matrix->getElements()), valuesBegin, valuesEnd,
//		const_cast<int*>(matrix->getIndices()), indexesBegin, indexesEnd,
//		const_cast<int*>(matrix->getVectorStarts()), startsBegin, startsEnd);
//
//	int numQuadTerms = originalInstance->getNumberOfQuadraticTerms();
//
//	if (numQuadTerms > 0)
//	{
//		auto quadTerms = originalInstance->getQuadraticTerms();
//		auto varRowIndexes = quadTerms->rowIndexes;
//
//		for (int i = 0; i < numQuadTerms; i++)
//		{
//			if (varRowIndexes[i] == -1)
//			{
//				varRowIndexes[i] = numCon;
//			}
//		}
//
//		newinstance->setQuadraticTerms(numQuadTerms, varRowIndexes, quadTerms->varOneIndexes, quadTerms->varTwoIndexes, quadTerms->coefficients, 0, numQuadTerms - 1);
//	}
//
//	if (originalInstance->getNumberOfNonlinearExpressions() > 0)
//	{
//		auto constraintTypes = originalInstance->getConstraintTypes();
//
//		auto allTrees = originalInstance->getAllNonlinearExpressionTrees();
//
//		std::map<int, OSExpressionTree*>::iterator pos;
//
//		int k = 0;
//		for (pos = allTrees.begin(); pos != allTrees.end(); ++pos)
//		{
//			int tmpIdx = pos->first;
//			if (tmpIdx == -1)
//			{
//
//			}
//			else
//			{
//
//				nlNodeVec = originalInstance->getNonlinearExpressionTreeInPostfix(tmpIdx);
//
//				newinstance->instanceData->nonlinearExpressions->nl[k] = new Nl();
//
//				newinstance->instanceData->nonlinearExpressions->nl[k]->osExpressionTree = new OSExpressionTree();
//				newinstance->instanceData->nonlinearExpressions->nl[k]->osExpressionTree->m_treeRoot = nlNodeVec[0]->createExpressionTreeFromPostfix(nlNodeVec);
//
//				newinstance->instanceData->nonlinearExpressions->nl[k]->idx = tmpIdx;
//
//				if (constraintTypes[tmpIdx] == 'G')
//				{
//					// Greater than constraint
//					double tmpBound = newinstance->instanceData->constraints->con[tmpIdx]->lb;
//					newinstance->instanceData->constraints->con[tmpIdx]->lb = -COIN_DBL_MAX;
//					newinstance->instanceData->constraints->con[tmpIdx]->ub = 0.0;
//
//					nlNodeNumberPoint = new OSnLNodeNumber();
//					nlNodeNumberPoint->value = tmpBound;
//					nlNodeVec.push_back(nlNodeNumberPoint);
//
//					nlNodePoint = new OSnLNodeMinus();
//					nlNodeVec.push_back(nlNodePoint);
//
//					nlNodePoint = new OSnLNodeNegate();
//					nlNodeVec.push_back(nlNodePoint);
//					constraintTypes[tmpIdx] = 'L';
//				}
//				else if (constraintTypes[tmpIdx] == 'L')
//				{
//					nlNodeNumberPoint = new OSnLNodeNumber();
//					nlNodeNumberPoint->value = newinstance->instanceData->constraints->con[tmpIdx]->ub;
//					nlNodeVec.push_back(nlNodeNumberPoint);
//
//					nlNodePoint = new OSnLNodeMinus();
//					nlNodeVec.push_back(nlNodePoint);
//
//					newinstance->instanceData->constraints->con[tmpIdx]->ub = 0.0;
//				}
//
//				//nlNodeVariablePoint = new OSnLNodeVariable();
//				//nlNodeVariablePoint->idx = numVar;
//				//nlNodeVec.push_back(nlNodeVariablePoint);
//
//				//nlNodePoint = new OSnLNodeMinus();
//				//nlNodeVec.push_back(nlNodePoint);
//
//				newinstance->instanceData->nonlinearExpressions->nl[k]->osExpressionTree->m_treeRoot = nlNodeVec[0]->createExpressionTreeFromPostfix(nlNodeVec);
//				//std::cout << nlNodeVec[0]->createExpressionTreeFromPostfix(nlNodeVec) << std::endl;
//				//std::cout << nlNodeVec[0]->getNonlinearExpressionInXML() << std::endl;
//				nlNodeVec.clear();
//				k++;
//			}
//		}
//	}
//	else
//	{
//
//	}
//
//
//	newinstance->bVariablesModified = true;
//	newinstance->bConstraintsModified = true;
//	newinstance->bObjectivesModified = true;
//
//	problemInstance = newinstance;
//
//	//std::cout << problemInstance->printModel() << std::endl;
//}
//
//
//void OptProblemNLPMinimax::reformulate(OSInstance *originalInstance)
//{
//	originalInstance->addQTermsToExpressionTree();
//	bool nonlinobjfunct = false;
//	int numVar = originalInstance->getVariableNumber();
//	auto varNames = originalInstance->getVariableNames();
//	auto varTypes = originalInstance->getVariableTypes();
//	auto varLBs = originalInstance->getVariableLowerBounds();
//	auto varUBs = originalInstance->getVariableUpperBounds();
//
//	int numCon = originalInstance->getConstraintNumber();
//	auto conNames = originalInstance->getConstraintNames();
//	auto conLBs = originalInstance->getConstraintLowerBounds();
//	auto conUBs = originalInstance->getConstraintUpperBounds();
//	auto conConsts = originalInstance->getConstraintConstants();
//
//	int nunNonlinearConstrs = originalInstance->getNumberOfNonlinearConstraints();
//
//	auto nonlinearConstrs = originalInstance->getNonlinearExpressions();
//	int nunLinearConstrs = originalInstance->getConstraintNumber() - nunNonlinearConstrs;
//
//	bool isObjMin = originalInstance->getObjectiveMaxOrMins()[0] == "min";
//
//	OSInstance *newinstance = NULL;
//	newinstance = new OSInstance();
//
//
//	/*if (originalInstance->getNumberOfNonlinearObjectives() > 0)
//	{
//	nonlinobjfunct = true;
//	newinstance->setVariableNumber(numVar + 1);
//	newinstance->setConstraintNumber(numCon + 1);
//
//	idxObjectiveVariable = numVar + 1;
//	idxNonlinearObjectiveFunctionConstraint = numCon;
//
//	for (int i = 0; i < numVar; i++)
//	{
//	newinstance->addVariable(i, varNames[i], varLBs[i], varUBs[i], varTypes[i]);
//	}
//
//	newinstance->addVariable(numVar, "addobjvar", -100000000, 100000000, 'C');
//	}
//	else
//	{*/
//	newinstance->setVariableNumber(numVar + 1);
//	newinstance->setConstraintNumber(numCon);
//
//	for (int i = 0; i < numVar; i++)
//	{
//		newinstance->addVariable(i, varNames[i], varLBs[i], varUBs[i], 'C');
//	}
//	//}
//	double tmpObjBound = settings->getDoubleSetting("NLPObjectiveBound", "NLP");
//
//	newinstance->addVariable(numVar, "tempobjvar", -tmpObjBound, tmpObjBound, 'C');
//	newinstance->bVariablesModified = true;
//
//
//	for (int i = 0; i < numCon; i++)
//	{
//		newinstance->addConstraint(i,
//			originalInstance->instanceData->constraints->con[i]->name,
//			originalInstance->instanceData->constraints->con[i]->lb,
//			originalInstance->instanceData->constraints->con[i]->ub,
//			originalInstance->instanceData->constraints->con[i]->constant);
//	}
//
//	newinstance->bConstraintsModified = true;
//
//	//std::cout << newinstance->printModel(0) << std::endl;
//
//	newinstance->setObjectiveNumber(1);
//
//	//SparseVector** oldobjcoeff = originalInstance->getObjectiveCoefficients();
//
//	int numCoeff = originalInstance->instanceData->objectives->obj[0]->numberOfObjCoef;
//
//	SparseVector * newobjcoeff = new SparseVector(1);
//	newobjcoeff->indexes[0] = numVar;
//	newobjcoeff->values[0] = 1;
//
//	newinstance->addObjective(-1, "newobj", "min", 0.0, 1.0, newobjcoeff);
//	delete newobjcoeff;
//
//
//	newinstance->bObjectivesModified = true;
//
//	OSnLNodeVariable *nlNodeVariablePoint;
//	OSnLNodeNumber *nlNodeNumberPoint;
//	std::vector<OSnLNode*> nlNodeVec;
//	OSnLNode *nlNodePoint;
//
//	newinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions = nunNonlinearConstrs;
//
//	newinstance->instanceData->nonlinearExpressions->nl = new Nl*[nunNonlinearConstrs];
//
//	// Copies the linear constraints
//	int row_nonz = 0;
//	int obj_nonz = 0;
//	int varIdx = 0;
//	SparseMatrix *m_linearConstraintCoefficientsInRowMajor = originalInstance->getLinearConstraintCoefficientsInRowMajor();
//
//	CoinPackedMatrix *matrix = new CoinPackedMatrix(false, 0, 0);
//	matrix->setDimensions(0, numVar);
//
//	std::vector<double>rowLBs;
//	std::vector<double>rowUBs;
//
//	for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
//	{
//		if (originalInstance->instanceData->linearConstraintCoefficients != NULL &&
//			originalInstance->instanceData->linearConstraintCoefficients->numberOfValues > 0)
//		{
//			row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1] - m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];
//
//			std::vector<int> varIndexes;
//			std::vector<double> varElements;
//
//			CoinPackedVector row;
//
//			for (int j = 0; j < row_nonz; j++)
//			{
//				varIdx = m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
//				row.insert(varIdx, m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j]);
//			}
//
//			matrix->appendRow(row);
//
//			// Adds the bounds of the rows
//			rowLBs.push_back(originalInstance->instanceData->constraints->con[rowIdx]->lb);
//			rowUBs.push_back(originalInstance->instanceData->constraints->con[rowIdx]->ub);
//		}
//	}
//
//	int numnonz = originalInstance->instanceData->linearConstraintCoefficients->numberOfValues;
//
//	int valuesBegin = 0;
//	int valuesEnd = numnonz - 1;
//	int startsBegin = 0;
//	int indexesBegin = 0;
//	int indexesEnd = numnonz - 1;
//	int startsEnd = matrix->isColOrdered() ? numVar + 1 : numCon;
//	newinstance->setLinearConstraintCoefficients(numnonz, matrix->isColOrdered(),
//		const_cast<double*>(matrix->getElements()), valuesBegin, valuesEnd,
//		const_cast<int*>(matrix->getIndices()), indexesBegin, indexesEnd,
//		const_cast<int*>(matrix->getVectorStarts()), startsBegin, startsEnd);
//
//	//auto origObjCoeffsNum = originalInstance->getObjectiveCoefficientNumbers()[0];
//	//bool quadraticInObjective = false;
//
//	//nunNonlinearConstrs = originalInstance->getNumberOfNonlinearExpressionTreeIndexes();
//
//	//newinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions = originalInstance->getNumberOfNonlinearExpressionTreeIndexes();
//	//newinstance->instanceData->nonlinearExpressions->nl = new Nl*[nunNonlinearConstrs];
//
//	//newinstance->printModel();
//
//	//
//	auto constraintTypes = originalInstance->getConstraintTypes();
//
//	auto allTrees = originalInstance->getAllNonlinearExpressionTrees();
//
//	std::map<int, OSExpressionTree*>::iterator pos;
//
//	int k = 0;
//	for (pos = allTrees.begin(); pos != allTrees.end(); ++pos)
//		//for (int i = 0; i < origInstance->getNumberOfNonlinearExpressionTreeIndexes(); i++)
//	{
//		int tmpIdx = pos->first;
//		if (tmpIdx == -1)
//		{
//
//		}
//		else
//		{
//
//			nlNodeVec = originalInstance->getNonlinearExpressionTreeInPostfix(tmpIdx);
//
//			newinstance->instanceData->nonlinearExpressions->nl[k] = new Nl();
//
//			newinstance->instanceData->nonlinearExpressions->nl[k]->osExpressionTree = new OSExpressionTree();
//			newinstance->instanceData->nonlinearExpressions->nl[k]->osExpressionTree->m_treeRoot = nlNodeVec[0]->createExpressionTreeFromPostfix(nlNodeVec);
//
//			newinstance->instanceData->nonlinearExpressions->nl[k]->idx = tmpIdx;
//
//			if (constraintTypes[tmpIdx] == 'G')
//			{
//				// Greater than constraint
//				double tmpBound = newinstance->instanceData->constraints->con[tmpIdx]->lb;
//				newinstance->instanceData->constraints->con[tmpIdx]->lb = -COIN_DBL_MAX;
//				newinstance->instanceData->constraints->con[tmpIdx]->ub = 0.0;
//
//				nlNodeNumberPoint = new OSnLNodeNumber();
//				nlNodeNumberPoint->value = tmpBound;
//				nlNodeVec.push_back(nlNodeNumberPoint);
//
//				nlNodePoint = new OSnLNodeMinus();
//				nlNodeVec.push_back(nlNodePoint);
//
//				nlNodePoint = new OSnLNodeNegate();
//				nlNodeVec.push_back(nlNodePoint);
//				constraintTypes[tmpIdx] == 'L';
//			}
//			else if (constraintTypes[tmpIdx] == 'L')
//			{
//				nlNodeNumberPoint = new OSnLNodeNumber();
//				nlNodeNumberPoint->value = newinstance->instanceData->constraints->con[tmpIdx]->ub;
//				nlNodeVec.push_back(nlNodeNumberPoint);
//
//				nlNodePoint = new OSnLNodeMinus();
//				nlNodeVec.push_back(nlNodePoint);
//
//				newinstance->instanceData->constraints->con[tmpIdx]->ub = 0.0;
//			}
//
//			nlNodeVariablePoint = new OSnLNodeVariable();
//			nlNodeVariablePoint->idx = numVar;
//			nlNodeVec.push_back(nlNodeVariablePoint);
//
//			nlNodePoint = new OSnLNodeMinus();
//			nlNodeVec.push_back(nlNodePoint);
//
//			newinstance->instanceData->nonlinearExpressions->nl[k]->osExpressionTree->m_treeRoot = nlNodeVec[0]->createExpressionTreeFromPostfix(nlNodeVec);
//			//std::cout << nlNodeVec[0]->createExpressionTreeFromPostfix(nlNodeVec) << std::endl;
//			//std::cout << nlNodeVec[0]->getNonlinearExpressionInXML() << std::endl;
//			nlNodeVec.clear();
//			k++;
//		}
//	}
//
//	newinstance->bVariablesModified = true;
//	newinstance->bConstraintsModified = true;
//	newinstance->bObjectivesModified = true;
//	//newinstance->bAMatrixModified = true;
//
//	/*for (int i = 0; i < originalInstance->instanceData->constraints->numberOfConstraints; i++)
//	std::cout << originalInstance->getConstraintTypes()[i] << std::endl;
//	*/
//
//
//	//std::cout << "Q: " << newinstance->instanceData->linearConstraintCoefficients-> << std::endl;
//
//	//std::cout << newinstance->printModel(0) << std::endl;
//	//std::cout << newinstance->printModel(1) << std::endl;
//	//std::cout << newinstance->printModel(2) << std::endl;
//
//	//std::cout << newinstance->printModel() << std::endl;
//	//auto test = newinstance->instanceData->objectives->obj[0];
//
//	problemInstance = newinstance;
//
//	//std::cout << problemInstance->printModel() << std::endl;
//}
//
//void OptProblemNLP::reformulate(OSInstance *originalProblem)
//{
//	OSInstance *newinstance = NULL;
//	newinstance = new OSInstance();
//	//newinstance->instanceData = origInstance->instanceData;
//
//	int numVar = originalProblem->getVariableNumber();
//	int numCon = originalProblem->getConstraintNumber();
//	auto varNames = originalProblem->getVariableNames();
//	auto conNames = originalProblem->getConstraintNames();
//	auto varLBs = originalProblem->getVariableLowerBounds();
//	auto varUBs = originalProblem->getVariableUpperBounds();
//	auto conLBs = originalProblem->getConstraintLowerBounds();
//	auto conUBs = originalProblem->getConstraintUpperBounds();
//
//	auto conConsts = originalProblem->getConstraintConstants();
//	auto varTypes = originalProblem->getVariableTypes();
//
//	auto nonlinearConstrs = originalProblem->getNonlinearExpressions();
//	int nunNonlinearConstrs = originalProblem->getNumberOfNonlinearExpressions();
//
//	newinstance->setVariableNumber(numVar + 1);
//
//	for (int i = 0; i < numVar; i++)
//	{
//		newinstance->addVariable(i, varNames[i], varLBs[i], varUBs[i], varTypes[i]);
//	}
//
//	newinstance->addVariable(numVar, "tempobjvar", -COIN_DBL_MAX, COIN_DBL_MAX, 'C');
//	newinstance->bVariablesModified = true;
//
//	newinstance->setConstraintNumber(numCon);
//
//	for (int i = 0; i < numCon; i++)
//	{
//		newinstance->addConstraint(i, conNames[i], conLBs[i], conUBs[i], conConsts[i]);
//	}
//
//	if (originalProblem->getObjectiveNumber() > 1)
//	{
//		std::cout << "Do not support multiobjective optimization!" << std::endl;
//	}
//	else
//	{
//		newinstance->addConstraint(-1, "addobjfunt", -COIN_DBL_MAX, 0.0, originalProblem->getObjectiveConstants()[0]);
//	}
//
//	newinstance->bConstraintsModified = true;
//
//	OSnLNodeVariable *nlNodeVariablePoint;
//	OSnLNodeNumber *nlNodeNumberPoint;
//	std::vector<OSnLNode*> nlNodeVec;
//	OSnLNode *nlNodePoint;
//
//	newinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions = originalProblem->instanceData->nonlinearExpressions->numberOfNonlinearExpressions;
//
//	newinstance->instanceData->nonlinearExpressions->nl = new Nl*[newinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions];
//
//	// Copies the linear constraints to the OSI instance
//	int row_nonz = 0;
//	int obj_nonz = 0;
//	int varIdx = 0;
//	SparseMatrix *m_linearConstraintCoefficientsInRowMajor = originalProblem->getLinearConstraintCoefficientsInRowMajor();
//
//	CoinPackedMatrix * matrix = new CoinPackedMatrix(false, 0, 0);
//	matrix->setDimensions(0, numVar);
//
//	std::vector<double>rowLBs;
//	std::vector<double>rowUBs;
//
//	for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
//	{
//		if (originalProblem->instanceData->linearConstraintCoefficients != NULL &&
//			originalProblem->instanceData->linearConstraintCoefficients->numberOfValues > 0)
//		{
//			row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1] - m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];
//
//			std::vector<int> varIndexes;
//			std::vector<double> varElements;
//
//			CoinPackedVector row;
//
//			for (int j = 0; j < row_nonz; j++)
//			{
//				varIdx = m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
//				row.insert(varIdx, m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j]);
//			}
//
//			matrix->appendRow(row);
//
//			// Adds the bounds of the rows
//			rowLBs.push_back(originalProblem->instanceData->constraints->con[rowIdx]->lb);
//			rowUBs.push_back(originalProblem->instanceData->constraints->con[rowIdx]->ub);
//		}
//	}
//
//
//	int numnonz = originalProblem->instanceData->linearConstraintCoefficients->numberOfValues;
//
//	int valuesBegin = 0;
//	int valuesEnd = numnonz - 1;
//	int startsBegin = 0;
//	int indexesBegin = 0;
//	int indexesEnd = numnonz - 1;
//	int startsEnd = matrix->isColOrdered() ? numVar : numCon;
//	newinstance->setLinearConstraintCoefficients(numnonz, matrix->isColOrdered(),
//		const_cast<double*>(matrix->getElements()), valuesBegin, valuesEnd,
//		const_cast<int*>(matrix->getIndices()), indexesBegin, indexesEnd,
//		const_cast<int*>(matrix->getVectorStarts()), startsBegin, startsEnd);
//
//	for (int i = 0; i < nunNonlinearConstrs; i++)
//	{
//		newinstance->instanceData->nonlinearExpressions->nl[i] = new Nl();
//		newinstance->instanceData->nonlinearExpressions->nl[i]->idx = originalProblem->instanceData->nonlinearExpressions->nl[i]->idx;
//		newinstance->instanceData->nonlinearExpressions->nl[i]->osExpressionTree = new OSExpressionTree();
//
//		//auto tmpTree = origInstance->getNonlinearExpressionTree(newinstance->instanceData->nonlinearExpressions->nl[i]->idx);
//
//		nlNodeVec = originalProblem->getNonlinearExpressionTreeInPostfix(originalProblem->instanceData->nonlinearExpressions->nl[i]->idx);
//
//		nlNodeVariablePoint = new OSnLNodeVariable();
//		nlNodeVariablePoint->idx = numVar;
//		nlNodeVec.push_back(nlNodeVariablePoint);
//
//		nlNodeNumberPoint = new OSnLNodeNumber();
//		nlNodeNumberPoint->value = -1.0;
//		nlNodeVec.push_back(nlNodeNumberPoint);
//
//		nlNodePoint = new OSnLNodeTimes();
//		nlNodeVec.push_back(nlNodePoint);
//
//		nlNodePoint = new OSnLNodePlus();
//		nlNodeVec.push_back(nlNodePoint);
//
//		newinstance->instanceData->nonlinearExpressions->nl[i]->osExpressionTree->m_treeRoot = nlNodeVec[0]->createExpressionTreeFromPostfix(nlNodeVec);
//		nlNodeVec.clear();
//	}
//
//	newinstance->setObjectiveNumber(1);
//	SparseVector *objcoeff;
//	objcoeff = new SparseVector(1);
//	objcoeff->indexes[0] = numVar;
//	objcoeff->values[0] = 1;
//
//	newinstance->addObjective(-1, "objfunction", "min", 0.0, 1.0, objcoeff);
//	objcoeff->bDeleteArrays = true;
//	delete objcoeff;
//	for (int i = 0; i < numVar; i++)
//	{
//		if (varTypes[i] == 'I' || varTypes[i] == 'B')
//		{
//			newinstance->instanceData->variables->var[i]->type = 'C';
//		}
//	}
//
//
//	newinstance->bVariablesModified = true;
//	newinstance->bConstraintsModified = true;
//	newinstance->bObjectivesModified = true;
//
//	problemInstance = newinstance;
//
//	//std::cout << problemInstance->printModel() << std::endl;
//}
IndexValuePair OptProblemNLPMinimax::getMostDeviatingConstraint(std::vector<double> point)
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

bool OptProblemNLPMinimax::isConstraintsFulfilledInPoint(std::vector<double> point, double eps)
{
	std::vector<int> idxNLCs = this->getNonlinearOrQuadraticConstraintIndexes();

	for (int i = 0; i < getNumberOfNonlinearConstraints(); i++)
	{
		double tmpVal = calculateConstraintFunctionValue(idxNLCs.at(i), point);
		if (tmpVal > eps) return false;
	}

	return true;
}
