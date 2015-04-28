#include "OptProblemNLPRelaxed.h"

OptProblemNLPRelaxed::OptProblemNLPRelaxed()
{
	//problemInstance = NULL;
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
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

	//newInstance->setVariableNumber(originalInstance->getVariableNumber());
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
	//newInstance->initForAlgDiff();

}

/*
 void OptProblemNLPRelaxed::copyVariables(OSInstance *source, OSInstance *destination, bool integerRelaxed)
 {
 //destination->setVariableNumber(source->getVariableNumber() + 1);

 OptProblemBase::copyVariables(source, destination, integerRelaxed);

 //double tmpObjBound = settings->getDoubleSetting("NLPObjectiveBound", "NLP");

 //destination->addVariable(source->getVariableNumber(), "tempobjvar", -tmpObjBound, tmpObjBound, 'C');
 //destination->bVariablesModified = true;
 }*/

//
//void OptProblemNLPRelaxed::reformulate(OSInstance *originalInstance)
//{
//	//bool tmp = originalInstance->addQTermsToExpressionTree();
//	originalInstance->bConstraintsModified = true;
//	originalInstance->bObjectivesModified = true;
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
//	int nunNonlinearExprs = originalInstance->getNumberOfNonlinearExpressions();
//	//int numQuadraticRows = originalInstance->getNumberOfQuadraticRowIndexes();
//
//	auto nonlinearConstrs = originalInstance->getNonlinearExpressions();
//	int nunLinearConstrs = originalInstance->getConstraintNumber() - originalInstance->getNumberOfNonlinearConstraints();
//
//	bool isObjMin = originalInstance->getObjectiveMaxOrMins()[0] == "min";
//
//	OSInstance *newinstance = NULL;
//	newinstance = new OSInstance();
//
//
//	if (originalInstance->getNumberOfNonlinearObjectives() > 0)
//	{
//		nonlinobjfunct = true;
//		newinstance->setVariableNumber(numVar + 1);
//		newinstance->setConstraintNumber(numCon + 1);
//
//		idxObjectiveVariable = numVar + 1;
//		idxNonlinearObjectiveFunctionConstraint = numCon;
//
//		for (int i = 0; i < numVar; i++)
//		{
//			newinstance->addVariable(i, varNames[i], varLBs[i], varUBs[i], 'C');
//		}
//		double tmpObjBound = settings->getDoubleSetting("NLPObjectiveBound", "NLP");
//
//		newinstance->addVariable(numVar, "addobjvar", -tmpObjBound, tmpObjBound, 'C');
//
//	}
//	else
//	{
//		newinstance->setVariableNumber(numVar);
//		newinstance->setConstraintNumber(numCon);
//
//		for (int i = 0; i < numVar; i++)
//		{
//			newinstance->addVariable(i, varNames[i], varLBs[i], varUBs[i], 'C');
//		}
//	}
//
//	newinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions = nunNonlinearExprs;
//
//	newinstance->instanceData->nonlinearExpressions->nl = new Nl*[newinstance->instanceData->nonlinearExpressions->numberOfNonlinearExpressions];
//
//	newinstance->bVariablesModified = true;
//	newinstance->bConstraintsModified = true;
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
//	if (nonlinobjfunct)
//	{
//		newinstance->addConstraint(numCon, "objf", -COIN_DBL_MAX, 0.0, 0.0);
//	}
//
//	newinstance->bConstraintsModified = true;
//
//	//std::cout << newinstance->printModel(0) << std::endl;
//	//std::cout << newinstance->printModel(numCon) << std::endl;
//
//	newinstance->setObjectiveNumber(1);
//
//	if (nonlinobjfunct) // Nonlinear objective in original problem
//	{
//		SparseVector *objcoeff;
//		objcoeff = new SparseVector(1);
//		objcoeff->indexes[0] = numVar;
//		objcoeff->values[0] = 1;
//
//		newinstance->addObjective(-1, "objfunction", "min", 0.0, 1.0, objcoeff);
//		objcoeff->bDeleteArrays = true;
//		delete objcoeff;
//	}
//	else // Linear objective in original problem
//	{
//		SparseVector** oldobjcoeff = originalInstance->getObjectiveCoefficients();
//
//		int numCoeff = originalInstance->instanceData->objectives->obj[0]->numberOfObjCoef;
//
//		SparseVector * newobjcoeff = new SparseVector(numCoeff);
//
//		if (isObjMin)
//		{
//			for (int i = 0; i < numCoeff; i++)
//			{
//				newobjcoeff->indexes[i] = oldobjcoeff[0]->indexes[i];
//				newobjcoeff->values[i] = oldobjcoeff[0]->values[i];
//			}
//		}
//		else
//		{
//			for (int i = 0; i < numCoeff; i++)
//			{
//				newobjcoeff->indexes[i] = oldobjcoeff[0]->indexes[i];
//				newobjcoeff->values[i] = -oldobjcoeff[0]->values[i];
//			}
//		}
//
//		newinstance->addObjective(-1, originalInstance->instanceData->objectives->obj[0]->name, "min", originalInstance->instanceData->objectives->obj[0]->constant, 1.0, newobjcoeff);
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
//	auto constraintTypes = originalInstance->getConstraintTypes();
//	//auto nonExpressionTreeIndexes = originalInstance->getNonlinearExpressionTreeIndexes();
//
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
//
//				double tmpVal = m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
//
//				if (constraintTypes[rowIdx] == 'G' && originalInstance->getNonlinearExpressionTree(rowIdx) != NULL)
//				{
//					tmpVal = -tmpVal;
//				}
//
//				row.insert(varIdx, tmpVal);
//
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
//	if (nonlinobjfunct)
//	{
//		CoinPackedVector row;
//		row.insert(numVar, -1);
//		matrix->appendRow(row);
//		rowLBs.push_back(-COIN_DBL_MAX);
//		rowUBs.push_back(0);
//	}
//
//	int numnonz = originalInstance->instanceData->linearConstraintCoefficients->numberOfValues;
//
//	int valuesBegin = 0;
//	int valuesEnd = numnonz - 1;
//	int startsBegin = 0;
//	int indexesBegin = 0;
//	int indexesEnd = numnonz - 1;
//	int startsEnd = matrix->isColOrdered() ? numVar + 1 : numCon + 1;
//	newinstance->setLinearConstraintCoefficients(numnonz, matrix->isColOrdered(),
//		const_cast<double*>(matrix->getElements()), valuesBegin, valuesEnd,
//		const_cast<int*>(matrix->getIndices()), indexesBegin, indexesEnd,
//		const_cast<int*>(matrix->getVectorStarts()), startsBegin, startsEnd);
//
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
//
//	if (originalInstance->getNumberOfNonlinearExpressions() > 0)
//	{
//
//		auto origObjCoeffsNum = originalInstance->getObjectiveCoefficientNumbers()[0];
//		auto allTrees = originalInstance->getAllNonlinearExpressionTrees();
//
//		std::map<int, OSExpressionTree*>::iterator pos;
//
//		int k = 0;
//		for (pos = allTrees.begin(); pos != allTrees.end(); ++pos)
//		{
//			int tmpIdx = pos->first;
//
//			nlNodeVec = originalInstance->getNonlinearExpressionTreeInPostfix(tmpIdx);
//
//			newinstance->instanceData->nonlinearExpressions->nl[k] = new Nl();
//
//			newinstance->instanceData->nonlinearExpressions->nl[k]->osExpressionTree = new OSExpressionTree();
//			newinstance->instanceData->nonlinearExpressions->nl[k]->osExpressionTree->m_treeRoot = nlNodeVec[0]->createExpressionTreeFromPostfix(nlNodeVec);
//
//			if (tmpIdx == -1)
//			{
//				// The nonlinear part of the objective function
//
//				newinstance->instanceData->nonlinearExpressions->nl[k]->idx = numCon;
//
//				for (int j = 0; j < origObjCoeffsNum; j++)
//				{
//					double tmpCoeff = originalInstance->instanceData->objectives->obj[0]->coef[j]->value;
//					int varIdx = originalInstance->instanceData->objectives->obj[0]->coef[j]->idx;
//
//					nlNodeNumberPoint = new OSnLNodeNumber();
//					nlNodeNumberPoint->value = tmpCoeff;
//					nlNodeVec.push_back(nlNodeNumberPoint);
//
//					nlNodeVariablePoint = new OSnLNodeVariable();
//					nlNodeVariablePoint->idx = varIdx;
//					nlNodeVec.push_back(nlNodeVariablePoint);
//
//					nlNodePoint = new OSnLNodeTimes();
//					nlNodeVec.push_back(nlNodePoint);
//
//					nlNodePoint = new OSnLNodePlus();
//					nlNodeVec.push_back(nlNodePoint);
//				}
//
//				if (originalInstance->instanceData->objectives->obj[0]->constant != 0.0)
//				{
//					nlNodeNumberPoint = new OSnLNodeNumber();
//
//					nlNodeNumberPoint->value = originalInstance->instanceData->objectives->obj[0]->constant;
//
//					nlNodeVec.push_back(nlNodeNumberPoint);
//
//					nlNodePoint = new OSnLNodePlus();
//					nlNodeVec.push_back(nlNodePoint);
//
//					newinstance->instanceData->objectives->obj[0]->constant = 0.0;
//				}
//
//				if (!isObjMin)
//				{
//					nlNodePoint = new OSnLNodeNegate();
//					nlNodeVec.push_back(nlNodePoint);
//				}
//			}
//			else
//			{
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
//					constraintTypes[tmpIdx] == 'L';
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
//			}
//
//			newinstance->instanceData->nonlinearExpressions->nl[k]->osExpressionTree->m_treeRoot = nlNodeVec[0]->createExpressionTreeFromPostfix(nlNodeVec);
//
//			nlNodeVec.clear();
//			k++;
//		}
//	}
//
//	newinstance->bVariablesModified = true;
//	newinstance->bConstraintsModified = true;
//	newinstance->bObjectivesModified = true;
//
//	problemInstance = newinstance;
//
//	std::cout << problemInstance->printModel() << std::endl;
//}

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
