#include "OptProblem.h"

OptProblem::OptProblem()
{
	m_problemInstance = NULL;
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

OptProblem::~OptProblem()
{
	delete m_problemInstance;
}

int OptProblem::getNumberOfLinearConstraints()
{
	return getProblemInstance()->getConstraintNumber() - this->getNumberOfNonlinearConstraints();
}

int OptProblem::getNumberOfConstraints()
{
	return getProblemInstance()->getConstraintNumber();
}

std::vector<std::string> OptProblem::getConstraintNames()
{
	std::string* tmpArray = getProblemInstance()->getConstraintNames();
	std::vector < std::string > tmpVector(tmpArray, tmpArray + getProblemInstance()->getConstraintNumber());

	return tmpVector;
}

int OptProblem::getNumberOfVariables()
{
	return getProblemInstance()->getVariableNumber();
}

int OptProblem::getNumberOfBinaryVariables()
{
	return getProblemInstance()->getNumberOfBinaryVariables();
}

int OptProblem::getNumberOfIntegerVariables()
{
	return getProblemInstance()->getNumberOfIntegerVariables();
}

int OptProblem::getNumberOfRealVariables()
{
	return getProblemInstance()->getVariableNumber() - getProblemInstance()->getNumberOfBinaryVariables()
			- getProblemInstance()->getNumberOfIntegerVariables();
}

std::vector<std::string> OptProblem::getVariableNames()
{
	std::string* tmpArray = getProblemInstance()->getVariableNames();
	std::vector < std::string > tmpVector(tmpArray, tmpArray + getProblemInstance()->getVariableNumber());

	return tmpVector;
}

std::vector<char> OptProblem::getVariableTypes()
{
	char* tmpArray = getProblemInstance()->getVariableTypes();
	std::vector<char> tmpVector(tmpArray, tmpArray + getProblemInstance()->getVariableNumber());

	return tmpVector;
}

std::vector<double> OptProblem::getVariableLowerBounds()
{
	double* tmpArray = getProblemInstance()->getVariableLowerBounds();
	std::vector<double> tmpVector(tmpArray, tmpArray + getProblemInstance()->getVariableNumber());

	return tmpVector;
}

std::vector<double> OptProblem::getVariableUpperBounds()
{
	double* tmpArray = getProblemInstance()->getVariableUpperBounds();
	std::vector<double> tmpVector(tmpArray, tmpArray + getProblemInstance()->getVariableNumber());

	return tmpVector;
}

std::vector<int> OptProblem::getRealVariableIndices()
{
	std::vector<int> indices;
	indices.reserve(getNumberOfRealVariables());

	auto varTypes = getVariableTypes();

	for (int i = 0; i < getNumberOfVariables(); i++)
	{
		if (varTypes.at(i) == 'C') indices.push_back(i);
	}

	return indices;
}

std::vector<int> OptProblem::getDiscreteVariableIndices()
{
	std::vector<int> indices;
	indices.reserve(getNumberOfIntegerVariables() + getNumberOfBinaryVariables());

	auto varTypes = getVariableTypes();

	for (int i = 0; i < getNumberOfVariables(); i++)
	{
		if (varTypes.at(i) == 'I' || varTypes.at(i) == 'B') indices.push_back(i);
	}

	return indices;
}

void OptProblem::printProblemStatistics()
{
	processInfo->outputSummary("┌─── Problem statistics ─────────────────────────────────────────────────────────┐");

	processInfo->outputSummary(
			"│ Number of constraints (total/linear/nonlinear):   "
					+ to_string(getProblemInstance()->getConstraintNumber()) + "/"
					+ to_string(getNumberOfLinearConstraints()) + "/" + to_string(getNumberOfNonlinearConstraints()));

	processInfo->outputSummary(
			"│ Number of variables (total/real/binary/integer):  " + to_string(getNumberOfVariables()) + "/"
					+ to_string(getNumberOfRealVariables()) + "/" + to_string(getNumberOfBinaryVariables()) + "/"
					+ to_string(getNumberOfIntegerVariables()) + "/");

	processInfo->outputSummary("└────────────────────────────────────────────────────────────────────────────────┘");
	processInfo->outputSummary("");
}

void OptProblem::exportProblemToOsil(std::string fileName)
{
	OSiLWriter *osilWriter = new OSiLWriter();
	FileUtil *fileUtil = new FileUtil();

	std::string osil = osilWriter->writeOSiL(getProblemInstance());
	if (!fileUtil->writeFileFromString(fileName, osil))
	{
		processInfo->outputError("Error when writing to file " + fileName);
	}

	delete fileUtil;
	delete osilWriter;
}

void OptProblem::saveProblemModelToFile(std::string fileName)
{
	std::string problem = getProblemInstance()->printModel();
	FileUtil *fileUtil = new FileUtil();

	if (!fileUtil->writeFileFromString(fileName, problem))
	{
		processInfo->outputError("Error when writing to file " + fileName);
	}

	delete fileUtil;
}

std::string OptProblem::exportProblemToOsil()
{
	OSiLWriter *osilWriter = new OSiLWriter();

	std::string osil = osilWriter->writeOSiL(getProblemInstance());

	return (osil);
}

IndexValuePair OptProblem::getMostDeviatingConstraint(std::vector<double> point)
{
	IndexValuePair valpair;

	std::vector<int> idxNLCs = this->getNonlinearConstraintIndexes();

	return (this->getMostDeviatingConstraint(point, idxNLCs).first);

	/*if (idxNLCs.size() == 0)	//Only a quadratic objective function and quadratic constraints
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
	 return (valpair);*/
}

std::pair<IndexValuePair, std::vector<int>> OptProblem::getMostDeviatingConstraint(std::vector<double> point,
		std::vector<int> constrIdxs)
{
	IndexValuePair valpair;

	std::vector<int> activeConstraints;

	if (constrIdxs.size() == 0)
	{
		//Only a quadratic objective function and quadratic constraints
		valpair.idx = -1;
		valpair.value = 0.0;
	}
	else
	{
		std::vector<double> constrDevs(constrIdxs.size());

		for (int i = 0; i < constrIdxs.size(); i++)
		{
			constrDevs.at(i) = calculateConstraintFunctionValue(constrIdxs.at(i), point);

			if (constrDevs.at(i) >= 0) activeConstraints.push_back(constrIdxs.at(i));
		}

		auto biggest = std::max_element(std::begin(constrDevs), std::end(constrDevs));
		valpair.idx = constrIdxs.at(std::distance(std::begin(constrDevs), biggest));
		valpair.value = *biggest;
	}

	return (std::make_pair(valpair, activeConstraints));
}

IndexValuePair OptProblem::getMostDeviatingAllConstraint(std::vector<double> point)
{

	IndexValuePair valpair;

	int numConstr = this->getNumberOfConstraints();

	std::vector<double> constrDevs(numConstr);

	for (int i = 0; i < numConstr; i++)
	{
		constrDevs.at(i) = calculateConstraintFunctionValue(i, point);
	}

//UtilityFunctions::displayVector(constrDevs);

	auto biggest = std::max_element(std::begin(constrDevs), std::end(constrDevs));
	valpair.idx = std::distance(std::begin(constrDevs), biggest);
	valpair.value = *biggest;

	return valpair;
}

bool OptProblem::isConstraintsFulfilledInPoint(std::vector<double> point, double eps)
{
	std::vector<int> idxNLCs = this->getNonlinearConstraintIndexes();

	int numNLC = getNumberOfNonlinearConstraints();

	for (int i = 0; i < numNLC; i++)
	{
		double tmpVal = calculateConstraintFunctionValue(idxNLCs.at(i), point);
		if (tmpVal > eps)
		{
			return false;
		}
	}

	return true;
}

bool OptProblem::isConstraintsFulfilledInPoint(std::vector<double> point)
{
	return isConstraintsFulfilledInPoint(point, 0);
}

bool OptProblem::isLinearConstraintsFulfilledInPoint(std::vector<double> point, double eps)
{
	std::vector<int> idxLCs = this->getLinearConstraintIndexes();

	/*

	 for (int i = 0; i < idxLCs.size(); i++)
	 {
	 std::cout << "Linear constraint index: " << idxLCs.at(i) << std::endl;
	 }*/

	int numLC = idxLCs.size();

	for (int i = 0; i < numLC; i++)
	{
		double tmpVal = calculateConstraintFunctionValue(idxLCs.at(i), point);
		//std::cout << "Lin :" << tmpVal << std::endl;
		if (tmpVal > eps) return false;
	}

	return true;
}

bool OptProblem::isLinearConstraintsFulfilledInPoint(std::vector<double> point)
{
	return isLinearConstraintsFulfilledInPoint(point, 0);
}

SparseVector* OptProblem::calculateConstraintFunctionGradient(int idx, std::vector<double> point)
{
	processInfo->numGradientEvals++;
	return getProblemInstance()->calculateConstraintFunctionGradient(&point.at(0), idx, true);
}

double OptProblem::calculateOriginalObjectiveValue(std::vector<double> point)
{
	auto tmpVal = getProblemInstance()->calculateAllObjectiveFunctionValues(&point[0], true)[0];

	processInfo->numFunctionEvals++;
//std::cout << "Obj value calculated: " << tmpVal << std::endl;
	return tmpVal;
}

double OptProblem::calculateConstraintFunctionValue(int idx, std::vector<double> point)
{

	double tmpVal;
	if (idx != getNonlinearObjectiveConstraintIdx())	// Not the objective function
	{
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
	}
	/*else if (idx != -1)
	 {
	 tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true)
	 - getProblemInstance()->instanceData->constraints->con[idx]->ub;
	 }*/
	else
	{
		tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);
		tmpVal = tmpVal - getProblemInstance()->instanceData->constraints->con[idx]->ub; // -problemInstance->getConstraintConstants()[idx];
		processInfo->numFunctionEvals++;
	}

//else if (idx == -1)
//{
//	tmpVal = getProblemInstance()->calculateFunctionValue(idx, &point.at(0), true);
//}

	return tmpVal;
}

int OptProblem::getNumberOfNonlinearConstraints()
{
	int ctr = 0;

	std::vector<bool> isNonlinear(getProblemInstance()->getConstraintNumber(), false);

	for (int i = 0; i < getProblemInstance()->getNumberOfNonlinearExpressions(); i++)
	{
		int tmpIndex = getProblemInstance()->instanceData->nonlinearExpressions->nl[i]->idx;

		if (tmpIndex != -1) isNonlinear.at(tmpIndex) = true;
	}

	if (!((static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm")))
			== ES_QPStrategy::QuadraticallyConstrained))
	{
		for (int i = 0; i < getProblemInstance()->getNumberOfQuadraticTerms(); i++)
		{
			int tmpIndex = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

			if (tmpIndex != -1) isNonlinear.at(tmpIndex) = true;
		}
	}

	for (int i = 0; i < isNonlinear.size(); i++)
	{
		if (isNonlinear.at(i)) ctr++;
	}

	return ctr;
}

int OptProblem::getNumberOfNonlinearConstraints(OSInstance *instance)
{
	int ctr = 0;

	std::vector<bool> isNonlinear(instance->getConstraintNumber(), false);

	for (int i = 0; i < instance->getNumberOfNonlinearExpressions(); i++)
	{
		int tmpIndex = instance->instanceData->nonlinearExpressions->nl[i]->idx;

		if (tmpIndex != -1) isNonlinear.at(tmpIndex) = true;
	}

	for (int i = 0; i < instance->getNumberOfQuadraticTerms(); i++)
	{
		int tmpIndex = instance->instanceData->quadraticCoefficients->qTerm[i]->idx;

		if (tmpIndex != -1) isNonlinear.at(tmpIndex) = true;
	}

	for (int i = 0; i < isNonlinear.size(); i++)
	{
		if (isNonlinear.at(i)) ctr++;
	}

	return ctr;
}

std::vector<std::pair<int, double>> OptProblem::getObjectiveFunctionVarCoeffPairs()
{
	int numCoeffs = getProblemInstance()->instanceData->objectives->obj[0]->numberOfObjCoef;

	std::vector<std::pair<int, double>> tmpVector(numCoeffs);

	for (int i = 0; i < numCoeffs; i++)
	{
		std::pair<int, double> tmpPair;

		tmpPair.first = getProblemInstance()->instanceData->objectives->obj[0]->coef[i]->idx;
		tmpPair.second = getProblemInstance()->instanceData->objectives->obj[0]->coef[i]->value;

		tmpVector.push_back(tmpPair);
	}

	return tmpVector;
}

double OptProblem::getObjectiveConstant()
{
	return getProblemInstance()->getObjectiveConstants()[0];
}

void OptProblem::copyVariables(OSInstance *source, OSInstance *destination, bool integerRelaxed)
{
	if (source->instanceData->variables != NULL && source->instanceData->variables->numberOfVariables > 0)
	{
		int nvar = source->getVariableNumber();
		//destination->setVariableNumber(nvar);

		std::string *varname = source->getVariableNames();
		char *vartype = source->getVariableTypes();
		double *varlb = source->getVariableLowerBounds();
		double *varub = source->getVariableUpperBounds();

		destination->instanceData->variables = new Variables();
		destination->instanceData->variables->numberOfVariables = nvar;
		destination->instanceData->variables->var = new Variable*[nvar];

		if (!destination->setVariables(nvar, varname, varlb, varub, vartype)) throw ErrorClass(
				"Error duplicating variable information");

		if (integerRelaxed)
		{
			for (int i = 0; i < nvar; i++)
			{
				destination->instanceData->variables->var[i]->type = 'C';
			}
		}
	}

	destination->bVariablesModified = true;
}

void OptProblem::copyObjectiveFunction(OSInstance *source, OSInstance *destination)
{
	if (source->instanceData->objectives != NULL)
	{
		int nobj = source->getObjectiveNumber();

		std::string *objname = source->getObjectiveNames();
		std::string *objdir = source->getObjectiveMaxOrMins();
		double *objconst = source->getObjectiveConstants();
		double *objweight = source->getObjectiveWeights();
		SparseVector **objcoeff = source->getObjectiveCoefficients();

		destination->instanceData->objectives = new Objectives();
		destination->instanceData->objectives->numberOfObjectives = nobj;
		destination->instanceData->objectives->obj = new Objective*[nobj];

		if (!destination->setObjectives(nobj, objname, objdir, objconst, objweight, objcoeff)) throw ErrorClass(
				"Error duplicating objective information");
	}
}

void OptProblem::copyConstraints(OSInstance *source, OSInstance *destination)
{
	if (source->instanceData->constraints != NULL)
	{
		int ncon = source->getConstraintNumber();

		std::string *conname = source->getConstraintNames();
		double *conlb = source->getConstraintLowerBounds();
		double *conub = source->getConstraintUpperBounds();
		double *con_c = source->getConstraintConstants();

		destination->instanceData->constraints = new Constraints();
		destination->instanceData->constraints->numberOfConstraints = ncon;
		destination->instanceData->constraints->con = new Constraint*[ncon];

		if (!destination->setConstraints(ncon, conname, conlb, conub, con_c)) throw ErrorClass(
				"Error duplicating constraint information");
	}

}

void OptProblem::copyQuadraticTerms(OSInstance *source, OSInstance *destination)
{
	if (source->instanceData->quadraticCoefficients != NULL)
	{
		int nquad = source->getNumberOfQuadraticTerms();
		QuadraticTerms* qcoef = source->getQuadraticTerms();

		if (!destination->setQuadraticCoefficients(nquad, qcoef->rowIndexes, qcoef->varOneIndexes, qcoef->varTwoIndexes,
				qcoef->coefficients, 0, nquad - 1)) throw ErrorClass("Error duplicating quadratic coefficients");
	}
}

void OptProblem::copyNonlinearExpressions(OSInstance *source, OSInstance *destination)
{
	/*Nl** root = NULL;

	 if (source->instanceData->nonlinearExpressions != NULL)
	 {
	 int nexpr = source->getNumberOfNonlinearExpressions();
	 //            root = osinstance->getNonlinearExpressions();
	 root = new Nl*[source->getNumberOfNonlinearExpressions()];
	 for (int i=0; i < source->getNumberOfNonlinearExpressions(); i++)
	 {
	 root[i] = source->instanceData->nonlinearExpressions->nl[i];
	 }

	 if (!destination->setNonlinearExpressions(nexpr, root))
	 throw ErrorClass("Error duplicating nonlinear expressions");
	 }*/

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
			destination->instanceData->nonlinearExpressions->nl[i]->idx = rowIdx;
			nlNodeVec.clear();
			//valsAdded++;

		}
	}
}

void OptProblem::copyLinearTerms(OSInstance *source, OSInstance *destination)
{
	if (source->instanceData->linearConstraintCoefficients != NULL)
	{
		int ncoef = source->getLinearConstraintCoefficientNumber();
		bool isColMajor = source->getLinearConstraintCoefficientMajor();
		int nstart;
		SparseMatrix* coeff;

		// getLinearConstraintCoefficients returns a pointer to a sparse matrix structure
		if (isColMajor)
		{
			nstart = source->getVariableNumber();
			coeff = source->getLinearConstraintCoefficientsInColumnMajor();
		}
		else
		{
			nstart = source->getConstraintNumber();
			coeff = source->getLinearConstraintCoefficientsInRowMajor();
		}

		if (!destination->copyLinearConstraintCoefficients(ncoef, isColMajor, coeff->values, 0, ncoef - 1,
				coeff->indexes, 0, ncoef - 1, coeff->starts, 0, nstart)) throw ErrorClass(
				"Error duplicating linear constraint coefficients");
	}
}

void OptProblem::setNonlinearConstraintIndexes()
{
	std::vector<bool> isNonlinear(this->getProblemInstance()->getConstraintNumber(), false);
	std::vector<bool> isQuadratic(this->getProblemInstance()->getConstraintNumber(), false);
	int numNonlinExpr = this->getProblemInstance()->getNumberOfNonlinearExpressions();
	int numQuadTerms = this->getProblemInstance()->getNumberOfQuadraticTerms();

	for (int i = 0; i < numNonlinExpr; i++)
	{
		int tmpIndex = this->getProblemInstance()->instanceData->nonlinearExpressions->nl[i]->idx;

		if (tmpIndex != -1)
		{
			isNonlinear.at(tmpIndex) = true;
		}
	}

//	if (settings->getBoolSetting("UseQuadraticProgramming", "Algorithm"))
	if (static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm"))
			== ES_QPStrategy::QuadraticallyConstrained)
	{
		for (int i = 0; i < numQuadTerms; i++)
		{
			int tmpIndex = this->getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

			if (tmpIndex != -1)
			{
				isQuadratic.at(tmpIndex) = true;
			}
		}
	}
	else
	{
		for (int i = 0; i < numQuadTerms; i++)
		{
			int tmpIndex = this->getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

			if (tmpIndex != -1)
			{
				isNonlinear.at(tmpIndex) = true;
			}
		}
	}

	std::vector<int> NLCIndexes, QCIndexes;

	for (int i = 0; i < isNonlinear.size(); i++)
	{
		if (isNonlinear.at(i)) NLCIndexes.push_back(i);
	}

	for (int i = 0; i < isQuadratic.size(); i++)
	{
		if (isQuadratic.at(i)) QCIndexes.push_back(i);
	}

//UtilityFunctions::displayVector(NLCIndexes);
//UtilityFunctions::displayVector(QCIndexes);

	setNonlinearConstraints(NLCIndexes);
	setQuadraticConstraints(QCIndexes);

}

/*
 void OptProblemBase::setQuadraticConstraintIndexes()
 {
 std::vector<int> quadraticIndexes;

 if (settings->getBoolSetting("UseQuadraticProgramming", "Algorithm"))
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

 for (int i = 0; i < isQuadratic.size(); i++)
 {
 if (isQuadratic.at(i))	quadraticIndexes.push_back(i);
 }

 }

 setQuadraticConstraints(quadraticIndexes);
 }*/

std::vector<int> OptProblem::getLinearConstraintIndexes()
{
	std::vector<int> idxs;
	int numCon = this->getProblemInstance()->getConstraintNumber();
//std::cout << "Nonlinear constraint index : " << this->getNonlinearObjectiveConstraintIdx() << std::endl;
//int numCon = this->getNumberOfConstraints();
//idxs.reserve(numCon);

	for (int i = 0; i < numCon; i++)
	{
		if (!isConstraintNonlinear(this->getProblemInstance(), i) && this->getNonlinearObjectiveConstraintIdx() != i)
		{
			//std::cout << "Constr " << i << " is linear!" << std::endl;
			idxs.push_back(i);
		}
	}

	return idxs;
}

std::vector<int> OptProblem::getNonlinearConstraintIndexes()
{
	return m_nonlinearConstraints;
}

void OptProblem::setNonlinearConstraints(std::vector<int> idxs)
{
	m_nonlinearConstraints = idxs;

	if (m_nonlinearOrQuadraticConstraints.size() == 0)
	{
		m_nonlinearOrQuadraticConstraints = idxs;
	}
	else
	{
		vector<int> newVect;
		set_union(m_nonlinearOrQuadraticConstraints.begin(), m_nonlinearOrQuadraticConstraints.end(),
				m_nonlinearConstraints.begin(), m_nonlinearConstraints.end(), back_inserter(newVect));

		m_nonlinearOrQuadraticConstraints = newVect;
	}
}

std::vector<int> OptProblem::getQuadraticConstraintIndexes()
{
	return m_quadraticConstraints;
}

void OptProblem::setQuadraticConstraints(std::vector<int> idxs)
{
	m_quadraticConstraints = idxs;

	if (m_nonlinearOrQuadraticConstraints.size() == 0)
	{
		m_nonlinearOrQuadraticConstraints = idxs;
	}
	else
	{
		vector<int> newVect;
		set_union(m_nonlinearOrQuadraticConstraints.begin(), m_nonlinearOrQuadraticConstraints.end(),
				m_quadraticConstraints.begin(), m_quadraticConstraints.end(), back_inserter(newVect));

		m_nonlinearOrQuadraticConstraints = newVect;
	}
}

std::vector<int> OptProblem::getNonlinearOrQuadraticConstraintIndexes()
{
	return m_nonlinearOrQuadraticConstraints;
}

/*
 void OptProblemBase::setNonlinearOrQuadraticConstraints(std::vector<int> idxs)
 {
 m_nonlinearOrQuadraticConstraints = idxs;
 }*/

bool OptProblem::isTypeOfObjectiveMinimize()
{
	return m_isTypeOfObjectiveMinimize;
}

bool OptProblem::isObjectiveFunctionNonlinear()
{
	return m_isObjectiveFunctionNonlinear;
}

void OptProblem::setTypeOfObjectiveMinimize(bool value)
{
	m_isTypeOfObjectiveMinimize = value;
}

void OptProblem::setObjectiveFunctionNonlinear(bool value)
{
	m_isObjectiveFunctionNonlinear = value;
}

bool OptProblem::isConstraintNonlinear(OSInstance *instance, int constrIdx)
{
	for (int i = 0; i < instance->getNumberOfNonlinearExpressions(); i++)
	{
		int tmpIndex = instance->instanceData->nonlinearExpressions->nl[i]->idx;

		if (tmpIndex == constrIdx) return true;
	}

	for (int i = 0; i < instance->getNumberOfQuadraticTerms(); i++)
	{
		int tmpIndex = instance->instanceData->quadraticCoefficients->qTerm[i]->idx;

		if (tmpIndex == constrIdx) return true;
	}

	return false;
}

bool OptProblem::isConstraintNonlinear(int constrIdx)
{
	for (int i = 0; i < getProblemInstance()->getNumberOfNonlinearExpressions(); i++)
	{
		int tmpIndex = getProblemInstance()->instanceData->nonlinearExpressions->nl[i]->idx;

		if (tmpIndex == constrIdx) return true;
	}

	auto QPStrategy = static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm"));

	if (QPStrategy == ES_QPStrategy::Nonlinear || QPStrategy != ES_QPStrategy::QuadraticallyConstrained)
//	if (!settings->getBoolSetting("UseQuadraticProgramming", "Algorithm"))
	{
		for (int i = 0; i < getProblemInstance()->getNumberOfQuadraticTerms(); i++)
		{
			int tmpIndex = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

			if (tmpIndex == constrIdx) return true;
		}
	}

	return false;
}

bool OptProblem::isConstraintQuadratic(int constrIdx)
{
	auto QPStrategy = static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm"));

	if (QPStrategy == ES_QPStrategy::QuadraticallyConstrained || QPStrategy == ES_QPStrategy::QuadraticObjective)
//	if (settings->getBoolSetting("UseQuadraticProgramming", "Algorithm"))
	{
		for (int i = 0; i < getProblemInstance()->getNumberOfQuadraticTerms(); i++)
		{
			int tmpIndex = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i]->idx;

			if (tmpIndex == constrIdx) return true;
		}
	}

	return false;
}

bool OptProblem::isProblemNonlinear(OSInstance *instance)
{
	if (instance->getNumberOfNonlinearExpressions() != 0) return true;

	if (instance->getNumberOfQuadraticTerms() != 0) return true;

	return false;
}

OSInstance * OptProblem::getProblemInstance()
{
	return m_problemInstance;
}
void OptProblem::setProblemInstance(OSInstance * instance)
{
	m_problemInstance = instance;
}

void OptProblem::setNonlinearObjectiveConstraintIdx(int idx)
{
	m_idxNonlinearObjectiveConstraint = idx;
}

int OptProblem::getNonlinearObjectiveConstraintIdx()
{
	return m_idxNonlinearObjectiveConstraint;
}

void OptProblem::setNonlinearObjectiveVariableIdx(int idx)
{
	m_idxNonlinearObjectiveVariable = idx;
}

int OptProblem::getNonlinearObjectiveVariableIdx()
{
	return m_idxNonlinearObjectiveVariable;
}

void OptProblem::setObjectiveFunctionType(E_ObjectiveFunctionType type)
{
	m_objectiveFunctionType = type;
}

E_ObjectiveFunctionType OptProblem::getObjectiveFunctionType()
{
	return m_objectiveFunctionType;
}

void OptProblem::repairNonboundedObjectiveVariable(OSInstance *instance)
{
	if (isConstraintNonlinear(-1)) return; // Do nothing if we have an nonlinear objective function

	if (instance->getObjectiveCoefficientNumbers()[0] == 1)
	{
		double tmpObjBound = settings->getDoubleSetting("NLPObjectiveBound", "NLP");
		int varIdx = instance->getObjectiveCoefficients()[0]->indexes[0];
		;

		if (instance->getObjectiveMaxOrMins()[0] == "min")
		{
			if (instance->instanceData->variables->var[varIdx]->lb < -tmpObjBound)
			{
				instance->instanceData->variables->var[varIdx]->lb = -tmpObjBound;
			}
		}
		else
		{
			if (instance->instanceData->variables->var[varIdx]->ub > tmpObjBound)
			{
				instance->instanceData->variables->var[varIdx]->ub = tmpObjBound;
			}
		}
	}
}

std::vector<QuadraticTerm*> OptProblem::getQuadraticTermsInConstraint(int constrIdx)
{

	auto QPStrategy = static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm"));
	std::vector<QuadraticTerm*> quadTerms;

	if (constrIdx != -1 && QPStrategy == ES_QPStrategy::QuadraticallyConstrained)
	{
		int numQuadTerms = getProblemInstance()->getNumberOfQuadraticTerms();

		quadTerms.reserve(numQuadTerms);

		for (int i = 0; i < numQuadTerms; i++)
		{
			auto tmpTerm = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i];

			if (tmpTerm->idx == constrIdx) quadTerms.push_back(tmpTerm);
		}
	}
	else if (constrIdx == -1
			&& (QPStrategy == ES_QPStrategy::QuadraticallyConstrained || QPStrategy == ES_QPStrategy::QuadraticObjective))

	{
		int numQuadTerms = getProblemInstance()->getNumberOfQuadraticTerms();

		quadTerms.reserve(numQuadTerms);

		for (int i = 0; i < numQuadTerms; i++)
		{
			auto tmpTerm = getProblemInstance()->instanceData->quadraticCoefficients->qTerm[i];

			if (tmpTerm->idx == constrIdx) quadTerms.push_back(tmpTerm);
		}
	}

	return (quadTerms);
}

void OptProblem::fixVariable(int varIdx, double value)
{
	getProblemInstance()->instanceData->variables->var[varIdx]->lb = value;
	getProblemInstance()->instanceData->variables->var[varIdx]->ub = value;
}
