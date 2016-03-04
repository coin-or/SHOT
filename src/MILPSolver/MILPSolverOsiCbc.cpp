#include "IMILPSolver.h"
#include "MILPSolverOsiCbc.h"
#include "CoinPragma.hpp"
#include "CbcModel.hpp"
#include "OsiClpSolverInterface.hpp"

#include <vector>

MILPSolverOsiCbc::MILPSolverOsiCbc()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	discreteVariablesActivated = true;

	cachedSolutionHasChanged = true;

	osiInterface = new OsiClpSolverInterface();

	checkParameters();
}

MILPSolverOsiCbc::~MILPSolverOsiCbc()
{
}

bool MILPSolverOsiCbc::createLinearProblem(OptProblem *origProblem)
{
	CoinModel *coinModel;
	coinModel = new CoinModel();

	auto numVar = origProblem->getNumberOfVariables();
	auto tmpLBs = origProblem->getVariableLowerBounds();
	auto tmpUBs = origProblem->getVariableUpperBounds();
	auto tmpNames = origProblem->getVariableNames();
	auto tmpTypes = origProblem->getVariableTypes();

	int numCon = origProblem->getNumberOfConstraints();
	if (origProblem->isObjectiveFunctionNonlinear()) numCon--; // Only want the number of original constraints and not the objective function

	// Now creating the variables
	for (int i = 0; i < numVar; i++)
	{
		coinModel->setColumnBounds(i, tmpLBs.at(i), tmpUBs.at(i));
		coinModel->setColName(i, tmpNames.at(i).c_str());

		if (tmpTypes.at(i) == 'C')
		{

		}
		else if (tmpTypes.at(i) == 'I' || tmpTypes.at(i) == 'B')
		{
			coinModel->setInteger(i);
		}
		else if (tmpTypes.at(i) == 'D')
		{

		}
		else
		{
			processInfo->logger.message(1) << "Error variable type " << tmpTypes.at(i) << " for "
					<< tmpNames.at(i).c_str() << CoinMessageEol;
		}
	}

	// Now creating the objective function

	auto tmpObjPairs = origProblem->getObjectiveFunctionVarCoeffPairs();

	for (int i = 0; i < tmpObjPairs.size(); i++)
	{
		coinModel->setColObjective(tmpObjPairs.at(i).first, tmpObjPairs.at(i).second);
	}

	// Add quadratic terms in the objective if they exist (and the strategy is to solve QPs)
	// Since this is not used for the Cbc solver here, it should never happen and quadratic objective functions
	// are regarded as general nonlinear
	if (origProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
	{
		auto quadTerms = origProblem->getQuadraticTermsInConstraint(-1);

		for (auto T : quadTerms)
		{
			coinModel->setQuadraticElement(T->idxOne, T->idxTwo, T->coef);
		}
	}

	double objConstant = origProblem->getObjectiveConstant();
	coinModel->setObjectiveOffset(objConstant);

	if (origProblem->isTypeOfObjectiveMinimize())
	{
		coinModel->setOptimizationDirection(1.0);
	}
	else
	{
		coinModel->setOptimizationDirection(-1.0);
	}

	// Now creating the constraints

	int row_nonz = 0;
	int obj_nonz = 0;
	int varIdx = 0;

	SparseMatrix *m_linearConstraintCoefficientsInRowMajor =
			origProblem->getProblemInstance()->getLinearConstraintCoefficientsInRowMajor();

	auto constrTypes = origProblem->getProblemInstance()->getConstraintTypes();
	auto constrNames = origProblem->getProblemInstance()->getConstraintNames();
	auto constrLBs = origProblem->getProblemInstance()->getConstraintLowerBounds();
	auto constrUBs = origProblem->getProblemInstance()->getConstraintUpperBounds();

//try
//{
	for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
	{
		// Only use constraints that don't contain a nonlinear part (may include a quadratic part)
		if (!origProblem->isConstraintNonlinear(rowIdx))
		{
			if (origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients != NULL
					&& origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients->numberOfValues
							> 0)
			{
				row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1]
						- m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];

				for (int j = 0; j < row_nonz; j++)
				{
					double val =
							m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx]
									+ j];
					varIdx =
							m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx]
									+ j];

					coinModel->setElement(rowIdx, varIdx, val);
				}
			}

			// Add quadratic terms if they exist and have been defined as quadratic and not nonlinear
			//auto quadTerms = origProblem->getQuadraticTermsInConstraint(rowIdx);

			/* TODO: not implemented
			 for (auto T : quadTerms)
			 {
			 expr += T->coef * cplexVars[T->idxOne] * cplexVars[T->idxTwo];
			 }*/

			double rowConstant = origProblem->getProblemInstance()->instanceData->constraints->con[rowIdx]->constant;

			coinModel->setRowName(rowIdx, constrNames[rowIdx].c_str());

			// Add the constraint
			if (constrTypes[rowIdx] == 'L')
			{
				coinModel->setRowUpper(rowIdx, constrUBs[rowIdx] - rowConstant);
			}
			else if (constrTypes[rowIdx] == 'G')
			{
				coinModel->setRowLower(rowIdx, constrLBs[rowIdx] - rowConstant);
			}
			else if (constrTypes[rowIdx] == 'E')
			{
				coinModel->setRowBounds(rowIdx, constrLBs[rowIdx] - rowConstant, constrUBs[rowIdx] - rowConstant);
			}
			else
			{
			}
			//coinModel->
		}
	}

//osiSolver->loadFromCoinModel(*coinModel);
//cbcModel = CbcModel(*osiSolver);

////OsiClpSolverInterface osiModel;

//int numLinConstrs = origInstance->getConstraintNumber() - origInstance->getNumberOfNonlinearConstraints();
//int numNonLinConstrs = origInstance->getNumberOfNonlinearConstraints();
//int numCon = origInstance->getConstraintNumber();
////int *idxNonlinear = origInstance->getNonlinearExpressionTreeIndexes();

//setVariableCharacteristics(origInstance);

//origInstance->initForAlgDiff();

//// Copies the linear constraints to the OSI instance
//int row_nonz = 0;
//int obj_nonz = 0;
//int varIdx = 0;
//SparseMatrix *m_linearConstraintCoefficientsInRowMajor = origInstance->getLinearConstraintCoefficientsInRowMajor();

//CoinPackedMatrix * matrix = new CoinPackedMatrix(false, 0, 0);
//matrix->setDimensions(0, numVariables);

//std::vector<double>rowLBs;
//std::vector<double>rowUBs;

//for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
//{
//	// Only use constraints that don't contain a nonlinear part
//	if (origInstance->getNonlinearExpressionTree(rowIdx) == NULL)
//	{
//		if (origInstance->instanceData->linearConstraintCoefficients != NULL &&
//			origInstance->instanceData->linearConstraintCoefficients->numberOfValues > 0)
//		{
//			row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1] - m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];

//			//vector<int> varIndexes;
//			//vector<double> varElements;

//			CoinPackedVector row;

//			for (int j = 0; j < row_nonz; j++)
//			{
//				varIdx = m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
//				row.insert(varIdx, m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j]);
//			}

//			matrix->appendRow(row);

//			// Adds the bounds of the rows
//			rowLBs.push_back(origInstance->instanceData->constraints->con[rowIdx]->lb);
//			rowUBs.push_back(origInstance->instanceData->constraints->con[rowIdx]->ub);
//		}
//	}
//}

//// Gets the objective function coefficients in the right form
////SparseVector *objectiveCoeffs = origInstance->getObjectiveCoefficients()[0];
////vector<double> objectiveCoeffsDbl(objectiveCoeffs->number);
////std::copy(objectiveCoeffs->values, objectiveCoeffs->values + objectiveCoeffs->number, objectiveCoeffsDbl.begin());

//auto dense = origInstance->getDenseObjectiveCoefficients()[0];
////vector<double> objectiveCoeffsDbl(objectiveCoeffs->number);
////std::copy(objectiveCoeffs->values, objectiveCoeffs->values + objectiveCoeffs->number, objectiveCoeffsDbl.begin());

//// Adds everything to the OSI instance
///*auto test = origInstance->getVariableLowerBounds();
//auto test2 = origInstance->getVariableUpperBounds();
//auto test3 = &rowLBs[0];
//auto test4 = &rowUBs[0];
//*/

//osiModel.loadProblem(*matrix, origInstance->getVariableLowerBounds(),
//	origInstance->getVariableUpperBounds(), &dense[0], &rowLBs[0], &rowUBs[0]);

//// Adds the variables
//for (int i = 0; i < numVariables; i++)
//{
//	Variable *tmpVar = origInstance->instanceData->variables->var[i];

//	// Sets the variable name
//	osiModel.setColName(i, tmpVar->name);

//	if (tmpVar->type == 'C')
//	{
//		osiModel.setContinuous(i);
//	}
//	else if (tmpVar->type == 'I' || tmpVar->type == 'B')
//	{
//		osiModel.setInteger(i);
//	}
//	else
//	{
//		processInfo->logger.message(1) << "ERROR in variable definition!" << CoinMessageEol;
//	}
//}

//osiModel.writeLp("c:\\test", "lp");

//osiModel.initialSolve();

	osiInterface->loadFromCoinModel(*coinModel);
	cbcModel = new CbcModel(*osiInterface);
	CbcMain0 (*cbcModel);
	cbcModel->setLogLevel(0);
	osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);

	//cbcModel->initialSolve();

	/*cbcModel = new CbcModel(*osiInterface);
	 CbcMain0 (*cbcModel);
	 cbcModel->setLogLevel(0);
	 cbcModel->solver()->setHintParam(OsiDoReducePrint, false, OsiHintTry);*/
	return (true);
}

void MILPSolverOsiCbc::initializeSolverSettings()
{

}

int MILPSolverOsiCbc::addLinearConstraint(std::vector<IndexValuePair> elements, double constant, bool isGreaterThan)
{
	CoinPackedVector cut;

	for (int i = 0; i < elements.size(); i++)
	{
		cut.insert(elements.at(i).idx, elements.at(i).value);
	}

	// Adds the cutting plane
	if (isGreaterThan) osiInterface->addRow(cut, -constant, osiInterface->getInfinity());
	else osiInterface->addRow(cut, -osiInterface->getInfinity(), -constant);

	/*
	 //CoinPackedVector cut;
	 int rowNum = coinModel->numberRows();

	 for (int i = 0; i < elements.size(); i++)
	 {
	 coinModel->setElement(rowNum, elements.at(i).idx, elements.at(i).value);
	 }

	 coinModel->setRowUpper(rowNum, -constant);
	 */
//osiSolver->loadFromCoinModel(*coinModel);
//cbcModel = CbcModel(*osiSolver);
	return (osiInterface->getNumRows() - 1);
}

void MILPSolverOsiCbc::activateDiscreteVariables(bool activate)
{
	auto variableTypes = processInfo->originalProblem->getVariableTypes();
	int numVar = processInfo->originalProblem->getNumberOfVariables();

	if (activate)
	{
		processInfo->logger.message(3) << "Activating MILP strategy" << CoinMessageEol;

		for (int i = 0; i < numVar; i++)
		{
			if (variableTypes.at(i) == 'I' || variableTypes.at(i) == 'B')
			{
				osiInterface->setInteger(i);
			}
		}

		discreteVariablesActivated = true;
	}
	else
	{
		processInfo->logger.message(3) << "Activating LP strategy" << CoinMessageEol;
		for (int i = 0; i < numVar; i++)
		{
			if (variableTypes.at(i) == 'I' || variableTypes.at(i) == 'B')
			{
				osiInterface->setContinuous(i);
			}
		}

		discreteVariablesActivated = false;
	}

}

E_ProblemSolutionStatus MILPSolverOsiCbc::getSolutionStatus()
{
	E_ProblemSolutionStatus MILPSolutionStatus;

	/*if (status == GRB_LOADED)
	 {
	 MILPSolutionStatus = EMILPStatus::loaded;
	 }*/
	if (cbcModel->isProvenOptimal())
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::Optimal;
	}
	else if (cbcModel->isProvenInfeasible())
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
	}
	/*else if (status == GRB_INF_OR_UNBD)
	 {
	 MILPSolutionStatus = EMILPStatus::inf_or_unbd;
	 }
	 else if (status == GRB_UNBOUNDED)
	 {
	 MILPSolutionStatus = EMILPStatus::unbounded;
	 }
	 else if (status == GRB_CUTOFF)
	 {
	 MILPSolutionStatus = EMILPStatus::cutoff;
	 }*/
	else if (cbcModel->isSolutionLimitReached())
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
	}
	else if (cbcModel->isSecondsLimitReached())
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
	}
	/*else if (osiModel.is)
	 {
	 MILPSolutionStatus = EMILPStatus::node_limit;
	 }
	 else if (status == GRB_TIME_LIMIT)
	 {
	 MILPSolutionStatus = EMILPStatus::time_limit;
	 }
	 else if (osiModel.sol)
	 {
	 MILPSolutionStatus = EMILPStatus::solution_limit;
	 }
	 else if (status == GRB_INTERRUPTED)
	 {
	 MILPSolutionStatus = EMILPStatus::interrupted;
	 }
	 else if (status == GRB_NUMERIC)
	 {
	 MILPSolutionStatus = EMILPStatus::numeric;
	 }
	 else if (status == GRB_SUBOPTIMAL)
	 {
	 MILPSolutionStatus = EMILPStatus::suboptimal;
	 }
	 else if (status == GRB_INPROGRESS)
	 {
	 MILPSolutionStatus = EMILPStatus::in_progress;
	 }*/
	else
	{
		processInfo->logger.message(0) << "MILP solver return status unknown." << CoinMessageEol;
	}

	return (MILPSolutionStatus);
}

E_ProblemSolutionStatus MILPSolverOsiCbc::solveProblem()
{
	E_ProblemSolutionStatus MILPSolutionStatus;
	cachedSolutionHasChanged = true;

	try
	{
		processInfo->logger.message(4) << " Solving MILP..." << CoinMessageEol;
		//osiInterface->loadFromCoinModel(*coinModel);

		/*
		 cbcModel = new CbcModel(*osiInterface);
		 CbcMain0 (*cbcModel);
		 cbcModel->setLogLevel(0);
		 cbcModel->solver()->setHintParam(OsiDoReducePrint, false, OsiHintTry);
		 */
		//cbcModel->initialSolve();
		//CbcMain0 (*cbcModel);
		cbcModel->solver()->setHintParam(OsiDoReducePrint, false, OsiHintTry);

		cbcModel = new CbcModel(*osiInterface);
		cbcModel->setMaximumSolutions(solLimit);
		cbcModel->setMaximumSavedSolutions(solLimit);
		cbcModel->setCutoff(this->cutOff = cutOff);
		CbcMain0 (*cbcModel);
		cbcModel->setLogLevel(0);
		cbcModel->branchAndBound();
		processInfo->logger.message(4) << " MILP solved..." << CoinMessageEol;

		MILPSolutionStatus = getSolutionStatus();

	}
	catch (exception &e)
	{
		processInfo->logger.message(0) << e.what() << CoinMessageEol;
		MILPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	return (MILPSolutionStatus);
}

int MILPSolverOsiCbc::increaseSolutionLimit(int increment)
{
	this->solLimit += increment;

	cbcModel->setMaximumSolutions(this->solLimit);
	cbcModel->setMaximumSavedSolutions(this->solLimit);

	return (this->solLimit);
}

void MILPSolverOsiCbc::setSolutionLimit(int limit)
{
	this->solLimit = limit;
	cbcModel->setMaximumSolutions(limit);
	cbcModel->setMaximumSavedSolutions(limit);
}

int MILPSolverOsiCbc::getSolutionLimit()
{
	return (this->solLimit);
}

/*std::vector<SolutionPoint> MILPSolverOsiCbc::getAllVariableSolutions()
 {
 std::vector < SolutionPoint > allSolutions;
 return allSolutions;
 }*/

/*
 std::vector<SolutionPoint> MILPSolverOsiCbc::getAllVariableSolutions()
 {
 return (MILPSolverBase::getAllVariableSolutions());
 }*/

void MILPSolverOsiCbc::setTimeLimit(double seconds)
{
}

void MILPSolverOsiCbc::setCutOff(double cutOff)
{
	try
	{
		//cbcModel->setCutoff(cutOff);
		this->cutOff = cutOff;

		if (processInfo->originalProblem->isTypeOfObjectiveMinimize())
		{
			processInfo->logger.message(3) << "Setting cutoff value to " << cutOff << " for minimization."
					<< CoinMessageEol;
		}
		else
		{

			processInfo->logger.message(3) << "Setting cutoff value to " << cutOff << " for maximization."
					<< CoinMessageEol;
		}
	}
	catch (exception &e)
	{
		processInfo->logger.message(0) << "Error when setting cut off value:" << CoinMessageNewline << e.what()
				<< CoinMessageEol;

	}
}

void MILPSolverOsiCbc::addMIPStart(std::vector<double> point)
{

}

void MILPSolverOsiCbc::writeProblemToFile(std::string filename)
{
	try
	{
		//osiInterface->loadFromCoinModel(*coinModel);
		osiInterface->writeLp(filename.c_str(), "lp");
	}
	catch (exception &e)
	{
		processInfo->logger.message(0) << "Error when saving model to file:" << CoinMessageNewline << e.what()
				<< CoinMessageEol;

	}
}

double MILPSolverOsiCbc::getObjectiveValue(int solIdx)
{

	bool isMILP = getDiscreteVariableStatus();

	double objVal = NAN;

	if (!isMILP && solIdx > 0) // LP problems only have one solution!
	{
		processInfo->logger.message(0) << "Cannot obtain solution with index " << solIdx
				<< " since the problem is LP/QP!" << CoinMessageEol;

		return (objVal);
	}

	try
	{
		// Fixes some strange behavior with the objective value when solving MILPs vs LPs
		if (isMILP && processInfo->originalProblem->isTypeOfObjectiveMinimize())
		{
			objVal = 1.0;
		}
		else if (isMILP && !processInfo->originalProblem->isTypeOfObjectiveMinimize())
		{
			objVal = -1.0;
		}
		else
		{
			objVal = 1.0;
		}

		if (isMILP)
		{
			objVal *= cbcModel->savedSolutionObjective(solIdx);
		}
		else
		{
			objVal *= cbcModel->getObjValue();
		}

	}
	catch (exception &e)
	{
		processInfo->logger.message(0) << "Error when obtaining objective value for solution index " << solIdx << ":"
				<< CoinMessageNewline << e.what() << CoinMessageEol;

	}

//std::cout << "Obj: " << objVal << std::endl;
	return (objVal);
}

void MILPSolverOsiCbc::changeConstraintToLazy(GeneratedHyperplane &hyperplane)
{

}

void MILPSolverOsiCbc::deleteMIPStarts()
{
}

std::vector<double> MILPSolverOsiCbc::getVariableSolution(int solIdx)
{
	bool isMILP = getDiscreteVariableStatus();
	int numVar = processInfo->originalProblem->getNumberOfVariables();
	std::vector<double> solution(numVar);

	try
	{
		if (isMILP)
		{
			auto tmpSol = cbcModel->savedSolution(solIdx);
			for (int i = 0; i < numVar; i++)
			{
				solution.at(i) = tmpSol[i];
			}
		}
		else
		{
			auto tmpSol = cbcModel->bestSolution();

			for (int i = 0; i < numVar; i++)
			{
				solution.at(i) = tmpSol[i];
			}
		}

	}
	catch (exception&e)
	{
		processInfo->logger.message(0) << "Error when reading solution with index " << solIdx << ":"
				<< CoinMessageNewline << e.what() << CoinMessageEol;
	}
	return (solution);
}

int MILPSolverOsiCbc::getNumberOfSolutions()
{
	int numSols = 0;
	bool isMILP = getDiscreteVariableStatus();

	try
	{
		if (isMILP) numSols = cbcModel->getSolutionCount();
		else numSols = 1;
	}
	catch (exception &e)
	{
		processInfo->logger.message(0) << "Error when obtaining number of solutions:" << CoinMessageNewline << e.what()
				<< CoinMessageEol;

	}

	return (numSols);
}

void MILPSolverOsiCbc::fixVariable(int varIndex, double value)
{
}

void MILPSolverOsiCbc::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
}

pair<double, double> MILPSolverOsiCbc::getCurrentVariableBounds(int varIndex)
{
	pair<double, double> tmpBounds;

	tmpBounds.first = NAN;
	tmpBounds.second = NAN;

	return (tmpBounds);
}

void MILPSolverOsiCbc::populateSolutionPool()
{
	return;
}

bool MILPSolverOsiCbc::supportsQuadraticObjective()
{
	return (false);
}

bool MILPSolverOsiCbc::supportsQuadraticConstraints()
{
	return (false);
}

double MILPSolverOsiCbc::getDualObjectiveValue()
{
}

bool MILPSolverOsiCbc::supportsLazyConstraints()
{
	return (false);
}

void MILPSolverOsiCbc::checkParameters()
{
// Checks if quadratic objective functions or constraints are allowed in the settings, and corrects
// it since we do not support this for Cbc.

	bool useQuadraticObjective = (static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm")))
			== ES_QPStrategy::QuadraticObjective;

	bool useQuadraticConstraint = (static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm")))
			== ES_QPStrategy::QuadraticallyConstrained;

	if (useQuadraticObjective)
	{
		// MIP solver does not support quadratic objectives, reseting both settings
		settings->updateSetting("QPStrategy", "Algorithm", (int) ES_QPStrategy::Nonlinear);
		processInfo->logger.message(0)
				<< "Quadratic objective setting activated, but MIP solver does not support it. Resetting setting!"
				<< CoinMessageEol;
	}
	else if (useQuadraticConstraint)
	{
		// MIP solver supports quadratic objectives but not quadratic constraints, reseting setting
		settings->updateSetting("QPStrategy", "Algorithm", (int) ES_QPStrategy::Nonlinear);
		processInfo->logger.message(0)
				<< "Quadratic constraint setting activated, but MIP solver does not support it. Resetting setting!"
				<< CoinMessageEol;
	}
}

