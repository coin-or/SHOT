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
			processInfo->outputWarning("Error variable type " + to_string(tmpTypes.at(i)) + " for " + tmpNames.at(i));
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
		}
	}

	osiInterface->loadFromCoinModel(*coinModel);
	cbcModel = new CbcModel(*osiInterface);
	CbcMain0 (*cbcModel);
	cbcModel->setLogLevel(0);
	osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);

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

	return (osiInterface->getNumRows() - 1);
}

void MILPSolverOsiCbc::activateDiscreteVariables(bool activate)
{
	auto variableTypes = processInfo->originalProblem->getVariableTypes();
	int numVar = processInfo->originalProblem->getNumberOfVariables();

	if (activate)
	{
		processInfo->outputDebug("Activating MILP strategy");

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
		processInfo->outputDebug("Activating LP strategy");
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
		processInfo->outputError("MILP solver return status unknown.");
	}

	return (MILPSolutionStatus);
}

E_ProblemSolutionStatus MILPSolverOsiCbc::solveProblem()
{
	E_ProblemSolutionStatus MILPSolutionStatus;
	cachedSolutionHasChanged = true;

	try
	{
		cbcModel->solver()->setHintParam(OsiDoReducePrint, false, OsiHintTry);

		cbcModel = new CbcModel(*osiInterface);
		cbcModel->setMaximumSolutions(solLimit);
		cbcModel->setMaximumSavedSolutions(solLimit);
		cbcModel->setCutoff(this->cutOff = cutOff);
		CbcMain0 (*cbcModel);
		cbcModel->setLogLevel(0);
		cbcModel->branchAndBound();

		MILPSolutionStatus = getSolutionStatus();

	}
	catch (exception &e)
	{
		processInfo->outputError(e.what());
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

void MILPSolverOsiCbc::setTimeLimit(double seconds)
{
}

void MILPSolverOsiCbc::setCutOff(double cutOff)
{
	try
	{
		this->cutOff = cutOff;

		if (processInfo->originalProblem->isTypeOfObjectiveMinimize())
		{
			processInfo->outputInfo("     Setting cutoff value to " + to_string(cutOff) + " for minimization.");
		}
		else
		{
			processInfo->outputInfo("     Setting cutoff value to " + to_string(cutOff) + " for maximization.");
		}
	}
	catch (exception &e)
	{
		processInfo->outputError("Error when setting cut off value", e.what());
	}
}

void MILPSolverOsiCbc::addMIPStart(std::vector<double> point)
{

}

void MILPSolverOsiCbc::writeProblemToFile(std::string filename)
{
	try
	{
		osiInterface->writeLp(filename.c_str(), "lp");
	}
	catch (exception &e)
	{
		processInfo->outputError("Error when saving model to file", e.what());
	}
}

double MILPSolverOsiCbc::getObjectiveValue(int solIdx)
{
	bool isMILP = getDiscreteVariableStatus();

	double objVal = NAN;

	if (!isMILP && solIdx > 0) // LP problems only have one solution!
	{
		processInfo->outputError(
				"Cannot obtain solution with index " + to_string(solIdx) + " since the problem is LP/QP!");

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
		processInfo->outputError("Error when obtaining objective value for solution index " + to_string(solIdx),
				e.what());

	}

	return (objVal);
}

void MILPSolverOsiCbc::changeConstraintToLazy(GeneratedHyperplane &hyperplane)
{
	processInfo->outputError("Lazy constraints not implemented in Cbc interface!");
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
		processInfo->outputError("Error when reading solution with index " + to_string(solIdx), e.what());
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
		processInfo->outputError("Error when obtaining number of solutions", e.what());
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

std::pair<std::vector<double>, std::vector<double> > MILPSolverOsiCbc::presolveAndGetNewBounds()
{
	return (std::make_pair(processInfo->originalProblem->getVariableLowerBounds(),
			processInfo->originalProblem->getVariableLowerBounds()));
}

void MILPSolverOsiCbc::writePresolvedToFile(std::string filename)
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
		processInfo->outputWarning(
				"Quadratic objective setting activated, but MIP solver does not support it. Resetting setting!");
	}
	else if (useQuadraticConstraint)
	{
		// MIP solver supports quadratic objectives but not quadratic constraints, reseting setting
		settings->updateSetting("QPStrategy", "Algorithm", (int) ES_QPStrategy::Nonlinear);
		processInfo->outputWarning(
				"Quadratic constraint setting activated, but MIP solver does not support it. Resetting setting!");
	}
}
