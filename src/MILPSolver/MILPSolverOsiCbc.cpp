#include "IMILPSolver.h"
#include "MILPSolverOsiCbc.h"
#include "CoinPragma.hpp"
#include "CbcModel.hpp"
#include "OsiClpSolverInterface.hpp"

#include <vector>

OsiClpSolverInterface osiModel;

MILPSolverOsiCbc::MILPSolverOsiCbc()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	discreteVariablesActivated = false;
}

MILPSolverOsiCbc::~MILPSolverOsiCbc()
{
	delete &osiModel;
}

bool MILPSolverOsiCbc::createLinearProblem(OptProblem *origProblem)
{
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

	return true;
}

bool MILPSolverOsiCbc::getDiscreteVariableStatus()
{
	return (MILPSolverBase::getDiscreteVariableStatus());
}

bool MILPSolverOsiCbc::addLinearConstraint(std::vector<IndexValuePair> elements, int numNonZero, double constant)
{
	CoinPackedVector cut;

	for (int i = 0; i < numNonZero; i++)
	{
		cut.insert(elements.at(i).idx, elements.at(i).value);
	}

	// Adds the cutting plane
	osiModel.addRow(cut, -osiModel.getInfinity(), -constant);

	osiModel.writeLp("c:\\test", "lp", 0.0000000001, 10, 10, 1, true);
	//osiModel.writeMps("c:\\test.mps");

	return true;
}

std::vector<double> MILPSolverOsiCbc::getVariableSolution()
{
	int numCol = osiModel.getNumCols();

	const double * solution = osiModel.getColSolution();

	std::vector<double> sols(numCol);
	std::copy(solution, solution + numCol, sols.begin());

	delete solution;

	return sols;
}

void MILPSolverOsiCbc::activateDiscreteVariables(bool activate)
{
	/*if (activate)
	 {
	 processInfo->logger.message(3) << "Setting MILP strategy" << CoinMessageEol;
	 for (int i = 0; i < numVariables; i++)
	 {
	 if (variableTypes[i] == 'I')
	 {
	 osiModel.setInteger(i);
	 }
	 else if (variableTypes[i] == 'B')
	 {
	 osiModel.setInteger(i);
	 }
	 }

	 discreteVariablesActivated = true;
	 }
	 else
	 {
	 processInfo->logger.message(3) << "Setting LP strategy" << CoinMessageEol;
	 for (int i = 0; i < numVariables; i++)
	 {
	 if (variableTypes[i] == 'I')
	 {
	 osiModel.setContinuous(i);
	 }
	 else if (variableTypes[i] == 'B')
	 {
	 osiModel.setContinuous(i);
	 }
	 }

	 discreteVariablesActivated = false;
	 }*/
}

E_ProblemSolutionStatus MILPSolverOsiCbc::getSolutionStatus()
{
	E_ProblemSolutionStatus MILPSolutionStatus;

	/*if (status == GRB_LOADED)
	 {
	 MILPSolutionStatus = EMILPStatus::loaded;
	 }*/
	if (osiModel.isProvenOptimal())
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::Optimal;
	}
	else if (osiModel.isProvenPrimalInfeasible())
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
	else if (osiModel.isIterationLimitReached())
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::IterationLimit;
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

	return MILPSolutionStatus;
}

E_ProblemSolutionStatus MILPSolverOsiCbc::solveProblem()
{
	E_ProblemSolutionStatus MILPSolutionStatus;
	//osiModel.resolve();

	CbcModel model(osiModel);
	//model.setIntegerTolerance(0.0001);
	//model.setTypePresolve(0);
	model.branchAndBound();
	osiModel = *dynamic_cast<OsiClpSolverInterface*>(model.solver());

	MILPSolutionStatus = getSolutionStatus();

	return MILPSolutionStatus;
}

double MILPSolverOsiCbc::getLastObjectiveValue()
{
	return osiModel.getObjValue();
}

double MILPSolverOsiCbc::getBestObjectiveValue()
{
	return osiModel.getObjValue();
}

int MILPSolverOsiCbc::increaseSolutionLimit(int increment)
{
	/*
	 osiModel.setIntParam();
	 gurobiModel.getEnv().set(GRB_IntParam_SolutionLimit, gurobiModel.getEnv().get(GRB_IntParam_SolutionLimit) + increment);
	 */
	return 1;
}

void MILPSolverOsiCbc::setSolutionLimit(int limit)
{
}

int MILPSolverOsiCbc::getSolutionLimit()
{
	return 100;
}

std::vector<std::vector<double>> MILPSolverOsiCbc::getAllVariableSolutions()
{

	std::vector < std::vector<double> > allSolutions;
	return allSolutions;
}

void MILPSolverOsiCbc::setTimeLimit(double seconds)
{
}

void MILPSolverOsiCbc::setCutOff(double cutOff)
{

}

void MILPSolverOsiCbc::addMIPStart(std::vector<double> point)
{

}

void MILPSolverOsiCbc::writeProblemToFile(std::string filename)
{
}

void MILPSolverOsiCbc::changeConstraintToLazy(std::vector<int> constrIdxs)
{

}
