#include "MIPSolverGurobi.h"

MIPSolverGurobi::MIPSolverGurobi()
{
	discreteVariablesActivated = true;

	gurobiEnv = new GRBEnv();
	gurobiModel = new GRBModel(*gurobiEnv);

	cachedSolutionHasChanged = true;
	isVariablesFixed = false;

	checkParameters();
}

MIPSolverGurobi::~MIPSolverGurobi()
{
	delete gurobiEnv;
	delete gurobiModel;
}

bool MIPSolverGurobi::createLinearProblem(OptProblem *origProblem)
{
	originalProblem = origProblem;

	try
	{
		auto numVar = origProblem->getNumberOfVariables();
		auto tmpLBs = origProblem->getVariableLowerBounds();
		auto tmpUBs = origProblem->getVariableUpperBounds();
		auto tmpNames = origProblem->getVariableNames();
		auto tmpTypes = origProblem->getVariableTypes();

		int numCon = origProblem->getNumberOfConstraints();
		if (origProblem->isObjectiveFunctionNonlinear()) numCon--;

		for (int i = 0; i < numVar; i++)
		{
			if (tmpTypes.at(i) == 'C')
			{
				GRBVar tmpVar = gurobiModel->addVar(tmpLBs.at(i), tmpUBs.at(i), 0.0, GRB_CONTINUOUS, tmpNames.at(i));
			}
			else if (tmpTypes.at(i) == 'I')
			{
				GRBVar tmpVar = gurobiModel->addVar(tmpLBs.at(i), tmpUBs.at(i), 0.0, GRB_INTEGER, tmpNames.at(i));
			}
			else if (tmpTypes.at(i) == 'B')
			{
				GRBVar tmpVar = gurobiModel->addVar(tmpLBs.at(i), tmpUBs.at(i), 0.0, GRB_BINARY, tmpNames.at(i));
			}
			else if (tmpTypes.at(i) == 'D')
			{
				GRBVar tmpVar = gurobiModel->addVar(tmpLBs.at(i), tmpUBs.at(i), 0.0, GRB_SEMICONT, tmpNames.at(i));
			}
			else
			{
				ProcessInfo::getInstance().outputWarning(
						"Error variable type " + std::to_string(tmpTypes.at(i)) + " for " + tmpNames.at(i));
			}
		}

		gurobiModel->update();

		auto tmpObjPairs = origProblem->getObjectiveFunctionVarCoeffPairs();

		gurobiModel->set(GRB_DoubleAttr_ObjCon, origProblem->getObjectiveConstant());

		if (origProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
		{
			GRBLinExpr *expr = new GRBLinExpr(0);

			for (int i = 0; i < tmpObjPairs.size(); i++)
			{
				*expr += +(tmpObjPairs.at(i).second) * (gurobiModel->getVar(tmpObjPairs.at(i).first));
			}

			double objConstant = origProblem->getObjectiveConstant();
			if (objConstant != 0.0) *expr += objConstant;

			gurobiModel->setObjective(*expr);
		}
		else
		{
			GRBQuadExpr *expr = new GRBQuadExpr(0);

			for (int i = 0; i < tmpObjPairs.size(); i++)
			{
				*expr += (tmpObjPairs.at(i).second) * (gurobiModel->getVar(tmpObjPairs.at(i).first));
			}

			auto quadTerms = origProblem->getQuadraticTermsInConstraint(-1);

			for (auto T : quadTerms)
			{
				*expr += (T->coef * gurobiModel->getVar(T->idxOne) * gurobiModel->getVar(T->idxTwo));
			}

			double objConstant = origProblem->getObjectiveConstant();
			if (objConstant != 0.0) *expr += objConstant;

			gurobiModel->setObjective(*expr);
		}

		gurobiModel->update();

		if (origProblem->isTypeOfObjectiveMinimize())
		{
			gurobiModel->set(GRB_IntAttr_ModelSense, 1);
		}
		else
		{
			gurobiModel->set(GRB_IntAttr_ModelSense, -1);
		}

		gurobiModel->update();

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

		for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
		{
			// Only use constraints that don't contain a nonlinear part (may include a quadratic part)
			if (!origProblem->isConstraintNonlinear(rowIdx))
			{
				auto quadTerms = origProblem->getQuadraticTermsInConstraint(rowIdx);

				if (quadTerms.size() == 0)
				{
					GRBLinExpr *expr = new GRBLinExpr(0);
					*expr += origProblem->getProblemInstance()->instanceData->constraints->con[rowIdx]->constant;

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
							auto variable = gurobiModel->getVar(varIdx);

							*expr += val * variable;
						}
					}
					if (constrTypes[rowIdx] == 'L')
					{
						gurobiModel->addConstr(*expr <= constrUBs[rowIdx], constrNames[rowIdx]);
					}
					else if (constrTypes[rowIdx] == 'G')
					{
						gurobiModel->addConstr(*expr >= constrLBs[rowIdx], constrNames[rowIdx]);
					}
					else if (constrTypes[rowIdx] == 'E')
					{
						gurobiModel->addConstr(*expr == constrUBs[rowIdx], constrNames[rowIdx]);
					}
					else
					{
					}
				}
				else
				{
					GRBQuadExpr *expr = new GRBQuadExpr(0);
					*expr += origProblem->getProblemInstance()->instanceData->constraints->con[rowIdx]->constant;

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
							auto variable = gurobiModel->getVar(varIdx);

							*expr += val * variable;
						}
					}

					for (auto T : quadTerms)
					{
						*expr += (T->coef * gurobiModel->getVar(T->idxOne) * gurobiModel->getVar(T->idxTwo));
					}

					if (constrTypes[rowIdx] == 'L')
					{
						gurobiModel->addQConstr(*expr <= constrUBs[rowIdx], constrNames[rowIdx]);
					}
					else if (constrTypes[rowIdx] == 'G')
					{
						gurobiModel->addQConstr(*expr >= constrLBs[rowIdx], constrNames[rowIdx]);
					}
					else if (constrTypes[rowIdx] == 'E')
					{
						gurobiModel->addQConstr(*expr == constrUBs[rowIdx], constrNames[rowIdx]);
					}
					else
					{
					}
				}
			}
		}

		gurobiModel->update();
	}
	catch (GRBException &e)
	{
		{
			ProcessInfo::getInstance().outputError("Error when creating linear problem:", e.getMessage());
		}

		return (false);
	}

	return (true);
}

void MIPSolverGurobi::initializeSolverSettings()
{
	try
	{
		gurobiModel->getEnv().set(GRB_DoubleParam_MIPGap, Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination")/2.0);
		gurobiModel->getEnv().set(GRB_DoubleParam_MIPGapAbs, Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination")/2.0);
		//gurobiModel->getEnv().set(GRB_IntParam_NumericFocus,3);
		gurobiModel->getEnv().set(GRB_IntParam_OutputFlag, 0);
		//gurobiModel->getEnv().set(GRB_DoubleParam_FeasibilityTol, 1e-6);
		//gurobiModel->getEnv().set(GRB_DoubleParam_IntFeasTol, 1e-6);
		//gurobiModel->getEnv().set(GRB_DoubleParam_OptimalityTol, 1e-6);
		//gurobiModel->getEnv().set(GRB_DoubleParam_MarkowitzTol, 1e-4);
		//gurobiModel->getEnv().set(GRB_DoubleParam_NodeLimit, 1e15);
		gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
		gurobiModel->getEnv().set(GRB_IntParam_SolutionNumber,
				Settings::getInstance().getIntSetting("MIP.SolutionPool.Capacity", "Dual") + 1);
	}
	catch (GRBException &e)
	{
		{
			ProcessInfo::getInstance().outputError("Error when initializing parameters for linear solver",
					e.getMessage());
		}
	}
}

int MIPSolverGurobi::addLinearConstraint(std::vector<IndexValuePair> elements, double constant, bool isGreaterThan)
{
	try
	{
		GRBLinExpr *expr = new GRBLinExpr(0.0);

		for (int i = 0; i < elements.size(); i++)
		{
			auto variable = gurobiModel->getVar(elements.at(i).idx);
			*expr = *expr + elements.at(i).value * variable;
		}

		if (isGreaterThan)
		{
			gurobiModel->addConstr(-constant >= *expr, "");
		}
		else
		{
			gurobiModel->addConstr(*expr <= -constant, "");
		}

		gurobiModel->update();
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when adding linear constraint", e.getMessage());

		return (-1);

	}

	return (gurobiModel->get(GRB_IntAttr_NumConstrs) - 1);
}

std::vector<double> MIPSolverGurobi::getVariableSolution(int solIdx)
{
	bool isMIP = getDiscreteVariableStatus();

	int numVar = gurobiModel->get(GRB_IntAttr_NumVars);
	std::vector<double> solution(numVar);

	try
	{
		if (isMIP && solIdx > 0)
		{
			gurobiModel->getEnv().set(GRB_IntParam_SolutionNumber, solIdx);

			for (int i = 0; i < numVar; i++)
			{
				GRBVar tmpVar = gurobiModel->getVar(i);
				solution.at(i) = tmpVar.get(GRB_DoubleAttr_Xn);
			}
		}
		else
		{
			for (int i = 0; i < numVar; i++)
			{
				GRBVar tmpVar = gurobiModel->getVar(i);
				solution.at(i) = tmpVar.get(GRB_DoubleAttr_X);
			}
		}
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when reading solution with index " + std::to_string(solIdx),
				e.getMessage());
	}

	return (solution);
}

int MIPSolverGurobi::getNumberOfSolutions()
{
	int numSols = 0;

	numSols = gurobiModel->get(GRB_IntAttr_SolCount);

	return (numSols);
}

void MIPSolverGurobi::activateDiscreteVariables(bool activate)
{
	auto variableTypes = originalProblem->getVariableTypes();
	int numVar = originalProblem->getNumberOfVariables();

	if (activate)
	{
		ProcessInfo::getInstance().outputInfo("Activating MIP strategy.");

		for (int i = 0; i < numVar; i++)
		{
			if (variableTypes.at(i) == 'I')
			{
				GRBVar tmpVar = gurobiModel->getVar(i);

				tmpVar.set(GRB_CharAttr_VType, 'I');
			}
			else if (variableTypes.at(i) == 'B')
			{
				GRBVar tmpVar = gurobiModel->getVar(i);

				tmpVar.set(GRB_CharAttr_VType, 'B');
			}
		}

		discreteVariablesActivated = true;
	}
	else
	{
		ProcessInfo::getInstance().outputInfo("Activating LP strategy.");
		for (int i = 0; i < numVar; i++)
		{
			if (variableTypes.at(i) == 'I' || variableTypes.at(i) == 'B')
			{
				GRBVar tmpVar = gurobiModel->getVar(i);

				tmpVar.set(GRB_CharAttr_VType, 'C');
			}
		}

		discreteVariablesActivated = false;
	}

	gurobiModel->update();
}

E_ProblemSolutionStatus MIPSolverGurobi::getSolutionStatus()
{
	E_ProblemSolutionStatus MIPSolutionStatus;

	int status = gurobiModel->get(GRB_IntAttr_Status);

//if (status == GRB_LOADED)
//{
//}
//else
	if (status == GRB_OPTIMAL)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
	}
	else if (status == GRB_INFEASIBLE)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
	}
	else if (status == GRB_INF_OR_UNBD)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
	}
	else if (status == GRB_UNBOUNDED)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
	}
//else if (status == GRB_CUTOFF)
//{
//}
	else if (status == GRB_ITERATION_LIMIT)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::IterationLimit;
	}
	else if (status == GRB_NODE_LIMIT)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::IterationLimit;
	}
	else if (status == GRB_TIME_LIMIT)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
	}
	else if (status == GRB_SOLUTION_LIMIT)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
	}
	else if (status == GRB_INTERRUPTED)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
	}
//else if (status == GRB_NUMERIC)
//{
//}
	else if (status == GRB_CUTOFF)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::CutOff;
	}
	else if (status == GRB_SUBOPTIMAL)
	{
		MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;
	}
//else if (status == GRB_INPROGRESS)
//{
//}
	else
	{
		ProcessInfo::getInstance().outputError("MIP solver return status " + to_string(status));
		MIPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	return (MIPSolutionStatus);
}

E_ProblemSolutionStatus MIPSolverGurobi::solveProblem()
{
	E_ProblemSolutionStatus MIPSolutionStatus;
	cachedSolutionHasChanged = true;

	try
	{
		gurobiModel->optimize();

		MIPSolutionStatus = getSolutionStatus();
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when solving MIP/LP problem", e.getMessage());
		MIPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	return (MIPSolutionStatus);
}

int MIPSolverGurobi::increaseSolutionLimit(int increment)
{
	gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit,
			gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit) + increment);

	return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobi::setSolutionLimit(long limit)
{
	if (limit > GRB_MAXINT) gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
	else gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, limit);
}

int MIPSolverGurobi::getSolutionLimit()
{
	return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobi::setTimeLimit(double seconds)
{
	try
	{
		if (seconds > 0)
		{
			gurobiModel->getEnv().set(GRB_DoubleParam_TimeLimit, seconds);
		}
		else
		{
			gurobiModel->getEnv().set(GRB_DoubleParam_TimeLimit, 0.00001);
		}
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when setting time limit", e.getMessage());
	}

}

void MIPSolverGurobi::setCutOff(double cutOff)
{
	try
	{
		// Gurobi has problems if not an epsilon value is added to the cutoff...

		if (originalProblem->isTypeOfObjectiveMinimize())
		{
			gurobiModel->getEnv().set(GRB_DoubleParam_Cutoff, cutOff + 0.0000001);

			ProcessInfo::getInstance().outputInfo(
					"     Setting cutoff value to " + to_string(cutOff) + " for minimization.");
		}
		else
		{
			gurobiModel->getEnv().set(GRB_DoubleParam_Cutoff, cutOff - 0.0000001);
			ProcessInfo::getInstance().outputInfo(
					"     Setting cutoff value to " + to_string(cutOff) + " for maximization.");
		}
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when setting cut off value", e.getMessage());
	}
}

void MIPSolverGurobi::addMIPStart(std::vector<double> point)
{
	int numVar = gurobiModel->get(GRB_IntAttr_NumVars);
	std::vector<double> solution(numVar);

	try
	{
		for (int i = 0; i < numVar; i++)
		{
			GRBVar tmpVar = gurobiModel->getVar(i);
			tmpVar.set(GRB_DoubleAttr_Start, point.at(i));
		}

	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when adding MIP starting point", e.getMessage());
	}

	ProcessInfo::getInstance().outputInfo("      Added MIP starting point.");
}

void MIPSolverGurobi::writeProblemToFile(std::string filename)
{
	try
	{
		gurobiModel->write(filename);
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when saving model to file", e.getMessage());
	}
}

double MIPSolverGurobi::getObjectiveValue(int solIdx)
{
	bool isMIP = getDiscreteVariableStatus();

	double objVal = NAN;

	if (!isMIP && solIdx > 0) // LP problems only have one solution!
	{
		ProcessInfo::getInstance().outputError(
				"Cannot obtain solution with index " + to_string(solIdx) + " since the problem is LP/QP!");

		return (objVal);
	}

	try
	{
		if ((isMIP && solIdx == 0) || !isMIP)
		{
			objVal = gurobiModel->get(GRB_DoubleAttr_ObjVal);
		}
		else // Gurobi has no functionality to access the objective value of a specific solution
		{
			gurobiModel->getEnv().set(GRB_IntParam_SolutionNumber, solIdx);

			int numvars = gurobiModel->get(GRB_IntAttr_NumVars);

			auto objective = gurobiModel->getObjective();
			objVal = objective.getLinExpr().getConstant();

			for (int i = 0; i < objective.size(); i++)
			{
				objVal += objective.getCoeff(i) * objective.getVar1(i).get(GRB_DoubleAttr_Xn)
						* objective.getVar2(i).get(GRB_DoubleAttr_Xn);
			}

			auto linexpr = objective.getLinExpr();

			for (int i = 0; i < linexpr.size(); i++)
			{
				objVal += linexpr.getCoeff(i) * linexpr.getVar(i).get(GRB_DoubleAttr_Xn);
			}
		}
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError(
				"Error when obtaining objective value for solution index " + to_string(solIdx), e.getMessage());
	}

	return (objVal);

}

void MIPSolverGurobi::deleteMIPStarts()
{
	int numVar = gurobiModel->get(GRB_IntAttr_NumVars);
	std::vector<double> solution(numVar);

	try
	{
		for (int i = 0; i < numVar; i++)
		{
			GRBVar tmpVar = gurobiModel->getVar(i);
			tmpVar.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
		}

	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when deleting MIP starting points", e.getMessage());
	}

	ProcessInfo::getInstance().outputDebug("    Deleted MIP starting points.");
}

void MIPSolverGurobi::populateSolutionPool()
{
}

void MIPSolverGurobi::fixVariable(int varIndex, double value)
{
	updateVariableBound(varIndex, value, value);
}

void MIPSolverGurobi::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
	try
	{
		GRBVar tmpVar = gurobiModel->getVar(varIndex);

		tmpVar.set(GRB_DoubleAttr_LB, lowerBound);
		tmpVar.set(GRB_DoubleAttr_UB, upperBound);

		gurobiModel->update();
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError(
				"Error when updating variable bounds for variable index" + to_string(varIndex), e.getMessage());
	}
}

pair<double, double> MIPSolverGurobi::getCurrentVariableBounds(int varIndex)
{
	pair<double, double> tmpBounds;

	try
	{
		GRBVar tmpVar = gurobiModel->getVar(varIndex);

		tmpBounds.first = tmpVar.get(GRB_DoubleAttr_LB);
		tmpBounds.second = tmpVar.get(GRB_DoubleAttr_UB);

		gurobiModel->update();
	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError(
				"Error when obtaining variable bounds for variable index" + to_string(varIndex), e.getMessage());
	}

	return (tmpBounds);
}

bool MIPSolverGurobi::supportsQuadraticObjective()
{
	return (true);
}
bool MIPSolverGurobi::supportsQuadraticConstraints()
{
	return (true);
}

double MIPSolverGurobi::getDualObjectiveValue()
{

	bool isMIP = getDiscreteVariableStatus();
	double objVal = NAN;

	try
	{
		objVal = gurobiModel->get(GRB_DoubleAttr_ObjBound);

	}
	catch (GRBException &e)
	{
		ProcessInfo::getInstance().outputError("Error when obtaining dual objective value", e.getMessage());

	}

	return (objVal);
}

void MIPSolverGurobi::writePresolvedToFile(std::string filename)
{
}

void MIPSolverGurobi::checkParameters()
{

}

std::pair<std::vector<double>, std::vector<double> > MIPSolverGurobi::presolveAndGetNewBounds()
{
	return (std::make_pair(originalProblem->getVariableLowerBounds(), originalProblem->getVariableLowerBounds()));
}
