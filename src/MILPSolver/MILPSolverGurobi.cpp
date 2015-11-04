#include "MILPSolverGurobi.h"

GRBEnv gurobiEnv = GRBEnv();
GRBModel gurobiModel = GRBModel(gurobiEnv);

MILPSolverGurobi::MILPSolverGurobi()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	discreteVariablesActivated = true;

	gurobiEnv = new GRBEnv();
	gurobiModel = new GRBModel(*gurobiEnv);

	checkParameters();
}

MILPSolverGurobi::~MILPSolverGurobi()
{
	delete gurobiEnv;
	delete gurobiModel;
}

bool MILPSolverGurobi::createLinearProblem(OptProblem *origProblem)
{
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
			//Variable *tmpVar = origInstance->instanceData->variables->var[i];

			//tmpUB = 10000000.0;

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
			else
			{
				processInfo->logger.message(1) << "Error variable type for " << tmpNames.at(i).c_str()
						<< CoinMessageEol;
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

			}
		}

		gurobiModel->update();
	}
	catch (GRBException &e)
	{
		{
			processInfo->logger.message(0) << "Error when creating linear problem:" << CoinMessageNewline
					<< e.getMessage() << CoinMessageEol;
		}

		return (false);
	}

	return (true);
}

void MILPSolverGurobi::initializeSolverSettings()
{
	try
	{
		gurobiModel->getEnv().set(GRB_IntParam_OutputFlag, 0);
		//gurobiModel->getEnv().set(GRB_DoubleParam_FeasibilityTol, 1e-6);
		//gurobiModel->getEnv().set(GRB_DoubleParam_IntFeasTol, 1e-6);
		//gurobiModel->getEnv().set(GRB_DoubleParam_OptimalityTol, 1e-6);
		//gurobiModel->getEnv().set(GRB_DoubleParam_MarkowitzTol, 1e-4);
		//gurobiModel->getEnv().set(GRB_DoubleParam_NodeLimit, 1e15);
		gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, 2100000000);
		gurobiModel->getEnv().set(GRB_IntParam_SolutionNumber,
				settings->getIntSetting("MaxHyperplanesPerIteration", "Algorithm") + 1);
	}
	catch (GRBException &e)
	{
		{
			processInfo->logger.message(0) << "Error when initializing parameters for linear solver:"
					<< CoinMessageNewline << e.getMessage() << CoinMessageEol;
		}
	}
}

int MILPSolverGurobi::addLinearConstraint(std::vector<IndexValuePair> elements, double constant, bool isGreaterThan)
{
	try
	{
		GRBLinExpr *expr = new GRBLinExpr(0.0);

		for (int i = 0; i < elements.size(); i++)
		{
			auto variable = gurobiModel->getVar(elements.at(i).idx);
			*expr = *expr + elements.at(i).value * variable;
		}

		/*if (settings->getBoolSetting("UseLazyConstraints", "MILP")) // Not implemented yet in Gurobi
		 {
		 if (discreteVariablesActivated)
		 {
		 gurobiModel->addConstr(*expr <= -constant, "");
		 }
		 else
		 {
		 gurobiModel->addConstr(*expr <= -constant, "");
		 }
		 }
		 else
		 {*/

		if (isGreaterThan)
		{
			gurobiModel->addConstr(-constant >= *expr, "");
		}
		else
		{
			gurobiModel->addConstr(*expr <= -constant, "");
		}

		//}

		gurobiModel->update();
	}
	catch (GRBException &e)
	{
		{
			processInfo->logger.message(0) << "Error when adding linear constraint:" << CoinMessageNewline
					<< e.getMessage() << CoinMessageEol;
		}

		return (-1);

	}

	return (gurobiModel->get(GRB_IntAttr_NumConstrs) - 1);
}

std::vector<double> MILPSolverGurobi::getVariableSolution(int solIdx)
{
	bool isMILP = getDiscreteVariableStatus();

	int numVar = gurobiModel->get(GRB_IntAttr_NumVars);
	std::vector<double> solution(numVar);

	try
	{
		if (isMILP && solIdx > 0)
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
		processInfo->logger.message(0) << "Error when reading solution with index " << solIdx << ":"
				<< CoinMessageNewline << e.getMessage() << CoinMessageEol;
	}

	return (solution);
}

int MILPSolverGurobi::getNumberOfSolutions()
{
	int numSols = 0;

	numSols = gurobiModel->get(GRB_IntAttr_SolCount);

	return (numSols);
}

void MILPSolverGurobi::activateDiscreteVariables(bool activate)
{
	auto variableTypes = processInfo->originalProblem->getVariableTypes();
	int numVar = processInfo->originalProblem->getNumberOfVariables();

	if (activate)
	{
		processInfo->logger.message(3) << "Activating MILP strategy" << CoinMessageEol;

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
		processInfo->logger.message(3) << "Activating LP strategy" << CoinMessageEol;
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

E_ProblemSolutionStatus MILPSolverGurobi::getSolutionStatus()
{
	E_ProblemSolutionStatus MILPSolutionStatus;

	int status = gurobiModel->get(GRB_IntAttr_Status);

//if (status == GRB_LOADED)
//{
//	MILPSolutionStatus = E_ProblemSolutionStatus::::loaded;
//}
//else
	if (status == GRB_OPTIMAL)
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::Optimal;
	}
	else if (status == GRB_INFEASIBLE)
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
	}
	else if (status == GRB_INF_OR_UNBD)
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
	}
	else if (status == GRB_UNBOUNDED)
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
	}
//else if (status == GRB_CUTOFF)
//{
//	MILPSolutionStatus = EMILPStatus::cutoff;
//}
	else if (status == GRB_ITERATION_LIMIT)
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::IterationLimit;
	}
	else if (status == GRB_NODE_LIMIT)
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::IterationLimit;
	}
	else if (status == GRB_TIME_LIMIT)
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
	}
	else if (status == GRB_SOLUTION_LIMIT)
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
	}
//else if (status == GRB_INTERRUPTED)
//{
//	MILPSolutionStatus = EMILPStatus::interrupted;
//}
//else if (status == GRB_NUMERIC)
//{
//	MILPSolutionStatus = EMILPStatus::numeric;
//}
	else if (status == GRB_SUBOPTIMAL)
	{
		MILPSolutionStatus = E_ProblemSolutionStatus::Feasible;
	}
//else if (status == GRB_INPROGRESS)
//{
//	MILPSolutionStatus = EMILPStatus::in_progress;
//}
	else
	{
		processInfo->logger.message(1) << "MILP solver return status unknown = " << status << CoinMessageEol;
	}

	return (MILPSolutionStatus);
}

E_ProblemSolutionStatus MILPSolverGurobi::solveProblem()
{
	E_ProblemSolutionStatus MILPSolutionStatus;
	cachedSolutionHasChanged = true;

	try
	{
		processInfo->logger.message(4) << " Solving MILP..." << CoinMessageEol;
		gurobiModel->optimize();
		processInfo->logger.message(4) << " MILP solved..." << CoinMessageEol;

		MILPSolutionStatus = getSolutionStatus();
	}
	catch (GRBException &e)
	{
		processInfo->logger.message(2) << "Error code = " << e.getErrorCode() << CoinMessageEol;
		processInfo->logger.message(2) << e.getMessage() << CoinMessageEol;
		MILPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	return (MILPSolutionStatus);
}

int MILPSolverGurobi::increaseSolutionLimit(int increment)
{
	gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit,
			gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit) + increment);

	return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MILPSolverGurobi::setSolutionLimit(int limit)
{
	gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, limit);
}

int MILPSolverGurobi::getSolutionLimit()
{
	return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MILPSolverGurobi::setTimeLimit(double seconds)
{
	gurobiModel->getEnv().set(GRB_DoubleParam_TimeLimit, seconds);
}

void MILPSolverGurobi::setCutOff(double cutOff)
{
	try
	{
		gurobiModel->getEnv().set(GRB_DoubleParam_Cutoff, cutOff);

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
	catch (GRBException &e)
	{
		processInfo->logger.message(0) << "Error when setting cut off value:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;

	}
}

void MILPSolverGurobi::addMIPStart(std::vector<double> point)
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
		processInfo->logger.message(0) << "Error when adding MIP starting point:" << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;
	}

	processInfo->logger.message(3) << "    Added MIP starting point" << CoinMessageEol;
}

void MILPSolverGurobi::writeProblemToFile(std::string filename)
{
	try
	{
		gurobiModel->write(filename);
	}
	catch (GRBException &e)
	{
		processInfo->logger.message(0) << "Error when saving model to file:" << CoinMessageNewline << e.getMessage()
				<< CoinMessageEol;
	}
}

double MILPSolverGurobi::getObjectiveValue(int solIdx)
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
		if ((isMILP && solIdx == 0) || !isMILP)
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
		processInfo->logger.message(0) << "Error when obtaining objective value for solution index " << solIdx << ":"
				<< CoinMessageNewline << e.getMessage() << CoinMessageEol;

	}

	return (objVal);

}

void MILPSolverGurobi::changeConstraintToLazy(GeneratedHyperplane &hyperplane)
{

}

void MILPSolverGurobi::deleteMIPStarts()
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
		processInfo->logger.message(0) << "Error when removing MIP starting point:" << CoinMessageNewline
				<< e.getMessage() << CoinMessageEol;
	}

	processInfo->logger.message(3) << "    Removed MIP starting point" << CoinMessageEol;
}

void MILPSolverGurobi::populateSolutionPool()
{
}

void MILPSolverGurobi::fixVariable(int varIndex, double value)
{
	updateVariableBound(varIndex, value, value);
}

void MILPSolverGurobi::updateVariableBound(int varIndex, double lowerBound, double upperBound)
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
		processInfo->logger.message(0) << "Error when updating variable bounds for variable index" << varIndex << ":"
				<< CoinMessageNewline << e.getMessage() << CoinMessageEol;
	}
}

pair<double, double> MILPSolverGurobi::getCurrentVariableBounds(int varIndex)
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
		processInfo->logger.message(0) << "Error when obtaining variable bounds for variable index" << varIndex << ":"
				<< CoinMessageNewline << e.getMessage() << CoinMessageEol;
	}

	return (tmpBounds);
}

bool MILPSolverGurobi::supportsQuadraticObjective()
{
	return (true);
}
bool MILPSolverGurobi::supportsQuadraticConstraints()
{
	return (true);
}

bool MILPSolverGurobi::supportsLazyConstraints()
{
	return (false);
}

void MILPSolverGurobi::checkParameters()
{

}
