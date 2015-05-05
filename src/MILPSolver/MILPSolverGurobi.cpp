#include "MILPSolverGurobi.h"

GRBEnv gurobiEnv = GRBEnv();
GRBModel gurobiModel = GRBModel(gurobiEnv);

MILPSolverGurobi::MILPSolverGurobi()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	discreteVariablesActivated = true;

	gurobiModel.getEnv().set(GRB_IntParam_OutputFlag, 0);
	//gurobiModel.getEnv().set(GRB_DoubleParam_FeasibilityTol, 1e-6);
	//gurobiModel.getEnv().set(GRB_DoubleParam_IntFeasTol, 1e-6);
	//gurobiModel.getEnv().set(GRB_DoubleParam_OptimalityTol, 1e-6);
	//gurobiModel.getEnv().set(GRB_DoubleParam_MarkowitzTol, 1e-4);
	//gurobiModel.getEnv().set(GRB_DoubleParam_NodeLimit, 1e15);
	gurobiModel.getEnv().set(GRB_IntParam_SolutionLimit, 2100000000);
	gurobiModel.getEnv().set(GRB_IntParam_SolutionNumber,
			settings->getIntSetting("MaxHyperplanesPerIteration", "Algorithm") + 1);
}

MILPSolverGurobi::~MILPSolverGurobi()
{
	delete &gurobiEnv;
	delete &gurobiModel;
}

bool MILPSolverGurobi::getDiscreteVariableStatus()
{
	return (MILPSolverBase::getDiscreteVariableStatus());
}

bool MILPSolverGurobi::createLinearProblem(OptProblem *origProblem)
{
	auto numVar = origProblem->getNumberOfVariables();

	auto tmpLBs = origProblem->getVariableLowerBounds();
	auto tmpUBs = origProblem->getVariableUpperBounds();
	auto tmpNames = origProblem->getVariableNames();

	auto tmpTypes = origProblem->getVariableTypes();

	for (int i = 0; i < numVar; i++)
	{
		//Variable *tmpVar = origInstance->instanceData->variables->var[i];

		//tmpUB = 10000000.0;

		if (tmpTypes.at(i) == 'C')
		{
			GRBVar tmpVar = gurobiModel.addVar(tmpLBs.at(i), tmpUBs.at(i), 0.0, GRB_CONTINUOUS, tmpNames.at(i));
		}
		else if (tmpTypes.at(i) == 'I')
		{
			GRBVar tmpVar = gurobiModel.addVar(tmpLBs.at(i), tmpUBs.at(i), 0.0, GRB_INTEGER, tmpNames.at(i));
		}
		else if (tmpTypes.at(i) == 'B')
		{
			GRBVar tmpVar = gurobiModel.addVar(tmpLBs.at(i), tmpUBs.at(i), 0.0, GRB_BINARY, tmpNames.at(i));
		}
		else
		{
			processInfo->logger.message(1) << "Error variable type for " << tmpNames.at(i).c_str() << CoinMessageEol;
		}
	}

	gurobiModel.update();

	auto tmpObjPairs = origProblem->getObjectiveFunctionVarCoeffPairs();

	gurobiModel.set(GRB_DoubleAttr_ObjCon, origProblem->getObjectiveConstant());

	if (origProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
	{
		GRBLinExpr *expr = new GRBLinExpr(0);

		for (int i = 0; i < tmpObjPairs.size(); i++)
		{
			*expr = *expr + (tmpObjPairs.at(i).second) * (gurobiModel.getVar(tmpObjPairs.at(i).first));
		}

		gurobiModel.setObjective(*expr);
	}
	else
	{
		GRBQuadExpr *expr = new GRBQuadExpr(0);

		for (int i = 0; i < tmpObjPairs.size(); i++)
		{
			*expr = *expr + (tmpObjPairs.at(i).second) * (gurobiModel.getVar(tmpObjPairs.at(i).first));
		}

		auto quadTerms = origProblem->getQuadraticTermsInConstraint(-1);

		for (auto T : quadTerms)
		{
			*expr = *expr
					+ (T->coef * gurobiModel.getVar(tmpObjPairs.at(T->idxOne).first)
							* gurobiModel.getVar(tmpObjPairs.at(T->idxTwo).first));
		}

		gurobiModel.setObjective(*expr);
	}

	gurobiModel.update();

	if (origProblem->isTypeOfObjectiveMinimize())
	{
		gurobiModel.set(GRB_IntAttr_ModelSense, 1);
	}
	else
	{
		gurobiModel.set(GRB_IntAttr_ModelSense, -1);
	}

	gurobiModel.update();

	int numCon = origProblem->getNumberOfConstraints();

	if (origProblem->isObjectiveFunctionNonlinear()) numCon--;

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
		if (!origProblem->isConstraintNonlinear(rowIdx))
		{
			auto quadTerms = origProblem->getQuadraticTermsInConstraint(rowIdx);

			if (quadTerms.size() == 0)
			{
				GRBLinExpr *expr = new GRBLinExpr(0);

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
						auto variable = gurobiModel.getVar(varIdx);

						*expr = *expr + val * variable;
					}
				}

				if (constrTypes[rowIdx] == 'L')
				{
					gurobiModel.addConstr(*expr <= constrUBs[rowIdx], constrNames[rowIdx]);
				}
				else if (constrTypes[rowIdx] == 'G')
				{
					gurobiModel.addConstr(*expr >= constrLBs[rowIdx], constrNames[rowIdx]);
				}
				else if (constrTypes[rowIdx] == 'E')
				{
					gurobiModel.addConstr(*expr == constrUBs[rowIdx], constrNames[rowIdx]);
				}
				else
				{
				}
			}
			else
			{
				GRBQuadExpr *expr = new GRBQuadExpr(0);

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
						auto variable = gurobiModel.getVar(varIdx);

						*expr = *expr + val * variable;
					}
				}

				for (auto T : quadTerms)
				{
					*expr = *expr + (T->coef * gurobiModel.getVar(T->idxOne) * gurobiModel.getVar(T->idxTwo));
				}

				if (constrTypes[rowIdx] == 'L')
				{
					gurobiModel.addConstr(*expr <= constrUBs[rowIdx], constrNames[rowIdx]);
				}
				else if (constrTypes[rowIdx] == 'G')
				{
					gurobiModel.addConstr(*expr >= constrLBs[rowIdx], constrNames[rowIdx]);
				}
				else if (constrTypes[rowIdx] == 'E')
				{
					gurobiModel.addConstr(*expr == constrUBs[rowIdx], constrNames[rowIdx]);
				}
				else
				{
				}
			}
		}
	}

	gurobiModel.update();

	return true;
}

bool MILPSolverGurobi::addLinearConstraint(std::vector<IndexValuePair> elements, int numNonZero, double constant)
{
	//auto nablag = osinstanceRefMINLP->calculateConstraintFunctionGradient(&point[0], constrIdx, true);

	// Sets the value g_i as the constant
	//double gkpp = osinstanceRefMINLP->calculateFunctionValue(constrIdx, &point[0], true);

	//cout << "Creating cutting plane: " << endl;

	GRBLinExpr *expr = new GRBLinExpr(0.0);

	//auto varNames = osinstanceRefMINLP->getVariableNames();

	for (int i = 0; i < numNonZero; i++)
	{
		// Inserts the term nabla g_i(x_i) * x_i to the cutting plane
		auto variable = gurobiModel.getVar(elements.at(i).idx);
		//cout << "Variable (gurobi): " << variable.get(GRB_StringAttr_VarName) << endl;
		//cout << "Variable (os): " << variable.get(GRB_StringAttr_VarName) << endl;

		//double nablaval = nablag->values[i];
		*expr = *expr + elements.at(i).value * variable;

		//cout << nablaval << "       " << variable.get(GRB_StringAttr_VarName) << endl;

		// Adds the value of -nabla g_i(x_i) * x_i  gradient to the constant
		//gkpp += -nablag->values[i] * point[nablag->indexes[i]];
		//cout << "   nablag" << nablag->indexes[i] << ": " << nablag->values[i] << endl;
	}

	// Adds the cutting plane
	//osi->addRow(cut, -osi->getInfinity(), -gkpp);
	gurobiModel.addConstr(*expr <= -constant, "");

	gurobiModel.update();

	//gurobiModel.write("test.lp");

	return true;
}

std::vector<double> MILPSolverGurobi::getVariableSolution()
{
	//int numVar = processInfo->originalProblem->getNumberOfVariables();
	int numVar = gurobiModel.get(GRB_IntAttr_NumVars);
	std::vector<double> solution(numVar);

	for (int i = 0; i < numVar; i++)
	{
		GRBVar tmpVar = gurobiModel.getVar(i);
		solution.at(i) = tmpVar.get(GRB_DoubleAttr_X);
	}

	return solution;
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
				GRBVar tmpVar = gurobiModel.getVar(i);

				tmpVar.set(GRB_CharAttr_VType, 'I');
			}
			else if (variableTypes.at(i) == 'B')
			{
				GRBVar tmpVar = gurobiModel.getVar(i);

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
				GRBVar tmpVar = gurobiModel.getVar(i);

				tmpVar.set(GRB_CharAttr_VType, 'C');
			}
		}

		discreteVariablesActivated = false;
	}

	gurobiModel.update();
}

E_ProblemSolutionStatus MILPSolverGurobi::getSolutionStatus()
{
	E_ProblemSolutionStatus MILPSolutionStatus;

	int status = gurobiModel.get(GRB_IntAttr_Status);

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

	return MILPSolutionStatus;
}

E_ProblemSolutionStatus MILPSolverGurobi::solveProblem()
{
	E_ProblemSolutionStatus MILPSolutionStatus;

	try
	{
		processInfo->logger.message(4) << " Solving MILP..." << CoinMessageEol;
		gurobiModel.optimize();
		processInfo->logger.message(4) << " MILP solved..." << CoinMessageEol;

		MILPSolutionStatus = getSolutionStatus();
	}
	catch (GRBException e)
	{
		processInfo->logger.message(2) << "Error code = " << e.getErrorCode() << CoinMessageEol;
		processInfo->logger.message(2) << e.getMessage() << CoinMessageEol;
		MILPSolutionStatus = E_ProblemSolutionStatus::Error;
	}

	return MILPSolutionStatus;
}

double MILPSolverGurobi::getLastObjectiveValue()
{
	return gurobiModel.get(GRB_DoubleAttr_ObjVal);
}

double MILPSolverGurobi::getBestObjectiveValue()
{
	return gurobiModel.get(GRB_DoubleAttr_ObjBound);
}

int MILPSolverGurobi::increaseSolutionLimit(int increment)
{
	gurobiModel.getEnv().set(GRB_IntParam_SolutionLimit,
			gurobiModel.getEnv().get(GRB_IntParam_SolutionLimit) + increment);

	return gurobiModel.getEnv().get(GRB_IntParam_SolutionLimit);
}

void MILPSolverGurobi::setSolutionLimit(int limit)
{
	gurobiModel.getEnv().set(GRB_IntParam_SolutionLimit, limit);
}

int MILPSolverGurobi::getSolutionLimit()
{
	return gurobiModel.getEnv().get(GRB_IntParam_SolutionLimit);
}

std::vector<SolutionPoint> MILPSolverGurobi::getAllVariableSolutions()
{
	std::vector < SolutionPoint > allSolutions;

	if (getDiscreteVariableStatus())
	{
		auto numSols = gurobiModel.get(GRB_IntAttr_SolCount);

		for (int i = 0; i < numSols; i++)
		{
			SolutionPoint tmpSolPt;

			int numVar = gurobiModel.get(GRB_IntAttr_NumVars);

			gurobiModel.getEnv().set(GRB_IntParam_SolutionNumber, i);

			vector<double> tmpPt(numVar);

			for (int i = 0; i < numVar; i++)
			{
				GRBVar tmpVar = gurobiModel.getVar(i);

				tmpPt.at(i) = tmpVar.get(GRB_DoubleAttr_Xn);
			}

			tmpSolPt.point = tmpPt;
			tmpSolPt.iterFound = processInfo->getCurrentIteration()->iterationNumber;
			tmpSolPt.objectiveValue = gurobiModel.get(GRB_DoubleAttr_ObjVal);

			allSolutions.push_back(tmpSolPt);
		}
	}
	else
	{
		SolutionPoint tmpSolPt;
		tmpSolPt.point = getVariableSolution();
		tmpSolPt.iterFound = processInfo->getCurrentIteration()->iterationNumber;
		tmpSolPt.objectiveValue = gurobiModel.get(GRB_DoubleAttr_ObjVal);

		allSolutions.push_back(tmpSolPt);
	}

	return (allSolutions);

}

void MILPSolverGurobi::setTimeLimit(double seconds)
{
	gurobiModel.getEnv().set(GRB_DoubleParam_TimeLimit, seconds);
}

void MILPSolverGurobi::setCutOff(double cutOff)
{
	gurobiModel.getEnv().set(GRB_DoubleParam_Cutoff, cutOff);

}

void MILPSolverGurobi::addMIPStart(std::vector<double> point)
{

}

void MILPSolverGurobi::writeProblemToFile(std::string filename)
{
	gurobiModel.write(filename);
}

void MILPSolverGurobi::changeConstraintToLazy(std::vector<int> constrIdxs)
{

}
