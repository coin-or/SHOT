/*
 * TaskInitializeDualSolver.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include "TaskInitializeDualSolver.h"

TaskInitializeDualSolver::TaskInitializeDualSolver(ES_MIPSolver solver, bool useLazyStrategy)
{
	ProcessInfo::getInstance().startTimer("MIP");

	if (useLazyStrategy)
	{
		if (solver == ES_MIPSolver::Cplex && Settings::getInstance().getBoolSetting("Cplex.UseNewCallbackType", "Subsolver"))
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverCplexLazy();
			ProcessInfo::getInstance().outputInfo("Cplex with lazy callbacks selected as MIP solver.");
		}
		else if (solver == ES_MIPSolver::Cplex)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverCplexLazyOriginalCallback();
			ProcessInfo::getInstance().outputInfo("Cplex with original lazy callbacks selected as MIP solver.");
		}
		else if (solver == ES_MIPSolver::Gurobi)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverGurobiLazy();
			ProcessInfo::getInstance().outputInfo("Gurobi with lazy callbacks selected as MIP solver.");
		}
		else if (solver == ES_MIPSolver::Cbc)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverOsiCbc();
			ProcessInfo::getInstance().outputInfo("Cbc selected as MIP solver.");
		}
		else
		{
			ProcessInfo::getInstance().outputError("Error in solver definition.");
			throw new ErrorClass("Error in MIP solver definition.");
		}
	}
	else
	{
		if (solver == ES_MIPSolver::Cplex)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverCplex();
			ProcessInfo::getInstance().outputInfo("Cplex selected as MIP solver.");
		}
		else if (solver == ES_MIPSolver::Gurobi)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverGurobi();
			ProcessInfo::getInstance().outputInfo("Gurobi selected as MIP solver.");
		}
		else if (solver == ES_MIPSolver::Cbc)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverOsiCbc();
			ProcessInfo::getInstance().outputInfo("Cbc selected as MIP solver.");
		}
		else
		{
			ProcessInfo::getInstance().outputError("Error in solver definition.");
			throw new ErrorClass("Error in MIP solver definition.");
		}
	}

	ProcessInfo::getInstance().stopTimer("MIP");
}

TaskInitializeDualSolver::~TaskInitializeDualSolver()
{
	// TODO Auto-generated destructor stub
}

void TaskInitializeDualSolver::run()
{

}
std::string TaskInitializeDualSolver::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

