/*
 * TaskInitializeMILPSolver.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include "TaskInitializeMILPSolver.h"

TaskInitializeMILPSolver::TaskInitializeMILPSolver(ES_MIPSolver solver, bool useLazyStrategy)
{
	ProcessInfo::getInstance().startTimer("MILP");

	if (useLazyStrategy)
	{
		if (solver == ES_MIPSolver::Cplex && Settings::getInstance().getBoolSetting("UseNewCallbackType", "CPLEX"))
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverCplexLazy();
			ProcessInfo::getInstance().outputInfo("Cplex with lazy callbacks selected as MIP solver.");
		}
		else if (solver == ES_MIPSolver::Cplex)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverCplexLazyOriginalCB();
			ProcessInfo::getInstance().outputInfo("Cplex with original lazy callbacks selected as MIP solver.");
		}
		else if (solver == ES_MIPSolver::Gurobi)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverGurobiLazy();
			ProcessInfo::getInstance().outputInfo("Gurobi with lazy callbacks selected as MIP solver.");
		}
		else if (solver == ES_MIPSolver::Cbc)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverOsiCbc();
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
			ProcessInfo::getInstance().MILPSolver = new MILPSolverCplex();
			ProcessInfo::getInstance().outputInfo("Cplex selected as MIP solver.");
		}
		else if (solver == ES_MIPSolver::Gurobi)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverGurobi();
			ProcessInfo::getInstance().outputInfo("Gurobi selected as MIP solver.");
		}
		else if (solver == ES_MIPSolver::Cbc)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverOsiCbc();
			ProcessInfo::getInstance().outputInfo("Cbc selected as MIP solver.");
		}
		else
		{
			ProcessInfo::getInstance().outputError("Error in solver definition.");
			throw new ErrorClass("Error in MIP solver definition.");
		}
	}

	ProcessInfo::getInstance().stopTimer("MILP");
}

TaskInitializeMILPSolver::~TaskInitializeMILPSolver()
{
	// TODO Auto-generated destructor stub
}

void TaskInitializeMILPSolver::run()
{

}
std::string TaskInitializeMILPSolver::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

