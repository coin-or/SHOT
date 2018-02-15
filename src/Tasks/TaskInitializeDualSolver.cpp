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
	
	if (solver != ES_MIPSolver::Cplex && solver != ES_MIPSolver::Gurobi && solver != ES_MIPSolver::Cbc)
	{
		ProcessInfo::getInstance().outputError("Error in solver definition. Check option 'Dual.MIP.Solver'.");
		throw new ErrorClass("Error in MIP solver definition.  Check option 'Dual.MIP.Solver'.");
	}

	if (useLazyStrategy)
	{
#ifdef HAS_CPLEX
		if (solver == ES_MIPSolver::Cplex)
		{
			if (Settings::getInstance().getBoolSetting("Cplex.UseNewCallbackType", "Subsolver"))
			{
				ProcessInfo::getInstance().MIPSolver = new MIPSolverCplexLazy();
				ProcessInfo::getInstance().usedMIPSolver = ES_MIPSolver::Cplex;
				ProcessInfo::getInstance().outputInfo("Cplex with lazy callbacks selected as MIP solver.");
			}
			else
			{
				ProcessInfo::getInstance().MIPSolver = new MIPSolverCplexLazyOriginalCallback();
				ProcessInfo::getInstance().usedMIPSolver = ES_MIPSolver::Cplex;
				ProcessInfo::getInstance().outputInfo("Cplex with original lazy callbacks selected as MIP solver.");
			}
		}
#endif

#ifdef HAS_GUROBI
		if (solver == ES_MIPSolver::Gurobi)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverGurobiLazy();
			ProcessInfo::getInstance().usedMIPSolver = ES_MIPSolver::Gurobi;
			ProcessInfo::getInstance().outputInfo("Gurobi with lazy callbacks selected as MIP solver.");
		}
#endif

		if (solver == ES_MIPSolver::Cbc)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverOsiCbc();
			ProcessInfo::getInstance().usedMIPSolver = ES_MIPSolver::Cbc;
			ProcessInfo::getInstance().outputInfo("Cbc selected as MIP solver.");
		}
	}
	else
	{

#ifdef HAS_CPLEX
		if (solver == ES_MIPSolver::Cplex)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverCplex();
			ProcessInfo::getInstance().outputInfo("Cplex selected as MIP solver.");
		}
#endif

#ifdef HAS_GUROBI
		if (solver == ES_MIPSolver::Gurobi)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverGurobi();
			ProcessInfo::getInstance().outputInfo("Gurobi selected as MIP solver.");
		}
#endif
		if (solver == ES_MIPSolver::Cbc)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverOsiCbc();
			ProcessInfo::getInstance().outputInfo("Cbc selected as MIP solver.");
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
