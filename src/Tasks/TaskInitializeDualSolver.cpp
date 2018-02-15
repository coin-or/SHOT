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
std::cout << "0" << std::endl;
	if (solver != ES_MIPSolver::Cplex && solver != ES_MIPSolver::Gurobi && solver != ES_MIPSolver::Cbc)
	{
		std::cout << "1" << std::endl;
		ProcessInfo::getInstance().outputError("Error in solver definition. Check option 'Dual.MIP.Solver'.");
		throw new ErrorClass("Error in MIP solver definition.  Check option 'Dual.MIP.Solver'.");
	}

	if (useLazyStrategy)
	{
		std::cout << "2" << std::endl;
#ifdef HAS_CPLEX
std::cout << "cplex" << std::endl;
		std::cout << "cplex" << std::endl;
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
std::cout << "gurobi" << std::endl;
		if (solver == ES_MIPSolver::Gurobi)
		{
			ProcessInfo::getInstance().MIPSolver = new MIPSolverGurobiLazy();
			ProcessInfo::getInstance().usedMIPSolver = ES_MIPSolver::Gurobi;
			ProcessInfo::getInstance().outputInfo("Gurobi with lazy callbacks selected as MIP solver.");
		}
#endif

		std::cout << "4" << std::endl;
		if (solver == ES_MIPSolver::Cbc)
		{
			std::cout << "6" << std::endl;
			ProcessInfo::getInstance().MIPSolver = new MIPSolverOsiCbc();
			ProcessInfo::getInstance().usedMIPSolver = ES_MIPSolver::Cbc;
			ProcessInfo::getInstance().outputInfo("Cbc selected as MIP solver.");
		}
	}
	else
	{
std::cout << "3" << std::endl;
#ifdef HAS_CPLEX
		if (solver == ES_MIPSolver::Cplex)
		{
			std::cout << "cplex2" << std::endl;
			ProcessInfo::getInstance().MIPSolver = new MIPSolverCplex();
			ProcessInfo::getInstance().outputInfo("Cplex selected as MIP solver.");
		}
#endif

#ifdef HAS_GUROBI
		if (solver == ES_MIPSolver::Gurobi)
		{
			std::cout << "gurobi2" << std::endl;
			ProcessInfo::getInstance().MIPSolver = new MIPSolverGurobi();
			ProcessInfo::getInstance().outputInfo("Gurobi selected as MIP solver.");
		}
#endif
		std::cout << "5" << std::endl;
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
