/*
 * TaskInitializeMILPSolver.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include "TaskInitializeMILPSolver.h"

TaskInitializeMILPSolver::TaskInitializeMILPSolver(OSInstance *originalInstance)
{

	ProcessInfo::getInstance().startTimer("MILP");

	bool useQuadraticObjective = (static_cast<ES_QPStrategy>(Settings::getInstance().getIntSetting("QPStrategy",
			"Algorithm"))) == ES_QPStrategy::QuadraticObjective;

	bool useQuadraticConstraint = (static_cast<ES_QPStrategy>(Settings::getInstance().getIntSetting("QPStrategy",
			"Algorithm"))) == ES_QPStrategy::QuadraticallyConstrained;

	bool useLazyStrategy = (static_cast<ES_SolutionStrategy>(Settings::getInstance().getIntSetting("SolutionStrategy",
			"Algorithm"))) == ES_SolutionStrategy::Lazy;

	bool isObjNonlinear = UtilityFunctions::isObjectiveGenerallyNonlinear(originalInstance);
	bool isObjQuadratic = UtilityFunctions::isObjectiveQuadratic(originalInstance);
	bool isQuadraticUsed = (useQuadraticObjective || (useQuadraticConstraint));

	if (isObjQuadratic && isQuadraticUsed)
	{
		useLazyStrategy = false;
	}
	else if (useQuadraticConstraint && originalInstance->getNumberOfQuadraticTerms() > 0)
	{
		useLazyStrategy = false;
	}

	auto solver = static_cast<ES_MILPSolver>(Settings::getInstance().getIntSetting("MILPSolver", "MILP"));

	if (useLazyStrategy)
	{
		if (solver == ES_MILPSolver::Cplex)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverCplexLazy();
			ProcessInfo::getInstance().outputInfo("Cplex (lazy) selected as MIP solver.");
		}
		else if (solver == ES_MILPSolver::Gurobi)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverGurobi();
			ProcessInfo::getInstance().outputInfo("Gurobi selected as MIP solver.");
		}
		else if (solver == ES_MILPSolver::Cbc)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverOsiCbc();
			ProcessInfo::getInstance().outputInfo("Cbc selected as MIP solver.");
		}
		else
		{
			throw new ErrorClass("Error in MIP solver definition.");
		}
	}
	else
	{
		if (solver == ES_MILPSolver::Cplex)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverCplex();
			ProcessInfo::getInstance().outputInfo("Cplex (lazy) selected as MIP solver.");
		}
		else if (solver == ES_MILPSolver::Gurobi)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverGurobi();
			ProcessInfo::getInstance().outputInfo("Gurobi selected as MIP solver.");
		}
		else if (solver == ES_MILPSolver::Cbc)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverOsiCbc();
			ProcessInfo::getInstance().outputInfo("Cbc selected as MIP solver.");
		}
		else
		{
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

