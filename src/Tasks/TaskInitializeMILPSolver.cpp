/*
 * TaskInitializeMILPSolver.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include <TaskInitializeMILPSolver.h>

TaskInitializeMILPSolver::TaskInitializeMILPSolver(OSInstance *originalInstance)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("MILP");

	bool useQuadraticObjective = (static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm")))
			== ES_QPStrategy::QuadraticObjective;

	bool useQuadraticConstraint = (static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm")))
			== ES_QPStrategy::QuadraticallyConstrained;

	bool isObjNonlinear = UtilityFunctions::isObjectiveGenerallyNonlinear(originalInstance);
	bool isObjQuadratic = UtilityFunctions::isObjectiveQuadratic(originalInstance);
	bool isQuadraticUsed = (useQuadraticObjective || (useQuadraticConstraint));

	auto solver = static_cast<ES_MILPSolver>(settings->getIntSetting("MILPSolver", "MILP"));

	if (solver == ES_MILPSolver::Cplex)
	{
		processInfo->MILPSolver = new MILPSolverCplex();
		processInfo->outputInfo("Cplex selected as MIP solver.");
	}
	else if (solver == ES_MILPSolver::Gurobi)
	{
		processInfo->MILPSolver = new MILPSolverGurobi();
		processInfo->outputInfo("Gurobi selected as MIP solver.");
	}
	else if (solver == ES_MILPSolver::Cbc)
	{
		processInfo->MILPSolver = new MILPSolverOsiCbc();
		processInfo->outputInfo("Cbc selected as MIP solver.");
	}
	else if (solver == ES_MILPSolver::CplexExperimental)
	{
		if (isObjQuadratic && isQuadraticUsed)
		{
			processInfo->MILPSolver = new MILPSolverCplex();
			processInfo->outputInfo("Cplex selected as MIP solver.");
			settings->updateSetting("MILPSolver", "MILP", 0);
		}
		else if (useQuadraticConstraint && originalInstance->getNumberOfQuadraticTerms() > 0)
		{
			processInfo->MILPSolver = new MILPSolverCplex();
			processInfo->outputInfo("Cplex selected as MIP solver.");
			settings->updateSetting("MILPSolver", "MILP", 0);
		}
		else
		{
			processInfo->MILPSolver = new MILPSolverCplexExperimental();
			processInfo->outputInfo("Cplex (lazy, experimental) selected as MIP solver.");
		}
	}
	else
	{
		throw new ErrorClass("Error in MIP solver definition.");
	}

	processInfo->stopTimer("MILP");
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

