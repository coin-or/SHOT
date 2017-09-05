/*
 * TaskInitializeMILPSolver.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include "TaskInitializeMILPSolver.h"

TaskInitializeMILPSolver::TaskInitializeMILPSolver(OSInstance *originalInstance)
{
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	ProcessInfo::getInstance().startTimer("MILP");

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
		ProcessInfo::getInstance().MILPSolver = new MILPSolverCplex();
		ProcessInfo::getInstance().outputInfo("Cplex selected as MIP solver.");
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
	else if (solver == ES_MILPSolver::CplexExperimental)
	{
		if (isObjQuadratic && isQuadraticUsed)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverCplex();
			ProcessInfo::getInstance().outputInfo("Cplex selected as MIP solver.");
			settings->updateSetting("MILPSolver", "MILP", 0);
		}
		else if (useQuadraticConstraint && originalInstance->getNumberOfQuadraticTerms() > 0)
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverCplex();
			ProcessInfo::getInstance().outputInfo("Cplex selected as MIP solver.");
			settings->updateSetting("MILPSolver", "MILP", 0);
		}
		else
		{
			ProcessInfo::getInstance().MILPSolver = new MILPSolverCplexExperimental();
			ProcessInfo::getInstance().outputInfo("Cplex (lazy, experimental) selected as MIP solver.");
		}
	}
	else
	{
		throw new ErrorClass("Error in MIP solver definition.");
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

