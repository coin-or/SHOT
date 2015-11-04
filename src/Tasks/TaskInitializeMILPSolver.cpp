/*
 * TaskInitializeMILPSolver.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include <TaskInitializeMILPSolver.h>

TaskInitializeMILPSolver::TaskInitializeMILPSolver()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("MILP");

	auto solver = static_cast<ES_MILPSolver>(settings->getIntSetting("MILPSolver", "MILP"));

	if (solver == ES_MILPSolver::Cplex)
	{
		processInfo->MILPSolver = new MILPSolverCplex();
		processInfo->logger.message(2) << "Cplex selected as MIP solver." << CoinMessageEol;
	}
	else if (solver == ES_MILPSolver::Gurobi)
	{
		processInfo->MILPSolver = new MILPSolverGurobi();
		processInfo->logger.message(2) << "Gurobi selected as MIP solver." << CoinMessageEol;
	}
	else if (solver == ES_MILPSolver::Cbc)
	{
		processInfo->MILPSolver = new MILPSolverOsiCbc();
		processInfo->logger.message(2) << "Cbc selected as MIP solver." << CoinMessageEol;
	}
	else
	{
		throw new ErrorClass("Error in MIP solver definition.");
	}

	processInfo->logger.message(1) << "MIP solver initialized" << CoinMessageEol;
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

