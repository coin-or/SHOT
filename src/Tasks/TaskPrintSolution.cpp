#include "TaskPrintSolution.h"

TaskPrintSolution::TaskPrintSolution()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskPrintSolution::~TaskPrintSolution()
{
}

void TaskPrintSolution::run()
{

	processInfo->stopTimer("Subproblems");

	auto tmpSol = processInfo->MILPSolver->getLastObjectiveValue();

	processInfo->logger.setPrecision(10);
	if (processInfo->terminationReason == E_TerminationReason::TimeLimit)
	{
		processInfo->logger.message(1)
				<< "=================================================================================="
				<< CoinMessageEol;
		//processInfo->logger.message(1) << "Time limit reached. Nonoptimal solution " << tmpSol << " found" << CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::IterationLimit)
	{
		processInfo->logger.message(1)
				<< "=================================================================================="
				<< CoinMessageEol;
		//processInfo->logger.message(1) << "Iteration limit reached. Nonoptimal solution " << tmpSol << " found" << CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::OptimalSolution)
	{
		processInfo->logger.message(1)
				<< "=================================================================================="
				<< CoinMessageEol;
		//processInfo->logger.message(1) << "Optimal solution" << tmpSol << " found" << CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::ObjectiveStagnation)
	{
		processInfo->logger.message(1)
				<< "=================================================================================="
				<< CoinMessageEol;
		//processInfo->logger.message(1) << "Solution" << tmpSol << " does not meet required tolerance!" << CoinMessageEol;
		processInfo->logger.message(1) << "MILP objective value has not changed in "
				<< processInfo->itersWithStagnationMILP << " iterations." << CoinMessageEol;

	}
	else if (processInfo->terminationReason == E_TerminationReason::InfeasibleProblem)
	{
		processInfo->logger.message(1)
				<< "=================================================================================="
				<< CoinMessageEol;
		processInfo->logger.message(1) << "Infeasible "
				<< (processInfo->getCurrentIteration()->isMILP() ? "MILP" : "LP") << " problem, check tolerances!"
				<< CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::Error)
	{
		processInfo->logger.message(1)
				<< "=================================================================================="
				<< CoinMessageEol;
		processInfo->logger.message(1) << "Error when solving "
				<< (processInfo->getCurrentIteration()->isMILP() ? "MILP" : "LP") << " problem!" << CoinMessageEol;
	}

	auto dualBound = processInfo->getDualBound();

	auto primalBound = processInfo->getPrimalBound();

	if (processInfo->originalProblem->isTypeOfObjectiveMinimize())
	{
		processInfo->logger.message(1) << "Dual bound: " << dualBound << " Primal bound: " << primalBound
				<< " (minimization)" << CoinMessageEol;
	}
	else
	{
		processInfo->logger.message(1) << "Primal bound: " << primalBound << " Dual bound: " << dualBound
				<< " (maximization)" << CoinMessageEol;
	}

	processInfo->logger.message(1) << "Relative duality gap: " << processInfo->getRelativeObjectiveGap()
			<< " Absolute duality gap: " << processInfo->getAbsoluteObjectiveGap() << CoinMessageEol;

	processInfo->logger.message(1)
			<< "==================================================================================" << CoinMessageEol;
	processInfo->logger.message(1) << "# optimal MILP problems solved:  " << processInfo->iterOptMILP << CoinMessageEol;
	processInfo->logger.message(1) << "# feasible MILP problems solved: " << processInfo->iterFeasMILP
			<< CoinMessageEol;
	processInfo->logger.message(1) << "# LP problems solved:            " << processInfo->iterLP << CoinMessageEol;
	processInfo->logger.message(1) << "# total problems solved:         "
			<< processInfo->iterOptMILP + processInfo->iterFeasMILP + processInfo->iterLP << CoinMessageEol;
	processInfo->logger.message(1)
			<< "==================================================================================" << CoinMessageEol;

}
