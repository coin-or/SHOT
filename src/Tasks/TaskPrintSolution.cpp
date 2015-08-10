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

	auto tmpSol = processInfo->MILPSolver->getObjectiveValue();
	processInfo->logger.message(1)
			<< "==================================================================================" << CoinMessageEol;

	processInfo->logger.setPrecision(10);
	if (processInfo->terminationReason == E_TerminationReason::ConstraintTolerance)
	{
		processInfo->logger.message(1) << "Optimal solution " << tmpSol << " found to constraint tolerance "
				<< processInfo->getCurrentIteration()->maxDeviation << " <= "
				<< settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm") << CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::AbsoluteGap)
	{
		processInfo->logger.message(1) << "Optimal solution " << tmpSol << " found to absolute gap tolerance "
				<< processInfo->getAbsoluteObjectiveGap() << " <= "
				<< settings->getDoubleSetting("GapTermTolAbsolute", "Algorithm") << CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::RelativeGap)
	{
		processInfo->logger.message(1) << "Optimal solution " << tmpSol << " found to relative gap tolerance "
				<< processInfo->getRelativeObjectiveGap() << " <= "
				<< settings->getDoubleSetting("GapTermTolRelative", "Algorithm") << CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::TimeLimit)
	{
		processInfo->logger.message(1) << "Nonoptimal solution " << tmpSol << " found due to time limit "
				<< processInfo->getElapsedTime("Total") << " > " << settings->getDoubleSetting("TimeLimit", "Algorithm")
				<< CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::IterationLimit)
	{
		processInfo->logger.message(1) << "Nonoptimal solution " << tmpSol << " found due to iteration limit "
				<< settings->getIntSetting("IterLimitLP", "Algorithm")
						+ settings->getIntSetting("IterLimitMILP", "Algorithm") << CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::ObjectiveStagnation)
	{
		processInfo->logger.message(1) << "Nonoptimal solution " << tmpSol
				<< " found due to objective function stagnation" << CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::InfeasibleProblem)
	{
		processInfo->logger.message(1) << "Nonoptimal solution " << tmpSol
				<< " found since linear solver reports an infeasible problem" << CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::InteriorPointError)
	{
		processInfo->logger.message(1) << "No solution found since an interior point could not be obtained."
				<< CoinMessageEol;
	}
	else if (processInfo->terminationReason == E_TerminationReason::Error)
	{
		processInfo->logger.message(1) << "Nonoptimal solution " << tmpSol
				<< " found since linear solver reports an error" << CoinMessageEol;
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

std::string TaskPrintSolution::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
