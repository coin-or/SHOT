/*
 * TaskCheckDualSolutionCandidates.cpp
 *
 *  Created on: Nov 22, 2015
 *      Author: alundell
 */

#include <TaskCheckDualSolutionCandidates.h>

TaskCheckDualSolutionCandidates::TaskCheckDualSolutionCandidates()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskCheckDualSolutionCandidates::~TaskCheckDualSolutionCandidates()
{
	// TODO Auto-generated destructor stub
}

void TaskCheckDualSolutionCandidates::run()
{
	bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();

	double currDualBound = processInfo->getDualBound();
	double currPrimalBound = processInfo->getPrimalBound();

	for (auto C : processInfo->dualSolutionCandidates)
	{

		if ((isMinimization && (C.objValue > currDualBound && C.objValue <= currPrimalBound))
				|| (!isMinimization && (C.objValue < currDualBound && C.objValue >= currPrimalBound)))
		{

			// New dual solution
			processInfo->currentObjectiveBounds.first = C.objValue;
			currDualBound = C.objValue;
			processInfo->iterLastDualBoundUpdate = processInfo->getCurrentIteration()->iterationNumber;

			// If the solution is MILP feasible we only have a bound, no variable solutions
			if (C.sourceType != E_DualSolutionSource::MILPSolutionFeasible)
			{
				processInfo->addDualSolution(C);
			}

			std::string sourceDesc;

			switch (C.sourceType)
			{
				/*case E_DualSolutionSource::Linesearch:
				 sourceDesc = "line search";
				 break;*/
				case E_DualSolutionSource::LPSolution:
					sourceDesc = "LP solution";
					break;
				case E_DualSolutionSource::MILPSolutionOptimal:
					sourceDesc = "MILP solution";
					break;
				case E_DualSolutionSource::MILPSolutionFeasible:
					sourceDesc = "MILP solution bound";
					break;
				case E_DualSolutionSource::ObjectiveConstraint:
					sourceDesc = "Obj. constr. linesearch";
					break;
				default:
					break;
			}

			auto tmpLine = boost::format("    New dual bound %1% (%2%) ") % C.objValue % sourceDesc;
			if (C.sourceType != E_DualSolutionSource::LPSolution)
			{
				processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;
			}
			else
			{
				processInfo->logger.message(4) << tmpLine.str() << CoinMessageEol;
			}

		}
	}

	processInfo->dualSolutionCandidates.clear();
}

std::string TaskCheckDualSolutionCandidates::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
