/*
 * TaskPrintIterationReport.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include <TaskPrintIterationReport.h>

TaskPrintIterationReport::TaskPrintIterationReport()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskPrintIterationReport::~TaskPrintIterationReport()
{
	// TODO Auto-generated destructor stub
}

void TaskPrintIterationReport::run()
{
	auto currIter = processInfo->getCurrentIteration();

	try
	{
		std::string tmpboundaryDistance = " ";

		if (!OSIsnan(currIter->boundaryDistance))
		{
			tmpboundaryDistance = (
					currIter->boundaryDistance == DBL_MAX ? " " : std::to_string(currIter->boundaryDistance));
		}

		std::size_t found = tmpboundaryDistance.find("INF");
		if (found != std::string::npos)
		{
			tmpboundaryDistance = "";
		}

		std::stringstream tmpType;

		if (processInfo->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic
				|| processInfo->originalProblem->getQuadraticConstraintIndexes().size() > 0)
		{
			tmpType << "Q";
		}
		else
		{
			tmpType << "L";
		}

		if (currIter->type == E_IterationProblemType::MIP)
		{
			tmpType << " T";
		}
		else if (currIter->type == E_IterationProblemType::Relaxed)
		{
			tmpType << " F";
		}

		if (currIter->solutionStatus == E_ProblemSolutionStatus::Error)
		{
			tmpType << " E";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Feasible)
		{
			tmpType << " F";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible)
		{
			tmpType << " I";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::IterationLimit)
		{
			tmpType << " IL";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
		{
			tmpType << " O";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
		{
			tmpType << " SL";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit)
		{
			tmpType << " TL";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Unbounded)
		{
			tmpType << " UB";
		}

		std::string solLimit = (currIter->isMILP() ? std::to_string(currIter->usedMILPSolutionLimit) : " ");

		std::string tmpConstr;

		if (currIter->maxDeviationConstraint != -1)
		{
			tmpConstr = processInfo->originalProblem->getConstraintNames()[currIter->maxDeviationConstraint];
		}
		else
		{
			tmpConstr = processInfo->originalProblem->getConstraintNames().back();
		}

		auto tmpLine = boost::format(
				"%1% %|4t|%2% %|10t|%3% %|14t|+%4% = %5% %|24t|%6% %|38t|%7% %|46t|%8%: %|54t|%9% %|70t|%10%")
				% currIter->iterationNumber % tmpType.str() % solLimit % currIter->numHyperplanesAdded
				% currIter->totNumHyperplanes % currIter->objectiveValue % currIter->usedConstraintTolerance % tmpConstr
				% currIter->maxDeviation % tmpboundaryDistance;

		processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;
	}
	catch (...)
	{
		processInfo->logger.message(1) << "ERROR, cannot write iteration solution report!" << CoinMessageEol;
	}
}
