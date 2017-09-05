/*
 * TaskPrintIterationReport.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include "TaskPrintIterationReport.h"

TaskPrintIterationReport::TaskPrintIterationReport()
{
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	lastNumHyperplane = 0;
}

TaskPrintIterationReport::~TaskPrintIterationReport()
{
	// TODO Auto-generated destructor stub
}

void TaskPrintIterationReport::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	try
	{
		std::stringstream tmpType;

		bool hasSolution = true;

		bool isMIQP = (ProcessInfo::getInstance().originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic);
		bool isMIQCP = (ProcessInfo::getInstance().originalProblem->getQuadraticConstraintIndexes().size() > 0);
		bool isDiscrete = (currIter->type == E_IterationProblemType::MIP);

		if (isMIQCP && isDiscrete) tmpType << "MIQCP";
		else if (isMIQCP) tmpType << "QCP";
		else if (isMIQP && isDiscrete) tmpType << "MIQP";
		else if (isMIQP) tmpType << "QP";
		else if (isDiscrete) tmpType << "MILP";
		else tmpType << "LP";

		if (currIter->solutionPoints.size() == 0) hasSolution = false;

		if (currIter->solutionStatus == E_ProblemSolutionStatus::Error)
		{
			tmpType << " ERR";
			hasSolution = false;
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Feasible)
		{
			tmpType << " FEA";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible)
		{
			tmpType << " INF";
			hasSolution = false;
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::IterationLimit)
		{
			tmpType << " ITL";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
		{
			tmpType << " OPT";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
		{
			tmpType << " SL";
			if (currIter->usedMILPSolutionLimit > 1000)
			{
				tmpType << "∞";
			}
			else
			{
				tmpType << std::to_string(currIter->usedMILPSolutionLimit);
			}
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit)
		{
			tmpType << " TIL";
			hasSolution = false;
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Unbounded)
		{
			tmpType << " UNB";
			hasSolution = false;
		}

		std::string hyperplanesExpr;

		auto numHyperAdded = currIter->numHyperplanesAdded;
		auto numHyperTot = currIter->totNumHyperplanes;

		if (numHyperTot > lastNumHyperplane)
		{
			hyperplanesExpr = "+" + to_string(numHyperAdded) + " = " + to_string(numHyperTot);
			lastNumHyperplane = numHyperTot;
		}
		else
		{
			hyperplanesExpr = "      ";
		}

		std::string primalBoundExpr;
		std::string dualBoundExpr;

		auto primalBound = ProcessInfo::getInstance().getPrimalBound();
		auto dualBound = ProcessInfo::getInstance().getDualBound();

		if (primalBound != lastPrimalBound && ProcessInfo::getInstance().primalSolutions.size() > 0
				&& ProcessInfo::getInstance().primalSolutions.at(0).sourceType == E_PrimalSolutionSource::MILPSolutionPool)
		{
			primalBoundExpr = UtilityFunctions::toString(primalBound);
			lastPrimalBound = primalBound;
		}
		else
		{
			primalBoundExpr = "";
		}

		if (dualBound != lastDualBound)
		{
			dualBoundExpr = UtilityFunctions::toString(dualBound);
			lastDualBound = dualBound;
		}
		else
		{
			dualBoundExpr = "";
		}

		std::string tmpObjVal = UtilityFunctions::toString(currIter->objectiveValue);

		std::string tmpConstr;

		double tmpConstrVal = currIter->maxDeviation;

		std::string tmpConstrExpr;

		tmpConstrExpr = UtilityFunctions::toStringFormat(currIter->maxDeviation, "%.5f");

		if (hasSolution && currIter->maxDeviationConstraint != -1)
		{
			tmpConstr = ProcessInfo::getInstance().originalProblem->getConstraintNames()[currIter->maxDeviationConstraint] + ": "
					+ tmpConstrExpr;
		}
		else if (hasSolution)
		{
			tmpConstr = ProcessInfo::getInstance().originalProblem->getConstraintNames().back() + ": " + tmpConstrExpr;
		}
		else
		{
			tmpConstr = "";
		}

		auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|")
				% currIter->iterationNumber % tmpType.str() % hyperplanesExpr % dualBoundExpr % tmpObjVal
				% primalBoundExpr % tmpConstr;

		ProcessInfo::getInstance().outputSummary(tmpLine.str());

	}
	catch (...)
	{
		ProcessInfo::getInstance().outputError("ERROR, cannot write iteration solution report!");
	}
}
std::string TaskPrintIterationReport::getType()
{
	std::string type = typeid(this).name();
	return (type);
}

