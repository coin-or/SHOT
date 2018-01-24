/*
 * PrimalSolutionStrategyFixedNLP.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: alundell
 */

#include "PrimalSolutionStrategyFixedNLP.h"

PrimalSolutionStrategyFixedNLP::PrimalSolutionStrategyFixedNLP()
{
	originalNLPTime = Settings::getInstance().getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound");
	originalNLPIter = Settings::getInstance().getIntSetting("NLPFixedMaxIters", "PrimalBound");

	switch (static_cast<ES_PrimalNLPSolver>(Settings::getInstance().getIntSetting("PrimalNLPSolver", "PrimalBound")))
	{
	case ES_PrimalNLPSolver::CuttingPlane:
	{
		NLPSolver = new NLPSolverCuttingPlaneRelaxed();
		break;
	}
	case ES_PrimalNLPSolver::IPOpt:
	{
		NLPSolver = new NLPSolverIPOptRelaxed();
		break;
	}
	case ES_PrimalNLPSolver::GAMS:
	{
		NLPSolver = new NLPSolverGAMS();
		break;
	}
	default:
		throw std::logic_error("Unknown PrimalNLPSolver setting.");
	}

	NLPSolver->setProblem(ProcessInfo::getInstance().originalProblem->getProblemInstance());
}

PrimalSolutionStrategyFixedNLP::~PrimalSolutionStrategyFixedNLP()
{
	// TODO Auto-generated destructor stub
}

bool PrimalSolutionStrategyFixedNLP::runStrategy()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	NLPSolver->initializeProblem();

	int numVars = ProcessInfo::getInstance().originalProblem->getNumberOfVariables();

	auto discreteVariableIndexes = ProcessInfo::getInstance().originalProblem->getDiscreteVariableIndices();
	auto realVariableIndexes = ProcessInfo::getInstance().originalProblem->getRealVariableIndices();

	bool isSolved;

	vector<PrimalFixedNLPCandidate> testPts;

	// Fix variables
	auto varTypes = ProcessInfo::getInstance().originalProblem->getVariableTypes();

	if (ProcessInfo::getInstance().primalFixedNLPCandidates.size() == 0)
	{
		ProcessInfo::getInstance().itersMILPWithoutNLPCall++;
		return (false);
	}

	if (testedPoints.size() > 0)
	{
		for (int j = 0; j < ProcessInfo::getInstance().primalFixedNLPCandidates.size(); j++)
		{
			for (int i = 0; i < testedPoints.size(); i++)
			{
				if (UtilityFunctions::isDifferentRoundedSelectedElements(
						ProcessInfo::getInstance().primalFixedNLPCandidates.at(j).point, testedPoints.at(i),
						discreteVariableIndexes))
				{
					testPts.push_back(ProcessInfo::getInstance().primalFixedNLPCandidates.at(j));
					testedPoints.push_back(ProcessInfo::getInstance().primalFixedNLPCandidates.at(j).point);
					break;
				}
			}
		}
	}
	else
	{
		testPts.push_back(ProcessInfo::getInstance().primalFixedNLPCandidates.at(0));
		testedPoints.push_back(ProcessInfo::getInstance().primalFixedNLPCandidates.at(0).point);
	}

	if (testPts.size() == 0)
	{
		ProcessInfo::getInstance().itersMILPWithoutNLPCall++;
		return (false);
	}

	auto lbs = NLPSolver->getVariableLowerBounds();
	auto ubs = NLPSolver->getVariableUpperBounds();

	for (int j = 0; j < testPts.size(); j++)
	{
		auto oldPrimalBound = ProcessInfo::getInstance().getPrimalBound();
		double timeStart = ProcessInfo::getInstance().getElapsedTime("Total");
		std::vector<double> fixedVariableValues(discreteVariableIndexes.size());

		int sizeOfVariableVector = NLPSolver->NLPProblem->getNumberOfVariables();

		std::vector<int> startingPointIndexes(sizeOfVariableVector);
		std::vector<double> startingPointValues(sizeOfVariableVector);

		// Sets the fixed values for discrete variables
		for (int k = 0; k < discreteVariableIndexes.size(); k++)
		{
			int currVarIndex = discreteVariableIndexes.at(k);

			auto tmpSolPt = UtilityFunctions::round(testPts.at(j).point.at(currVarIndex));

			fixedVariableValues.at(k) = tmpSolPt;

			// Sets the starting point to the fixed value
			if (Settings::getInstance().getBoolSetting("NLPFixedWarmstart", "PrimalBound"))
			{
				startingPointIndexes.at(currVarIndex) = currVarIndex;
				startingPointValues.at(currVarIndex) = tmpSolPt;
			}
		}

		if (Settings::getInstance().getBoolSetting("NLPFixedWarmstart", "PrimalBound"))
		{
			for (int k = 0; k < realVariableIndexes.size(); k++)
			{
				int currVarIndex = realVariableIndexes.at(k);

				if (NLPSolver->isObjectiveFunctionNonlinear() && currVarIndex == NLPSolver->getObjectiveFunctionVariableIndex())
				{
					continue;
				}

				auto tmpSolPt = testPts.at(j).point.at(currVarIndex);

				startingPointIndexes.at(currVarIndex) = currVarIndex;
				startingPointValues.at(currVarIndex) = tmpSolPt;
			}
		}

		NLPSolver->setStartingPoint(startingPointIndexes, startingPointValues);

		NLPSolver->fixVariables(discreteVariableIndexes, fixedVariableValues);

		if (Settings::getInstance().getBoolSetting("Debug", "SHOTSolver"))
		{
			std::string filename = Settings::getInstance().getStringSetting("DebugPath", "SHOTSolver") + "/primalnlp" + to_string(ProcessInfo::getInstance().getCurrentIteration()->iterationNumber) + "_" + to_string(j);
			NLPSolver->saveProblemToFile(filename + ".txt");
			NLPSolver->saveOptionsToFile(filename + ".osrl");
		}

		auto solvestatus = NLPSolver->solveProblem();

		NLPSolver->unfixVariables();
		ProcessInfo::getInstance().numPrimalFixedNLPProbsSolved++;

		double timeEnd = ProcessInfo::getInstance().getElapsedTime("Total");

		std::string sourceDesc;
		switch (testPts.at(j).sourceType)
		{
		case E_PrimalNLPSource::FirstSolution:
			sourceDesc = "SOLPT ";
			break;
		case E_PrimalNLPSource::FeasibleSolution:
			sourceDesc = "FEASPT";
			break;
		case E_PrimalNLPSource::UnFeasibleSolution:
			sourceDesc = "UNFEAS";
			break;
		case E_PrimalNLPSource::SmallestDeviationSolution:
			sourceDesc = "SMADEV";
			break;
		case E_PrimalNLPSource::FirstSolutionNewDualBound:
			sourceDesc = "NEWDB";
			break;
		default:
			break;
		}

		std::string solExpr = ((boost::format("(%.3f s)") % (timeEnd - timeStart)).str());

		if (solvestatus == E_NLPSolutionStatus::Feasible || solvestatus == E_NLPSolutionStatus::Optimal)
		{
			double tmpObj = NLPSolver->getObjectiveValue();
			auto variableSolution = NLPSolver->getSolution();

			if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear())
			{
				variableSolution.push_back(tmpObj);
			}

			auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(
				variableSolution);

			std::string tmpConstr;
			if (NLPSolver->isObjectiveFunctionNonlinear() && (mostDevConstr.idx == NLPSolver->NLPProblem->getNonlinearObjectiveConstraintIdx() || mostDevConstr.idx == -1))
			{
				tmpConstr = ProcessInfo::getInstance().originalProblem->getConstraintNames().at(
								ProcessInfo::getInstance().originalProblem->getNonlinearObjectiveConstraintIdx()) +
							": " + ((boost::format("%.5f") % mostDevConstr.value).str());
			}
			else
			{
				tmpConstr = ProcessInfo::getInstance().originalProblem->getConstraintNames().at(mostDevConstr.idx) + ": " + ((boost::format("%.5f") % mostDevConstr.value).str());
			}

			std::string tmpObjVal = UtilityFunctions::toStringFormat(tmpObj, "%.3f");

			std::string tmpPrimal = "";

			if (oldPrimalBound > ProcessInfo::getInstance().getPrimalBound())
			{
				tmpPrimal = tmpObjVal;
				oldPrimalBound = ProcessInfo::getInstance().getPrimalBound();
			}

			auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % ProcessInfo::getInstance().numPrimalFixedNLPProbsSolved % ("NLP" + sourceDesc) % solExpr % "" % tmpObjVal % tmpPrimal % tmpConstr;

			ProcessInfo::getInstance().outputSummary(tmpLine.str());

			if (Settings::getInstance().getBoolSetting("NLPFixedUpdateItersAndTime", "PrimalBound"))
			{
				int iters = max(ceil(Settings::getInstance().getIntSetting("NLPFixedMaxIters", "PrimalBound") * 0.98),
								originalNLPIter);
				Settings::getInstance().updateSetting("NLPFixedMaxIters", "PrimalBound", iters);

				double interval = max(
					0.9 * Settings::getInstance().getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound"),
					originalNLPTime);
				Settings::getInstance().updateSetting("NLPFixedMaxElapsedTime", "PrimalBound", interval);

				ProcessInfo::getInstance().addPrimalSolutionCandidate(variableSolution,
																	  E_PrimalSolutionSource::NLPFixedIntegers, currIter->iterationNumber);
			}
		}
		else
		{
			if (Settings::getInstance().getBoolSetting("NLPFixedUpdateItersAndTime", "PrimalBound"))
			{
				int iters = ceil(Settings::getInstance().getIntSetting("NLPFixedMaxIters", "PrimalBound") * 1.02);
				Settings::getInstance().updateSetting("NLPFixedMaxIters", "PrimalBound", iters);
				double interval = 1.1 * Settings::getInstance().getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound");
				Settings::getInstance().updateSetting("NLPFixedMaxElapsedTime", "PrimalBound", interval);

				ProcessInfo::getInstance().outputInfo(
					"     Duration:  " + to_string(timeEnd - timeStart) + " s. New interval: " + to_string(interval) + " s or " + to_string(iters) + " iters.");
			}

			auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % ProcessInfo::getInstance().numPrimalFixedNLPProbsSolved % ("NLP" + sourceDesc) % solExpr % "" % "infeasible" % "" % "";

			ProcessInfo::getInstance().outputSummary(tmpLine.str());

			if (Settings::getInstance().getBoolSetting("AddIntegerCuts", "Algorithm") && ProcessInfo::getInstance().originalProblem->getNumberOfIntegerVariables() == 0)
			{
				//Add integer cut.

				auto binVars = ProcessInfo::getInstance().originalProblem->getBinaryVariableIndices();

				if (binVars.size() > 0)
				{
					std::vector<int> elements;

					for (int i = 0; i < binVars.size(); i++)
					{
						if (testPts.at(j).point.at(binVars.at(i)) > 0.99)
						{
							elements.push_back(binVars.at(i));
						}
					}
					ProcessInfo::getInstance().integerCutWaitingList.push_back(elements);
				}
			}
		}

		ProcessInfo::getInstance().itersMILPWithoutNLPCall = 0;
		ProcessInfo::getInstance().solTimeLastNLPCall = ProcessInfo::getInstance().getElapsedTime("Total");
	}

	ProcessInfo::getInstance().primalFixedNLPCandidates.clear();

	return (true);
}
