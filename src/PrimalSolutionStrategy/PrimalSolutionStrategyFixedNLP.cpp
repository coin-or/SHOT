/*
 * PrimalSolutionStrategyFixedNLP.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: alundell
 */

#include "PrimalSolutionStrategyFixedNLP.h"

PrimalSolutionStrategyFixedNLP::PrimalSolutionStrategyFixedNLP()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	originalNLPTime = settings->getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound");
	originalNLPIter = settings->getIntSetting("NLPFixedMaxIters", "PrimalBound");

	//NLPSolver = new NLPSolverIPOptRelaxed();
	NLPSolver = new NLPSolverIPOptRelaxed();
	NLPSolver->setProblem(processInfo->originalProblem->getProblemInstance());

}

PrimalSolutionStrategyFixedNLP::~PrimalSolutionStrategyFixedNLP()
{
// TODO Auto-generated destructor stub
}

bool PrimalSolutionStrategyFixedNLP::runStrategy()
{
	auto currIter = processInfo->getCurrentIteration();
	NLPSolver->initializeProblem();
	int numVars = NLPSolver->NLPProblem->getNumberOfVariables();

	auto discreteVariableIndexes = NLPSolver->NLPProblem->getDiscreteVariableIndices();
	auto realVariableIndexes = NLPSolver->NLPProblem->getRealVariableIndices();

	bool isSolved;

	vector < PrimalFixedNLPCandidate > testPts;

	// Fix variables
	auto varTypes = NLPSolver->NLPProblem->getVariableTypes();

	if (processInfo->primalFixedNLPCandidates.size() == 0)
	{
		processInfo->itersMILPWithoutNLPCall++;
		return (false);
	}

	if (testedPoints.size() > 0)
	{
		for (int j = 0; j < processInfo->primalFixedNLPCandidates.size(); j++)
		{
			for (int i = 0; i < testedPoints.size(); i++)
			{
				if (UtilityFunctions::isDifferentRoundedSelectedElements(
						processInfo->primalFixedNLPCandidates.at(j).point, testedPoints.at(i), discreteVariableIndexes))
				{
					testPts.push_back(processInfo->primalFixedNLPCandidates.at(j));
					testedPoints.push_back(processInfo->primalFixedNLPCandidates.at(j).point);
					break;
				}
			}
		}
	}
	else
	{
		testPts.push_back(processInfo->primalFixedNLPCandidates.at(0));
		testedPoints.push_back(processInfo->primalFixedNLPCandidates.at(0).point);
	}

	if (testPts.size() == 0)
	{
		processInfo->itersMILPWithoutNLPCall++;
		return (false);
	}

	auto lbs = NLPSolver->getVariableLowerBounds();
	auto ubs = NLPSolver->getVariableUpperBounds();

	for (int j = 0; j < testPts.size(); j++)
	{
		auto oldPrimalBound = processInfo->getPrimalBound();
		double timeStart = processInfo->getElapsedTime("Total");
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
			if (settings->getBoolSetting("NLPFixedWarmstart", "PrimalBound"))
			{
				startingPointIndexes.at(currVarIndex) = currVarIndex;
				startingPointValues.at(currVarIndex) = tmpSolPt;
			}

		}

		if (settings->getBoolSetting("NLPFixedWarmstart", "PrimalBound"))
		{
			for (int k = 0; k < realVariableIndexes.size(); k++)
			{
				int currVarIndex = realVariableIndexes.at(k);

				if (NLPSolver->isObjectiveFunctionNonlinear()
						&& currVarIndex == NLPSolver->getObjectiveFunctionVariableIndex())
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

		if (settings->getBoolSetting("Debug", "SHOTSolver"))
		{
			std::string filename = settings->getStringSetting("DebugPath", "SHOTSolver") + "/primalnlp"
					+ to_string(processInfo->getCurrentIteration()->iterationNumber) + "_" + to_string(j);
			NLPSolver->saveProblemToFile(filename + ".txt");
			NLPSolver->saveOptionsToFile(filename + ".osrl");
		}

		auto solvestatus = NLPSolver->solveProblem();

		NLPSolver->unfixVariables();
		processInfo->numPrimalFixedNLPProbsSolved++;

		double timeEnd = processInfo->getElapsedTime("Total");

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

			if (NLPSolver->NLPProblem->isObjectiveFunctionNonlinear())
			{
				variableSolution.push_back(tmpObj);
			}

			auto mostDevConstr = NLPSolver->NLPProblem->getMostDeviatingConstraint(variableSolution);
			std::string tmpConstr;
			if (mostDevConstr.idx != -1)
			{
				tmpConstr = NLPSolver->NLPProblem->getConstraintNames()[mostDevConstr.idx] + ": "
						+ ((boost::format("%.5f") % mostDevConstr.value).str());
			}
			else
			{
				tmpConstr =
						NLPSolver->NLPProblem->getConstraintNames()[NLPSolver->NLPProblem->getNonlinearObjectiveConstraintIdx()]
								+ ": " + ((boost::format("%.5f") % mostDevConstr.value).str());
			}

			std::string tmpObjVal = UtilityFunctions::toStringFormat(tmpObj, "%.3f");

			std::string tmpPrimal = "";

			if (oldPrimalBound > processInfo->getPrimalBound())
			{
				tmpPrimal = tmpObjVal;
				oldPrimalBound = processInfo->getPrimalBound();
			}

			auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|")
					% processInfo->numPrimalFixedNLPProbsSolved % ("NLP" + sourceDesc) % solExpr % "" % tmpObjVal
					% tmpPrimal % tmpConstr;

			processInfo->outputSummary(tmpLine.str());

			int iters = max(ceil(settings->getIntSetting("NLPFixedMaxIters", "PrimalBound") * 0.98), originalNLPIter);
			settings->updateSetting("NLPFixedMaxIters", "PrimalBound", iters);

			double interval = max(0.9 * settings->getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound"),
					originalNLPTime);
			settings->updateSetting("NLPFixedMaxElapsedTime", "PrimalBound", interval);

			processInfo->addPrimalSolutionCandidate(variableSolution, E_PrimalSolutionSource::NLPFixedIntegers,
					currIter->iterationNumber);
		}
		else
		{
			int iters = ceil(settings->getIntSetting("NLPFixedMaxIters", "PrimalBound") * 1.02);
			settings->updateSetting("NLPFixedMaxIters", "PrimalBound", iters);
			double interval = 1.1 * settings->getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound");
			settings->updateSetting("NLPFixedMaxElapsedTime", "PrimalBound", interval);

			auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|")
					% processInfo->numPrimalFixedNLPProbsSolved % ("NLP" + sourceDesc) % solExpr % "" % "infeasible"
					% "" % "";

			processInfo->outputSummary(tmpLine.str());

			processInfo->outputInfo(
					"     Duration:  " + to_string(timeEnd - timeStart) + " s. New interval: " + to_string(interval)
							+ " s or " + to_string(iters) + " iters.");

			if (settings->getBoolSetting("AddIntegerCuts", "Algorithm")
					&& NLPSolver->NLPProblem->getNumberOfIntegerVariables() == 0)
			{
				//Add integer cut.

				auto binVars = NLPSolver->NLPProblem->getBinaryVariableIndices();

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

					processInfo->integerCutWaitingList.push_back(elements);
				}
			}
		}

		processInfo->itersMILPWithoutNLPCall = 0;
		processInfo->solTimeLastNLPCall = processInfo->getElapsedTime("Total");
	}

	processInfo->primalFixedNLPCandidates.clear();

	return (true);
}
