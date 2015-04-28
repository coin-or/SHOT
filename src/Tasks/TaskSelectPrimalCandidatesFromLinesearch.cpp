/*
 * TaskSelectPrimalCandidatesFromLinesearch.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include <TaskSelectPrimalCandidatesFromLinesearch.h>

TaskSelectPrimalCandidatesFromLinesearch::TaskSelectPrimalCandidatesFromLinesearch()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("PrimalBoundTotal");
	processInfo->startTimer("PrimalBoundLinesearch");
	if (settings->getIntSetting("LinesearchMethod", "Linesearch") == static_cast<int>(ES_LinesearchMethod::Boost))
	{
		processInfo->logger.message(2) << "Boost linesearch implementation selected for primal heuristics"
				<< CoinMessageEol;
		linesearchMethod = new LinesearchMethodBoost();
	}
	else if (settings->getIntSetting("LinesearchMethod", "Linesearch")
			== static_cast<int>(ES_LinesearchMethod::Bisection))
	{
		processInfo->logger.message(2) << "Bisection linesearch selected primal heuristics" << CoinMessageEol;
		linesearchMethod = new LinesearchMethodBisection();
	}

	processInfo->stopTimer("PrimalBoundLinesearch");
	processInfo->stopTimer("PrimalBoundTotal");
}

TaskSelectPrimalCandidatesFromLinesearch::~TaskSelectPrimalCandidatesFromLinesearch()
{
	// TODO Auto-generated destructor stub
}

void TaskSelectPrimalCandidatesFromLinesearch::run()
{

	processInfo->startTimer("PrimalBoundTotal");
	processInfo->startTimer("PrimalBoundLinesearch");

	//auto allSolutions = processInfo->MILPSolver->getAllVariableSolutions();
	auto allSolutions = processInfo->getCurrentIteration()->variableSolutions;
	for (int i = 0; i < allSolutions.size(); i++)
	{
		for (int j = 0; j < processInfo->interiorPts.size(); j++)
		{
			auto xNLP = processInfo->interiorPts.at(j).point;

			auto varTypes = processInfo->originalProblem->getVariableTypes();

			std::vector<double> xNLP2(xNLP.size());
			for (int k = 0; k < processInfo->originalProblem->getNumberOfVariables(); k++)
			{
				if (varTypes.at(k) == 'I' || varTypes.at(k) == 'B')
				{
					xNLP2.at(k) = allSolutions.at(i).at(k);

				}
				else
				{
					xNLP2.at(k) = xNLP.at(k);
				}
			}

			auto maxDevNLP2 = processInfo->originalProblem->getMostDeviatingAllConstraint(xNLP2);
			auto maxDevMILP = processInfo->originalProblem->getMostDeviatingAllConstraint(allSolutions.at(i));

			if (maxDevNLP2.value <= 0 && maxDevMILP.value >= 0)
			{
				auto xNewc2 = linesearchMethod->findZero(xNLP2, allSolutions.at(i),
						settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
						settings->getDoubleSetting("LinesearchEps", "Linesearch"));

				processInfo->addPrimalSolutionCandidate(xNewc2, E_PrimalSolutionSource::LinesearchFixedIntegers,
						processInfo->getCurrentIteration()->iterationNumber);
			}
		}
	}

	processInfo->stopTimer("PrimalBoundLinesearch");
	processInfo->stopTimer("PrimalBoundTotal");
}
