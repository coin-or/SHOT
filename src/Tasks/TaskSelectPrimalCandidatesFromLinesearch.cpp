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
		linesearchMethod = new LinesearchMethodBoost();
		processInfo->outputDebug("Boost linesearch implementation selected for primal heuristics.");
	}
	else if (settings->getIntSetting("LinesearchMethod", "Linesearch")
			== static_cast<int>(ES_LinesearchMethod::Bisection))
	{
		linesearchMethod = new LinesearchMethodBisection();
		processInfo->outputDebug("Bisection linesearch implementation selected for primal heuristics.");
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
	auto currIter = processInfo->getCurrentIteration();

	if (currIter->isMILP() && processInfo->getRelativeObjectiveGap() > 1e-10)
	{
		processInfo->startTimer("PrimalBoundTotal");
		processInfo->startTimer("PrimalBoundLinesearch");

		auto allSolutions = processInfo->getCurrentIteration()->solutionPoints;

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
						xNLP2.at(k) = allSolutions.at(i).point.at(k);

					}
					else
					{
						xNLP2.at(k) = xNLP.at(k);
					}
				}

				auto maxDevNLP2 = processInfo->originalProblem->getMostDeviatingAllConstraint(xNLP2);
				auto maxDevMILP = processInfo->originalProblem->getMostDeviatingAllConstraint(allSolutions.at(i).point);

				if (maxDevNLP2.value <= 0 && maxDevMILP.value > 0)
				{
					try
					{

						processInfo->startTimer("PrimalBoundLinesearch");
						auto xNewc = linesearchMethod->findZero(xNLP2, allSolutions.at(i).point,
								settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
								settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"), 0);

						processInfo->stopTimer("PrimalBoundLinesearch");

						processInfo->addPrimalSolutionCandidate(xNewc.first, E_PrimalSolutionSource::Linesearch,
								processInfo->getCurrentIteration()->iterationNumber);
					}
					catch (std::exception &e)
					{

						processInfo->outputWarning("Cannot find solution with primal bound linesearch.");
						processInfo->stopTimer("PrimalBoundTotal");
						processInfo->stopTimer("PrimalBoundLinesearch");
					}
				}
			}

			processInfo->stopTimer("PrimalBoundTotal");
			processInfo->stopTimer("PrimalBoundLinesearch");
		}
	}
}

std::string TaskSelectPrimalCandidatesFromLinesearch::getType()
{
	std::string type = typeid(this).name();
	return (type);
}

