/*
 * TaskSelectPrimalCandidatesFromLinesearch.cpp
 *
 *  Created on: Apr 7, 2015
 *      Author: alundell
 */

#include "TaskSelectPrimalCandidatesFromLinesearch.h"

TaskSelectPrimalCandidatesFromLinesearch::TaskSelectPrimalCandidatesFromLinesearch()
{

}

TaskSelectPrimalCandidatesFromLinesearch::~TaskSelectPrimalCandidatesFromLinesearch()
{
	// TODO Auto-generated destructor stub
}

void TaskSelectPrimalCandidatesFromLinesearch::run()
{
	this->run(ProcessInfo::getInstance().getCurrentIteration()->solutionPoints);
}

std::string TaskSelectPrimalCandidatesFromLinesearch::getType()
{
	std::string type = typeid(this).name();
	return (type);
}

void TaskSelectPrimalCandidatesFromLinesearch::run(vector<SolutionPoint> solPoints)
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->isMILP() && ProcessInfo::getInstance().getRelativeObjectiveGap() > 1e-10)
	{
		ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
		ProcessInfo::getInstance().startTimer("PrimalBoundLinesearch");

		for (int i = 0; i < solPoints.size(); i++)
		{
			for (int j = 0; j < ProcessInfo::getInstance().interiorPts.size(); j++)
			{
				auto xNLP = ProcessInfo::getInstance().interiorPts.at(j)->point;

				auto varTypes = ProcessInfo::getInstance().originalProblem->getVariableTypes();

				std::vector<double> xNLP2(xNLP.size());
				for (int k = 0; k < ProcessInfo::getInstance().originalProblem->getNumberOfVariables(); k++)
				{
					if (varTypes.at(k) == 'I' || varTypes.at(k) == 'B')
					{
						xNLP2.at(k) = solPoints.at(i).point.at(k);

					}
					else
					{
						xNLP2.at(k) = xNLP.at(k);
					}
				}

				auto maxDevNLP2 = ProcessInfo::getInstance().originalProblem->getMostDeviatingAllConstraint(xNLP2);
				auto maxDevMILP = ProcessInfo::getInstance().originalProblem->getMostDeviatingAllConstraint(
						solPoints.at(i).point);

				if (maxDevNLP2.value <= 0 && maxDevMILP.value > 0)
				{
					try
					{
						ProcessInfo::getInstance().startTimer("PrimalBoundLinesearch");
						auto xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(xNLP2, solPoints.at(i).point,
								Settings::getInstance().getIntSetting("LinesearchMaxIter", "Linesearch"),
								Settings::getInstance().getDoubleSetting("LinesearchLambdaEps", "Linesearch"), 0);

						ProcessInfo::getInstance().stopTimer("PrimalBoundLinesearch");

						ProcessInfo::getInstance().addPrimalSolutionCandidate(xNewc.first,
								E_PrimalSolutionSource::Linesearch,
								ProcessInfo::getInstance().getCurrentIteration()->iterationNumber);
					}
					catch (std::exception &e)
					{

						ProcessInfo::getInstance().outputWarning("Cannot find solution with primal bound linesearch.");
						ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
						ProcessInfo::getInstance().stopTimer("PrimalBoundLinesearch");
					}
				}
			}

			ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
			ProcessInfo::getInstance().stopTimer("PrimalBoundLinesearch");
		}
	}
}
