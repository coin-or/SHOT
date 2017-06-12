#include "TaskSelectPrimalFixedNLPPointsFromSolutionPool.h"

TaskSelectPrimalFixedNLPPointsFromSolutionPool::TaskSelectPrimalFixedNLPPointsFromSolutionPool()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskSelectPrimalFixedNLPPointsFromSolutionPool::~TaskSelectPrimalFixedNLPPointsFromSolutionPool()
{
	// TODO Auto-generated destructor stub
}

void TaskSelectPrimalFixedNLPPointsFromSolutionPool::run()
{
	auto currIter = processInfo->getCurrentIteration();
	auto allSolutions = processInfo->getCurrentIteration()->solutionPoints;

	bool callNLPSolver = false;
	bool useFeasibleSolutionExtra = false;

	if (!currIter->isMILP())
	{
		return;
	}

	if (allSolutions.size() == 0)
	{
		return;
	}

	if (currIter->MILPSolutionLimitUpdated && currIter->solutionStatus != E_ProblemSolutionStatus::Optimal)
	{
		processInfo->itersMILPWithoutNLPCall++;
		return;
	}

	processInfo->startTimer("PrimalBoundTotal");
	processInfo->startTimer("PrimalBoundSearchNLP");

	auto userSettingStrategy = settings->getIntSetting("NLPFixedStrategy", "PrimalBound");
	auto userSetting = settings->getIntSetting("NLPFixedSource", "PrimalBound");

	if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::DoNotUse))
	{
		callNLPSolver = false;
	}
	else if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal
			&& abs(allSolutions.at(0).objectiveValue - processInfo->getDualBound()) < 0.1)
	{
		callNLPSolver = true;
		useFeasibleSolutionExtra = true;
	}
	else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::AlwaysUse))
	{
		callNLPSolver = true;
	}
	else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTime)
			|| userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
	{
		/*if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
		 {
		 useFeasibleSolution = true;
		 }*/

		if (processInfo->itersMILPWithoutNLPCall >= settings->getIntSetting("NLPFixedMaxIters", "PrimalBound"))
		{
			processInfo->outputInfo(
					"     Activating fixed NLP primal strategy since max iterations since last call has been reached.");
			callNLPSolver = true;
		}
		else if (processInfo->getElapsedTime("Total") - processInfo->solTimeLastNLPCall
				> settings->getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound"))
		{
			processInfo->outputInfo(
					"     Activating fixed NLP primal strategy since max time limit since last call has been reached.");
			callNLPSolver = true;
		}
	}

	if (useFeasibleSolutionExtra)
	{
		auto tmpSol = allSolutions.at(0);
		processInfo->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolutionNewDualBound,
				tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalBoundNLPFixedPoint::SmallestDeviationSolution))
	{
		auto tmpSol = currIter->getSolutionPointWithSmallestDeviation();
		processInfo->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::SmallestDeviationSolution,
				tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalBoundNLPFixedPoint::FirstSolution))
	{
		auto tmpSol = allSolutions.at(0);
		processInfo->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution, tmpSol.objectiveValue,
				tmpSol.iterFound, tmpSol.maxDeviation);
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalBoundNLPFixedPoint::FirstAndFeasibleSolutions))
	{
		auto tmpSol = allSolutions.at(0);
		processInfo->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution, tmpSol.objectiveValue,
				tmpSol.iterFound, tmpSol.maxDeviation);

		auto smallestDevSolIdx = currIter->getSolutionPointWithSmallestDeviationIndex();

		if (smallestDevSolIdx != 0)
		{
			tmpSol = allSolutions.at(smallestDevSolIdx);
			processInfo->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
					tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
		}
	}
	else if (callNLPSolver
			&& userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
	{
		auto tmpSol = allSolutions.at(0);

		processInfo->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution, tmpSol.objectiveValue,
				tmpSol.iterFound, tmpSol.maxDeviation);

		for (int i = 1; i < allSolutions.size(); i++)
		{
			auto tmpSol = allSolutions.at(i);

			if (tmpSol.maxDeviation.value <= settings->getDoubleSetting("PrimalBoundNonlinearTolerance", "PrimalBound"))
			{
				processInfo->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
						tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
			}
		}
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalBoundNLPFixedPoint::AllSolutions))
	{
		auto tmpSol = allSolutions.at(0);

		processInfo->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution, tmpSol.objectiveValue,
				tmpSol.iterFound, tmpSol.maxDeviation);

		for (int i = 1; i < allSolutions.size(); i++)
		{
			tmpSol = allSolutions.at(i);

			if (tmpSol.maxDeviation.value <= settings->getDoubleSetting("PrimalBoundNonlinearTolerance", "PrimalBound"))
			{
				processInfo->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
						tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
			}
			else
			{
				processInfo->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::UnFeasibleSolution,
						tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
			}
		}
	}
	else
	{
		//processInfo->itersMILPWithoutNLPCall++;

		processInfo->stopTimer("PrimalBoundSearchNLP");
		processInfo->stopTimer("PrimalBoundTotal");

	}

	//std::cout << "Num NLP cands: " << processInfo->primalFixedNLPCandidates.size() << std::endl;
}

std::string TaskSelectPrimalFixedNLPPointsFromSolutionPool::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

