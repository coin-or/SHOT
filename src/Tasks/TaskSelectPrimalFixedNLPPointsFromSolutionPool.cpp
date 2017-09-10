#include "TaskSelectPrimalFixedNLPPointsFromSolutionPool.h"

TaskSelectPrimalFixedNLPPointsFromSolutionPool::TaskSelectPrimalFixedNLPPointsFromSolutionPool()
{

}

TaskSelectPrimalFixedNLPPointsFromSolutionPool::~TaskSelectPrimalFixedNLPPointsFromSolutionPool()
{
	// TODO Auto-generated destructor stub
}

void TaskSelectPrimalFixedNLPPointsFromSolutionPool::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();
	auto allSolutions = ProcessInfo::getInstance().getCurrentIteration()->solutionPoints;

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
		ProcessInfo::getInstance().itersMILPWithoutNLPCall++;
		return;
	}

	ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
	ProcessInfo::getInstance().startTimer("PrimalBoundSearchNLP");

	auto userSettingStrategy = Settings::getInstance().getIntSetting("NLPFixedStrategy", "PrimalBound");
	auto userSetting = Settings::getInstance().getIntSetting("NLPFixedSource", "PrimalBound");

	if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::DoNotUse))
	{
		callNLPSolver = false;
	}
	else if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal
			&& abs(allSolutions.at(0).objectiveValue - ProcessInfo::getInstance().getDualBound()) < 0.1)
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

		if (ProcessInfo::getInstance().itersMILPWithoutNLPCall
				>= Settings::getInstance().getIntSetting("NLPFixedMaxIters", "PrimalBound"))
		{
			ProcessInfo::getInstance().outputInfo(
					"     Activating fixed NLP primal strategy since max iterations since last call has been reached.");
			callNLPSolver = true;
		}
		else if (ProcessInfo::getInstance().getElapsedTime("Total") - ProcessInfo::getInstance().solTimeLastNLPCall
				> Settings::getInstance().getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound"))
		{
			ProcessInfo::getInstance().outputInfo(
					"     Activating fixed NLP primal strategy since max time limit since last call has been reached.");
			callNLPSolver = true;
		}
	}

	if (useFeasibleSolutionExtra)
	{
		auto tmpSol = allSolutions.at(0);
		ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point,
				E_PrimalNLPSource::FirstSolutionNewDualBound, tmpSol.objectiveValue, tmpSol.iterFound,
				tmpSol.maxDeviation);
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalBoundNLPFixedPoint::SmallestDeviationSolution))
	{
		auto tmpSol = currIter->getSolutionPointWithSmallestDeviation();
		ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point,
				E_PrimalNLPSource::SmallestDeviationSolution, tmpSol.objectiveValue, tmpSol.iterFound,
				tmpSol.maxDeviation);
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalBoundNLPFixedPoint::FirstSolution))
	{
		auto tmpSol = allSolutions.at(0);
		ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
				tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalBoundNLPFixedPoint::FirstAndFeasibleSolutions))
	{
		auto tmpSol = allSolutions.at(0);
		ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
				tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);

		auto smallestDevSolIdx = currIter->getSolutionPointWithSmallestDeviationIndex();

		if (smallestDevSolIdx != 0)
		{
			tmpSol = allSolutions.at(smallestDevSolIdx);
			ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
					tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
		}
	}
	else if (callNLPSolver
			&& userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
	{
		auto tmpSol = allSolutions.at(0);

		ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
				tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);

		for (int i = 1; i < allSolutions.size(); i++)
		{
			auto tmpSol = allSolutions.at(i);

			if (tmpSol.maxDeviation.value
					<= Settings::getInstance().getDoubleSetting("PrimalBoundNonlinearTolerance", "PrimalBound"))
			{
				ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
						tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
			}
		}
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalBoundNLPFixedPoint::AllSolutions))
	{
		auto tmpSol = allSolutions.at(0);

		ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
				tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);

		for (int i = 1; i < allSolutions.size(); i++)
		{
			tmpSol = allSolutions.at(i);

			if (tmpSol.maxDeviation.value
					<= Settings::getInstance().getDoubleSetting("PrimalBoundNonlinearTolerance", "PrimalBound"))
			{
				ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
						tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
			}
			else
			{
				ProcessInfo::getInstance().addPrimalFixedNLPCandidate(tmpSol.point,
						E_PrimalNLPSource::UnFeasibleSolution, tmpSol.objectiveValue, tmpSol.iterFound,
						tmpSol.maxDeviation);
			}
		}
	}
	else
	{
		//ProcessInfo::getInstance().itersMILPWithoutNLPCall++;

		ProcessInfo::getInstance().stopTimer("PrimalBoundSearchNLP");
		ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");

	}

	//std::cout << "Num NLP cands: " << ProcessInfo::getInstance().primalFixedNLPCandidates.size() << std::endl;
}

std::string TaskSelectPrimalFixedNLPPointsFromSolutionPool::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

