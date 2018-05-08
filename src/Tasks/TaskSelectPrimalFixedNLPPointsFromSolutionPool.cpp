/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectPrimalFixedNLPPointsFromSolutionPool.h"

TaskSelectPrimalFixedNLPPointsFromSolutionPool::TaskSelectPrimalFixedNLPPointsFromSolutionPool(EnvironmentPtr envPtr): TaskBase(envPtr)
{
}

TaskSelectPrimalFixedNLPPointsFromSolutionPool::~TaskSelectPrimalFixedNLPPointsFromSolutionPool()
{
}

void TaskSelectPrimalFixedNLPPointsFromSolutionPool::run()
{
	auto currIter = env->process->getCurrentIteration();
	auto allSolutions = env->process->getCurrentIteration()->solutionPoints;

	bool callNLPSolver = false;
	bool useFeasibleSolutionExtra = false;

	if (!currIter->isMIP())
	{
		return;
	}

	if (allSolutions.size() == 0)
	{
		return;
	}

	if (currIter->MIPSolutionLimitUpdated && currIter->solutionStatus != E_ProblemSolutionStatus::Optimal)
	{
		env->process->solutionStatistics.numberOfIterationsWithoutNLPCallMIP++;
		return;
	}

	env->process->startTimer("PrimalStrategy");
	env->process->startTimer("PrimalBoundStrategyNLP");

	auto userSettingStrategy = env->settings->getIntSetting("FixedInteger.CallStrategy", "Primal");
	auto userSetting = env->settings->getIntSetting("FixedInteger.Source", "Primal");

	auto dualBound = env->process->getDualBound();

	if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal &&
		abs(allSolutions.at(0).objectiveValue - env->process->getDualBound()) / ((1e-10) + abs(dualBound)) < env->settings->getDoubleSetting("FixedInteger.DualPointGap.Relative", "Primal"))
	{
		callNLPSolver = true;
		useFeasibleSolutionExtra = true;
	}
	else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::AlwaysUse))
	{
		callNLPSolver = true;
	}
	else if (userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTime) || userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
	{
		if (env->process->solutionStatistics.numberOfIterationsWithoutNLPCallMIP >= env->settings->getIntSetting("FixedInteger.Frequency.Iteration", "Primal"))
		{
			env->output->outputInfo(
				"     Activating fixed NLP primal strategy since max iterations since last call has been reached.");
			callNLPSolver = true;
		}
		else if (env->process->getElapsedTime("Total") - env->process->solutionStatistics.timeLastFixedNLPCall> env->settings->getDoubleSetting("FixedInteger.Frequency.Time", "Primal"))
		{
			env->output->outputInfo(
				"     Activating fixed NLP primal strategy since max time limit since last call has been reached.");
			callNLPSolver = true;
		}
	}

	if (useFeasibleSolutionExtra)
	{
		auto tmpSol = allSolutions.at(0);
		env->process->addPrimalFixedNLPCandidate(tmpSol.point,
															  E_PrimalNLPSource::FirstSolutionNewDualBound, tmpSol.objectiveValue, tmpSol.iterFound,
															  tmpSol.maxDeviation);
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::SmallestDeviationSolution))
	{
		auto tmpSol = currIter->getSolutionPointWithSmallestDeviation();
		env->process->addPrimalFixedNLPCandidate(tmpSol.point,
															  E_PrimalNLPSource::SmallestDeviationSolution, tmpSol.objectiveValue, tmpSol.iterFound,
															  tmpSol.maxDeviation);
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::FirstSolution))
	{
		auto tmpSol = allSolutions.at(0);
		env->process->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
															  tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::FirstAndFeasibleSolutions))
	{
		auto tmpSol = allSolutions.at(0);
		env->process->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
															  tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);

		auto smallestDevSolIdx = currIter->getSolutionPointWithSmallestDeviationIndex();

		if (smallestDevSolIdx != 0)
		{
			tmpSol = allSolutions.at(smallestDevSolIdx);
			env->process->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
																  tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
		}
	}
	else if (callNLPSolver && userSettingStrategy == static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions))
	{
		auto tmpSol = allSolutions.at(0);

		env->process->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
															  tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);

		for (int i = 1; i < allSolutions.size(); i++)
		{
			auto tmpSol = allSolutions.at(i);

			if (tmpSol.maxDeviation.value <= env->settings->getDoubleSetting("Tolerance.NonlinearConstraint", "Primal"))
			{
				env->process->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
																	  tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
			}
		}
	}
	else if (callNLPSolver && userSetting == static_cast<int>(ES_PrimalNLPFixedPoint::AllSolutions))
	{
		auto tmpSol = allSolutions.at(0);

		env->process->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FirstSolution,
															  tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);

		for (int i = 1; i < allSolutions.size(); i++)
		{
			tmpSol = allSolutions.at(i);

			if (tmpSol.maxDeviation.value <= env->settings->getDoubleSetting("Tolerance.NonlinearConstraint", "Primal"))
			{
				env->process->addPrimalFixedNLPCandidate(tmpSol.point, E_PrimalNLPSource::FeasibleSolution,
																	  tmpSol.objectiveValue, tmpSol.iterFound, tmpSol.maxDeviation);
			}
			else
			{
				env->process->addPrimalFixedNLPCandidate(tmpSol.point,
																	  E_PrimalNLPSource::InfeasibleSolution, tmpSol.objectiveValue, tmpSol.iterFound,
																	  tmpSol.maxDeviation);
			}
		}
	}
	else
	{
		env->process->stopTimer("PrimalBoundStrategyNLP");
		env->process->stopTimer("PrimalStrategy");
	}
}

std::string TaskSelectPrimalFixedNLPPointsFromSolutionPool::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
