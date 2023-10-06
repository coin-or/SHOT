/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskRepairInfeasibleDualProblem.h"

#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

#include "../Model/Problem.h"

namespace SHOT
{

TaskRepairInfeasibleDualProblem::TaskRepairInfeasibleDualProblem(
    EnvironmentPtr envPtr, std::string taskIDTrue, std::string taskIDFalse)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue), taskIDIfFalse(taskIDFalse)
{
}

TaskRepairInfeasibleDualProblem::~TaskRepairInfeasibleDualProblem() = default;

void TaskRepairInfeasibleDualProblem::run()
{
    env->timing->startTimer("DualStrategy");

    auto currIter = env->results->getCurrentIteration();

    if(currIter->solutionStatus != E_ProblemSolutionStatus::Infeasible)
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    if(env->solutionStatistics.numberOfDualRepairsSinceLastPrimalUpdate
        >= env->settings->getSetting<int>("MIP.InfeasibilityRepair.IterationLimit", "Dual"))
    {
        env->tasks->setNextTask(taskIDIfFalse);
        return;
    }

    // Loop detection
    if(auto optional = env->results->getLastFeasibleIteration();
        env->solutionStatistics.numberOfDualRepairsSinceLastPrimalUpdate > 1 && optional)
    {
        bool noNewSolutions = true;

        for(auto& SCURR : currIter->solutionPoints)
        {
            bool solutionFound = false;

            for(auto& SFEAS : optional->get()->solutionPoints)
            {
                if(SCURR.hashValue == SFEAS.hashValue)
                {
                    solutionFound = true;
                    break;
                }
            }

            if(!solutionFound)
            {
                noNewSolutions = false;
                break;
            }
        }

        if(noNewSolutions)
        {
            currIter->forceObjectiveReductionCut = true;
            env->tasks->setNextTask(taskIDIfFalse);

            std::stringstream tmpType;
            tmpType << "REP-LOOP";

            env->report->outputIterationDetail(totRepairTries, tmpType.str(), env->timing->getElapsedTime("Total"),
                currIter->numberOfInfeasibilityRepairedConstraints, 0, 0, env->dualSolver->cutOffToUse, 0, 0, 0, 0,
                currIter->maxDeviation, E_IterationLineType::DualRepair, true);

            return;
        }
    }

    auto currentSolutionLimit = env->dualSolver->MIPSolver->getSolutionLimit();

    currIter->hasInfeasibilityRepairBeenPerformed = true;

    env->dualSolver->MIPSolver->setTimeLimit(
        env->settings->getSetting<double>("MIP.InfeasibilityRepair.TimeLimit", "Dual"));

    // Otherwise repair problem might not be solved to optimality
    env->dualSolver->MIPSolver->setSolutionLimit(2100000000);

    std::stringstream tmpType;
    tmpType << "REP";

    if(env->dualSolver->MIPSolver->repairInfeasibility())
    {
        env->tasks->setNextTask(taskIDIfTrue);
        iterLastRepair = currIter->iterationNumber;
        env->solutionStatistics.hasInfeasibilityRepairBeenPerformedSincePrimalImprovement = true;
        env->solutionStatistics.numberOfSuccessfulDualRepairsPerformed++;

        currIter->wasInfeasibilityRepairSuccessful = true;
        tmpType << "-SUCC";
    }
    else if(mainRepairTries < 2)
    {
        currIter->wasInfeasibilityRepairSuccessful = false;
        env->solutionStatistics.numberOfUnsuccessfulDualRepairsPerformed++;

        env->dualSolver->cutOffToUse = env->results->getPrimalBound();
        env->solutionStatistics.numberOfDualRepairsSinceLastPrimalUpdate = 0;
        env->tasks->setNextTask(taskIDIfTrue);
        mainRepairTries++;
        tmpType << "-F-" << mainRepairTries;
    }
    else
    {
        currIter->wasInfeasibilityRepairSuccessful = false;
        env->solutionStatistics.numberOfUnsuccessfulDualRepairsPerformed++;

        env->tasks->setNextTask(taskIDIfFalse);
        mainRepairTries++;
        tmpType << "-F-" << mainRepairTries;
    }

    totRepairTries++;

    env->dualSolver->MIPSolver->setSolutionLimit(currentSolutionLimit);

    env->solutionStatistics.numberOfDualRepairsSinceLastPrimalUpdate++;
    env->results->solutionIsGlobal = false;

    env->report->outputIterationDetail(totRepairTries, tmpType.str(), env->timing->getElapsedTime("Total"),
        currIter->numberOfInfeasibilityRepairedConstraints, 0, 0, env->dualSolver->cutOffToUse, 0, 0, 0, 0,
        currIter->maxDeviation, E_IterationLineType::DualRepair, true);

    env->timing->stopTimer("DualStrategy");
}

std::string TaskRepairInfeasibleDualProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT