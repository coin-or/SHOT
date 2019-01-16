/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskAddObjectiveCutFromPrimal.h"

namespace SHOT
{

TaskAddObjectiveCutFromPrimal::TaskAddObjectiveCutFromPrimal(EnvironmentPtr envPtr, std::string taskIDTrue)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
    if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
    {
        previousCutOffOriginal = SHOT_DBL_MAX;
        previousCutOffModified = SHOT_DBL_MAX;
    }
    else
    {
        previousCutOffOriginal = SHOT_DBL_MIN;
        previousCutOffModified = SHOT_DBL_MIN;
    }
}

TaskAddObjectiveCutFromPrimal::~TaskAddObjectiveCutFromPrimal() {}

void TaskAddObjectiveCutFromPrimal::run()
{
    auto currIter = env->results->getCurrentIteration(); // The solved iteration

    std::cout << "********in TaskAddObjectiveCutFromPrimal\n";

    if(currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible && !currIter->wasInfeasibilityRepairSuccessful)
    {
        // .return;
    }

    if(previousCutOffOriginal == env->results->getPrimalBound()) // The primal has not been improved
    {
        numWithoutPrimalUpdate++;
        std::cout << "        iters without update: " << numWithoutPrimalUpdate << '\n';
    }
    else
    {
        numWithoutPrimalUpdate = 0;
        numCutOff = 0;
        previousCutOffOriginal = env->results->getPrimalBound();

        if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
        {
            previousCutOffModified = previousCutOffOriginal;
        }
        else
        {
            previousCutOffModified = previousCutOffOriginal;
        }

        // env->solutionStatistics.numberOfIterationsWithSignificantObjectiveUpdate = currIter->iterationNumber;
        // env->solutionStatistics.numberOfIterationsWithStagnationMIP = 0;
    }

    if(numCutOff > 10)
    {
        std::cout << "        will not add more cuts since maximum " << numCutOff << " is reached\n";
        return;
    }

    // if(currIter->solutionStatus == E_ProblemSolutionStatus::Optimal && (numCutOff < 10 || numWithoutPrimalUpdate >
    // 10))
    if(true || currIter->solutionStatus == E_ProblemSolutionStatus::Optimal && (numWithoutPrimalUpdate > 10))
    {
        if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
        {
            previousCutOffModified = previousCutOffModified - 0.01;
            env->dualSolver->MIPSolver->setCutOffAsConstraint(previousCutOffModified);
            // env->dualSolver->MIPSolver->setCutOff(previousCutOffModified);
            numCutOff++;
            std::cout << "        objective cutoff = " << previousCutOffModified << " number: " << numCutOff << '\n';
            env->results->currentDualBound = SHOT_DBL_MIN;
            // env->dualSolver->MIPSolver->setSolutionLimit(2100000000);
        }
        else
        {
            previousCutOffModified = previousCutOffModified + 0.01;
            env->dualSolver->MIPSolver->setCutOffAsConstraint(previousCutOffModified);
            // env->dualSolver->MIPSolver->setCutOff(previousCutOffModified);
            numCutOff++;
            std::cout << "        objective cutoff = " << previousCutOffModified << " number: " << numCutOff << '\n';
            env->results->currentDualBound = SHOT_DBL_MAX;
            // env->dualSolver->MIPSolver->setSolutionLimit(2100000000);
        }

        env->tasks->setNextTask(taskIDIfTrue);
    }
    else
    {
        numCutOff = 0;
    }
}

std::string TaskAddObjectiveCutFromPrimal::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT