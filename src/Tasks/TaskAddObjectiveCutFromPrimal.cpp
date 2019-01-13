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

TaskAddObjectiveCutFromPrimal::TaskAddObjectiveCutFromPrimal(EnvironmentPtr envPtr, std::string taskIDTrue) : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
    env->timing->startTimer("DualStrategy");

    env->timing->stopTimer("DualStrategy");
}

TaskAddObjectiveCutFromPrimal::~TaskAddObjectiveCutFromPrimal()
{
}

void TaskAddObjectiveCutFromPrimal::run()
{
    env->timing->startTimer("DualStrategy");

    auto currIter = env->results->getCurrentIteration(); // The solved iteration

    if (previousCutOff == env->results->getPrimalBound())
    {
        numWithoutPrimalUpdate++;
    }
    else
    {
        numWithoutPrimalUpdate = 0;
    }

    if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal && (numCutOff < 10 || numWithoutPrimalUpdate > 50))
    {
        if (numCutOff == 0 || numWithoutPrimalUpdate > 50)
            previousCutOff = env->results->getPrimalBound();

        if (env->reformulatedProblem->objectiveFunction->properties.isMinimize)
        {
            previousCutOff = previousCutOff - 1;
            env->dualSolver->MIPSolver->setCutOffAsConstraint(previousCutOff);
            numCutOff++;
            std::cout << "objective cutoff = " << previousCutOff << '\n';
        }
        else
        {
            previousCutOff = previousCutOff + 1;
            env->dualSolver->MIPSolver->setCutOffAsConstraint(previousCutOff);
            numCutOff++;
            std::cout << "objective cutoff = " << previousCutOff << '\n';
        }

        env->tasks->setNextTask(taskIDIfTrue);
    }
    else
    {
        numCutOff = 0;
    }

    env->timing->stopTimer("DualStrategy");
}

std::string TaskAddObjectiveCutFromPrimal::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT