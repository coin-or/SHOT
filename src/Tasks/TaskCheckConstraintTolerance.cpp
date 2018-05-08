/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckConstraintTolerance.h"

TaskCheckConstraintTolerance::TaskCheckConstraintTolerance(EnvironmentPtr envPtr, std::string taskIDTrue): TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckConstraintTolerance::~TaskCheckConstraintTolerance() {}

void TaskCheckConstraintTolerance::run()
{
    if (!isInitialized)
    {
        this->isObjectiveNonlinear = env->process->problemStats.isObjectiveNonlinear();
        this->nonlinearConstraintIndexes = env->process->originalProblem->getNonlinearConstraintIndexes();

        if (this->isObjectiveNonlinear)
        {
            this->nonlinearObjectiveConstraintIndex = env->process->originalProblem->getNonlinearObjectiveConstraintIdx();

            // Removes the nonlinear constraint index from the list
            std::vector<int>::iterator position = std::find(this->nonlinearConstraintIndexes.begin(), this->nonlinearConstraintIndexes.end(), -1);
            if (position != this->nonlinearConstraintIndexes.end()) // means the element was not found
                this->nonlinearConstraintIndexes.erase(position);

            position = std::find(this->nonlinearConstraintIndexes.begin(), this->nonlinearConstraintIndexes.end(), this->nonlinearObjectiveConstraintIndex);
            if (position != this->nonlinearConstraintIndexes.end()) // means the element was not found
                this->nonlinearConstraintIndexes.erase(position);
        }
    }

    auto currIter = env->process->getCurrentIteration();
    auto solutionPoint = currIter->solutionPoints.at(0).point;

    // Checks if the nonlinear constraints are fulfilled to tolerance
    if (this->nonlinearConstraintIndexes.size() > 0)
    {
        auto maxDev = env->process->originalProblem->getMostDeviatingConstraint(solutionPoint, this->nonlinearConstraintIndexes).first;

        if (maxDev.value >= env->settings->getDoubleSetting("ConstraintTolerance", "Termination"))
            return;
    }

    // Checks if objective constraint is fulfilled to tolerance
    if (this->isObjectiveNonlinear)
    {
        double objDev = env->process->originalProblem->calculateConstraintFunctionValue(this->nonlinearObjectiveConstraintIndex, solutionPoint);

        if (objDev >= env->settings->getDoubleSetting("ObjectiveConstraintTolerance", "Termination"))
            return;
    }

    if (env->process->problemStats.isDiscreteProblem)
    {
        if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal &&
            currIter->type == E_IterationProblemType::MIP)
        {
            env->process->terminationReason = E_TerminationReason::ConstraintTolerance;
            env->process->tasks->setNextTask(taskIDIfTrue);
        }
    }
    else
    {
        if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
        {
            env->process->terminationReason = E_TerminationReason::ConstraintTolerance;
            env->process->tasks->setNextTask(taskIDIfTrue);
        }
    }

    return;
}

std::string TaskCheckConstraintTolerance::getType()
{
    std::string type = typeid(this).name();
    return (type);
}