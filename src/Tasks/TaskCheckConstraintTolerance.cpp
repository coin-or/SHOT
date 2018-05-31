/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckConstraintTolerance.h"

TaskCheckConstraintTolerance::TaskCheckConstraintTolerance(
    std::string taskIDTrue)
{
    taskIDIfTrue = taskIDTrue;
}

TaskCheckConstraintTolerance::~TaskCheckConstraintTolerance() {}

void TaskCheckConstraintTolerance::run()
{
    if (!isInitialized)
    {
        this->isObjectiveNonlinear = ProcessInfo::getInstance().problemStats.isObjectiveNonlinear();
        this->nonlinearConstraintIndexes = ProcessInfo::getInstance().originalProblem->getNonlinearConstraintIndexes();

        if (this->isObjectiveNonlinear)
        {
            this->nonlinearObjectiveConstraintIndex = ProcessInfo::getInstance().originalProblem->getNonlinearObjectiveConstraintIdx();

            // Removes the nonlinear constraint index from the list
            std::vector<int>::iterator position = std::find(this->nonlinearConstraintIndexes.begin(), this->nonlinearConstraintIndexes.end(), -1);
            if (position != this->nonlinearConstraintIndexes.end()) // means the element was not found
                this->nonlinearConstraintIndexes.erase(position);

            position = std::find(this->nonlinearConstraintIndexes.begin(), this->nonlinearConstraintIndexes.end(), this->nonlinearObjectiveConstraintIndex);
            if (position != this->nonlinearConstraintIndexes.end()) // means the element was not found
                this->nonlinearConstraintIndexes.erase(position);
        }
    }

    auto currIter = ProcessInfo::getInstance().getCurrentIteration();
    auto solutionPoint = currIter->solutionPoints.at(0).point;

    // Checks if the nonlinear constraints are fulfilled to tolerance
    if (this->nonlinearConstraintIndexes.size() > 0)
    {
        auto maxDev = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solutionPoint, this->nonlinearConstraintIndexes).first;

        if (maxDev.value >= Settings::getInstance().getDoubleSetting("ConstraintTolerance", "Termination"))
            return;
    }

    // Checks if objective constraint is fulfilled to tolerance
    if (this->isObjectiveNonlinear)
    {
        double objDev = ProcessInfo::getInstance().originalProblem->calculateConstraintFunctionValue(this->nonlinearObjectiveConstraintIndex, solutionPoint);

        if (objDev >= Settings::getInstance().getDoubleSetting("ObjectiveConstraintTolerance", "Termination"))
            return;
    }

    if (ProcessInfo::getInstance().problemStats.isDiscreteProblem)
    {
        if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal &&
            currIter->type == E_IterationProblemType::MIP)
        {
            ProcessInfo::getInstance().terminationReason = E_TerminationReason::ConstraintTolerance;
            ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
        }
    }
    else
    {
        if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
        {
            ProcessInfo::getInstance().terminationReason = E_TerminationReason::ConstraintTolerance;
            ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
        }
    }

    return;
}

std::string TaskCheckConstraintTolerance::getType()
{
    std::string type = typeid(this).name();
    return (type);
}