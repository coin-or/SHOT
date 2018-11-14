/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsByObjectiveLinesearch.h"

TaskSelectHyperplanePointsByObjectiveLinesearch::TaskSelectHyperplanePointsByObjectiveLinesearch(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
}

TaskSelectHyperplanePointsByObjectiveLinesearch::~TaskSelectHyperplanePointsByObjectiveLinesearch()
{
}

void TaskSelectHyperplanePointsByObjectiveLinesearch::run()
{
    env->process->startTimer("DualObjectiveLiftRootSearch");

    env->process->setObjectiveUpdatedByLinesearch(false);

    auto currIter = env->process->getCurrentIteration();

    auto allSolutions = currIter->solutionPoints;

    for (int i = 0; i < allSolutions.size(); i++)
    {
        if (allSolutions.at(i).maxDeviation.value < 0)
            continue;

        auto oldObjVal = allSolutions.at(i).objectiveValue;
        auto changed = updateObjectiveInPoint(allSolutions.at(i));

        // Update the iteration solution as well (for i==0)
        if (changed && i == 0 /*&& (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal*/)
        {
            currIter->maxDeviation = allSolutions.at(i).maxDeviation.value;
            currIter->maxDeviationConstraint = allSolutions.at(i).maxDeviation.index;
            currIter->objectiveValue = allSolutions.at(0).objectiveValue;

            env->process->setObjectiveUpdatedByLinesearch(true);

            auto diffobj = abs(oldObjVal - allSolutions.at(i).objectiveValue);

            // Change the status of the solution if it has been updated much
            if (diffobj > env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination"))
            {
                if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
                {
                    currIter->solutionStatus = E_ProblemSolutionStatus::SolutionLimit;
                }
            }
        }
    }

    env->process->stopTimer("DualObjectiveLiftRootSearch");
}

std::string TaskSelectHyperplanePointsByObjectiveLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

bool TaskSelectHyperplanePointsByObjectiveLinesearch::updateObjectiveInPoint(SolutionPoint &solution)
{
    auto oldObjVal = solution.objectiveValue;
    double objectiveLinearizationError = env->reformulatedProblem->objectiveFunction->calculateValue(solution.point) - solution.point.back();

    double objectiveLB = solution.objectiveValue;
    double objectiveUB = objectiveLB + (1 + std::min(0.01, 1 / abs(solution.objectiveValue))) * objectiveLinearizationError;

    bool changed = false;

    try
    {
        auto xNewc = env->process->linesearchMethod->findZero(solution.point, objectiveLB, objectiveUB,
                                                              env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                              env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"), 0,
                                                              std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction));

        double diffobj = abs(oldObjVal - xNewc.second);

        Hyperplane hyperplane;
        hyperplane.sourceConstraintIndex = -1;
        hyperplane.generatedPoint = solution.point;
        hyperplane.source = E_HyperplaneSource::ObjectiveLinesearch;
        env->process->hyperplaneWaitingList.push_back(hyperplane);

        env->output->outputInfo(
            "     Obj. for sol. # 0 upd. by l.s. " + UtilityFunctions::toString(oldObjVal) + " -> " + UtilityFunctions::toString(xNewc.second) + " (diff:" + UtilityFunctions::toString(diffobj) + ")  #");
    }
    catch (std::exception &e)
    {
        env->output->outputWarning(
            "     Cannot find solution with linesearch for updating nonlinear objective.");
    }

    return (changed);
}
