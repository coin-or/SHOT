/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsByObjectiveLinesearch.h"
namespace SHOT
{

TaskSelectHyperplanePointsByObjectiveLinesearch::TaskSelectHyperplanePointsByObjectiveLinesearch(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
}

TaskSelectHyperplanePointsByObjectiveLinesearch::~TaskSelectHyperplanePointsByObjectiveLinesearch()
{
}

void TaskSelectHyperplanePointsByObjectiveLinesearch::run()
{
    env->process->startTimer("DualObjectiveRootSearch");

    //env->process->setObjectiveUpdatedByLinesearch(false);

    auto prevIter = env->process->getPreviousIteration();

    for (int i = 0; i < prevIter->solutionPoints.size(); i++)
    {
        Hyperplane hyperplane;
        hyperplane.isObjectiveHyperplane = true;
        hyperplane.sourceConstraintIndex = -1;
        hyperplane.generatedPoint = prevIter->solutionPoints.at(i).point;
        hyperplane.source = E_HyperplaneSource::ObjectiveLinesearch;
        env->process->hyperplaneWaitingList.push_back(hyperplane);

        /*
        auto oldObjVal = prevIter->solutionPoints.at(i).objectiveValue;
        double objectiveLinearizationError = env->reformulatedProblem->objectiveFunction->calculateValue(prevIter->solutionPoints.at(i).point) - prevIter->solutionPoints.at(i).objectiveValue;

        double objectiveLB = prevIter->solutionPoints.at(i).objectiveValue;
        double objectiveUB = (1 + std::min(0.01, 1 / abs(prevIter->solutionPoints.at(i).objectiveValue))) * objectiveLinearizationError;

        try
        {
            auto xNewc = env->process->linesearchMethod->findZero(prevIter->solutionPoints.at(i).point, objectiveLB, objectiveUB,
                                                                  env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                  env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"), 0,
                                                                  std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction).get());

            auto internalPoint = xNewc.first;
            auto externalPoint = xNewc.second;

            double diffobj = abs(oldObjVal - xNewc.second);

            Hyperplane hyperplane;
            hyperplane.isObjectiveHyperplane = true;
            hyperplane.source = E_HyperplaneSource::ObjectiveLinesearch;
            hyperplane.sourceConstraintIndex = -1;
            hyperplane.generatedPoint = xNewc.second.point;

            env->process->hyperplaneWaitingList.push_back(hyperplane);

            env->output->outputAlways(
                "     Obj. for sol. # 0 upd. by l.s. " + UtilityFunctions::toString(oldObjVal) + " -> " + UtilityFunctions::toString(xNewc.second) + " (diff:" + UtilityFunctions::toString(diffobj) + ")  #");
        }
        catch (std::exception &e)
        {
            env->output->outputWarning(
                "     Cannot find solution with linesearch for updating nonlinear objective.");
        }*/
    }

    env->process->stopTimer("DualObjectiveRootSearch");
}

std::string TaskSelectHyperplanePointsByObjectiveLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

bool TaskSelectHyperplanePointsByObjectiveLinesearch::updateObjectiveInPoint(SolutionPoint &solution)
{
    auto oldObjVal = solution.objectiveValue;
    //double objectiveLinearizationError = env->reformulatedProblem->objectiveFunction->calculateValue(solution.point) - solution.point.back();

    double objectiveLB = solution.objectiveValue;
    double objectiveUB = (1 + std::min(0.01, 1 / abs(objectiveLB))) * env->reformulatedProblem->objectiveFunction->calculateValue(solution.point);

    auto tmp = env->reformulatedProblem->objectiveFunction->calculateValue(solution.point);

    bool changed = false;

    try
    {
        /*auto xNewc = env->process->linesearchMethod->findZero(solution.point, objectiveLB, objectiveUB,
                                                              env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                              env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"), 0,
                                                              std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction).get());

        double diffobj = abs(oldObjVal - xNewc.second);
*/
        Hyperplane hyperplane;
        hyperplane.isObjectiveHyperplane = true;
        hyperplane.sourceConstraintIndex = -1;
        hyperplane.generatedPoint = solution.point;
        hyperplane.source = E_HyperplaneSource::ObjectiveLinesearch;
        env->process->hyperplaneWaitingList.push_back(hyperplane);

        //env->output->outputAlways(
        //"     Obj. for sol. # 0 upd. by l.s. " + UtilityFunctions::toString(oldObjVal) + " -> " + UtilityFunctions::toString(xNewc.second) + " (diff:" + UtilityFunctions::toString(diffobj) + ")  #");
    }
    catch (std::exception &e)
    {
        env->output->outputWarning(
            "     Cannot find solution with linesearch for updating nonlinear objective.");
    }

    return (changed);
}
} // namespace SHOT