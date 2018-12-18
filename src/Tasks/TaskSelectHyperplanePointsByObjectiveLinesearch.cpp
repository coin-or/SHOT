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
    this->run(env->process->getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsByObjectiveLinesearch::run(std::vector<SolutionPoint> sourcePoints)
{
    env->process->startTimer("DualObjectiveRootSearch");

    bool useRootsearch = false;

    if (useRootsearch)
    {
        for (auto &SOLPT : sourcePoints)
        {
            double oldObjVal = SOLPT.objectiveValue;
            double objectiveLinearizationError = std::abs(env->reformulatedProblem->objectiveFunction->calculateValue(SOLPT.point) - SOLPT.objectiveValue);

            double objectiveLB = SOLPT.objectiveValue;
            double objectiveUB = (1 + std::min(0.01, 1 / abs(SOLPT.objectiveValue))) * objectiveLinearizationError;

            if (objectiveLinearizationError == 0)
                continue;

            try
            {
                auto rootBound = env->process->linesearchMethod->findZero(SOLPT.point, objectiveLB, objectiveUB,
                                                                          env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                          env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"), 0,
                                                                          std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction).get());

                double diffobj = abs(oldObjVal - rootBound.second);

                Hyperplane hyperplane;
                hyperplane.isObjectiveHyperplane = true;
                hyperplane.source = E_HyperplaneSource::ObjectiveLinesearch;
                hyperplane.sourceConstraintIndex = -1;
                hyperplane.generatedPoint = SOLPT.point;
                hyperplane.objectiveFunctionValue = rootBound.second;

                env->process->hyperplaneWaitingList.push_back(hyperplane);
            }
            catch (std::exception &e)
            {
                env->output->outputAlways("     Cannot find solution with root search for generating objective supporting hyperplane:");
                env->output->outputAlways(e.what());
            }
        }
    }
    else
    {
        for (auto &SOLPT : sourcePoints)
        {
            Hyperplane hyperplane;
            hyperplane.isObjectiveHyperplane = true;
            hyperplane.sourceConstraintIndex = -1;
            hyperplane.generatedPoint = SOLPT.point;
            hyperplane.source = E_HyperplaneSource::ObjectiveLinesearch;
            hyperplane.objectiveFunctionValue = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)->calculateValue(hyperplane.generatedPoint);

            env->process->hyperplaneWaitingList.push_back(hyperplane);
        }
    }

    env->process->stopTimer("DualObjectiveRootSearch");
}

std::string TaskSelectHyperplanePointsByObjectiveLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT