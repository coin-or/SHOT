/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsByObjectiveLinesearch.h"

#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Utilities.h"
#include "../Timing.h"

#include "../Model/Problem.h"

#include "../LinesearchMethod/ILinesearchMethod.h"

namespace SHOT
{

TaskSelectHyperplanePointsByObjectiveLinesearch::TaskSelectHyperplanePointsByObjectiveLinesearch(EnvironmentPtr envPtr)
    : TaskBase(envPtr)
{
}

TaskSelectHyperplanePointsByObjectiveLinesearch::~TaskSelectHyperplanePointsByObjectiveLinesearch() {}

void TaskSelectHyperplanePointsByObjectiveLinesearch::run()
{
    this->run(env->results->getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsByObjectiveLinesearch::run(std::vector<SolutionPoint> sourcePoints)
{
    env->timing->startTimer("DualObjectiveRootSearch");

    bool useRootsearch = false;

    if(useRootsearch)
    {
        for(auto& SOLPT : sourcePoints)
        {
            double oldObjVal = SOLPT.objectiveValue;
            double objectiveLinearizationError = std::abs(
                env->reformulatedProblem->objectiveFunction->calculateValue(SOLPT.point) - SOLPT.objectiveValue);

            double objectiveLB = SOLPT.objectiveValue;
            double objectiveUB = (1 + std::min(0.01, 1 / abs(SOLPT.objectiveValue))) * objectiveLinearizationError;

            if(objectiveLinearizationError == 0)
                continue;

            try
            {
                auto rootBound = env->rootsearchMethod->findZero(SOLPT.point, objectiveLB, objectiveUB,
                    env->settings->getSetting<int>("Rootsearch.MaxIterations", "Subsolver"),
                    env->settings->getSetting<double>("Rootsearch.TerminationTolerance", "Subsolver"), 0,
                    std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                        .get());

                double diffobj = abs(oldObjVal - rootBound.second);

                Hyperplane hyperplane;
                hyperplane.isObjectiveHyperplane = true;
                hyperplane.source = E_HyperplaneSource::ObjectiveLinesearch;
                hyperplane.sourceConstraintIndex = -1;
                hyperplane.generatedPoint = SOLPT.point;
                hyperplane.objectiveFunctionValue = rootBound.second;

                env->dualSolver->MIPSolver->hyperplaneWaitingList.push_back(hyperplane);
            }
            catch(std::exception& e)
            {
                env->output->outputCritical(
                    "     Cannot find solution with root search for generating objective supporting hyperplane:");
                env->output->outputCritical(e.what());
            }
        }
    }
    else
    {
        for(auto& SOLPT : sourcePoints)
        {
            Hyperplane hyperplane;
            hyperplane.isObjectiveHyperplane = true;
            hyperplane.sourceConstraintIndex = -1;
            hyperplane.generatedPoint = SOLPT.point;
            hyperplane.source = E_HyperplaneSource::ObjectiveLinesearch;

            if(env->reformulatedProblem->objectiveFunction->properties.hasNonlinearExpression)
            {
                hyperplane.objectiveFunctionValue
                    = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                          ->calculateValue(hyperplane.generatedPoint);
            }
            else
            {
                hyperplane.objectiveFunctionValue
                    = std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                          ->calculateValue(hyperplane.generatedPoint);
            }

            env->dualSolver->MIPSolver->hyperplaneWaitingList.push_back(hyperplane);
        }
    }

    env->timing->stopTimer("DualObjectiveRootSearch");
}

std::string TaskSelectHyperplanePointsByObjectiveLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT