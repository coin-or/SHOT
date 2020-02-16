/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsByObjectiveRootsearch.h"

#include "../DualSolver.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Utilities.h"
#include "../Timing.h"

#include "../Model/Problem.h"

#include "../RootsearchMethod/IRootsearchMethod.h"

namespace SHOT
{

TaskSelectHyperplanePointsByObjectiveRootsearch::TaskSelectHyperplanePointsByObjectiveRootsearch(EnvironmentPtr envPtr)
    : TaskBase(envPtr)
{
}

TaskSelectHyperplanePointsByObjectiveRootsearch::~TaskSelectHyperplanePointsByObjectiveRootsearch() = default;

void TaskSelectHyperplanePointsByObjectiveRootsearch::run()
{
    this->run(env->results->getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsByObjectiveRootsearch::run(std::vector<SolutionPoint> sourcePoints)
{
    if(sourcePoints.size() == 0)
        return;

    env->timing->startTimer("DualObjectiveRootSearch");

    env->output->outputDebug("        Selecting separating hyperplanes for objective function:");

    bool useRootsearch = false;

    if(useRootsearch)
    {
        for(auto& SOLPT : sourcePoints)
        {
            double objectiveLinearizationError = std::abs(
                env->reformulatedProblem->objectiveFunction->calculateValue(SOLPT.point) - SOLPT.objectiveValue);

            double objectiveLB = SOLPT.objectiveValue;
            double objectiveUB = (1 + std::min(0.01, 1 / std::abs(SOLPT.objectiveValue))) * objectiveLinearizationError;

            if(objectiveLinearizationError == 0)
                continue;

            try
            {
                auto rootBound = env->rootsearchMethod->findZero(SOLPT.point, objectiveLB, objectiveUB,
                    env->settings->getSetting<int>("Rootsearch.MaxIterations", "Subsolver"),
                    env->settings->getSetting<double>("Rootsearch.TerminationTolerance", "Subsolver"), 0,
                    std::dynamic_pointer_cast<NonlinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                        .get());

                Hyperplane hyperplane;
                hyperplane.isObjectiveHyperplane = true;
                hyperplane.source = E_HyperplaneSource::ObjectiveRootsearch;
                hyperplane.sourceConstraintIndex = -1;
                hyperplane.generatedPoint = SOLPT.point;
                hyperplane.objectiveFunctionValue = rootBound.second;

                env->dualSolver->addHyperplane(hyperplane);
            }
            catch(std::exception& e)
            {
                env->output->outputWarning(
                    "         Cannot find solution with root search for generating objective supporting hyperplane:");
                env->output->outputWarning(e.what());
            }
        }
    }
    else
    {

        if(env->solutionStatistics.numberOfIterationsWithDualStagnation > 2
            && env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Convex)
        {
            Hyperplane hyperplane;
            hyperplane.isObjectiveHyperplane = true;
            hyperplane.sourceConstraintIndex = -1;
            hyperplane.generatedPoint = sourcePoints[0].point;
            hyperplane.source = E_HyperplaneSource::ObjectiveRootsearch;

            hyperplane.objectiveFunctionValue = 0.0;
            /*= env->reformulatedProblem->objectiveFunction->calculateValue(hyperplane.generatedPoint) - 0.01;*/

            env->dualSolver->addHyperplane(hyperplane);

            env->output->outputWarning("         Adding objective cutting plane since the dual has stagnated.");
        }

        bool isConvex = env->reformulatedProblem->objectiveFunction->properties.convexity == E_Convexity::Linear
            || ((env->reformulatedProblem->objectiveFunction->properties.isMinimize
                    && env->reformulatedProblem->objectiveFunction->properties.convexity == E_Convexity::Convex)
                   || (env->reformulatedProblem->objectiveFunction->properties.isMaximize
                          && env->reformulatedProblem->objectiveFunction->properties.convexity
                              == E_Convexity::Concave));

        if(!isConvex && env->results->getCurrentIteration()->numHyperplanesAdded > 0)
        {
            // Nonconvex objective function, do not add a cut if not necessary
            env->output->outputDebug("         No need to add cut to nonconvex objective function.");
            return;
        }
        else
        {
            env->output->outputDebug("         Adding cut to nonconvex objective function.");
        }

        for(auto& SOLPT : sourcePoints)
        {
            Hyperplane hyperplane;
            hyperplane.isObjectiveHyperplane = true;
            hyperplane.sourceConstraintIndex = -1;
            hyperplane.generatedPoint = SOLPT.point;
            hyperplane.source = E_HyperplaneSource::ObjectiveRootsearch;

            hyperplane.objectiveFunctionValue
                = env->reformulatedProblem->objectiveFunction->calculateValue(hyperplane.generatedPoint);

            env->dualSolver->addHyperplane(hyperplane);
        }
    }

    env->timing->stopTimer("DualObjectiveRootSearch");
}

std::string TaskSelectHyperplanePointsByObjectiveRootsearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT