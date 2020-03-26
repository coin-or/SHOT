/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanePointsObjectiveFunction.h"

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

TaskSelectHyperplanePointsObjectiveFunction::TaskSelectHyperplanePointsObjectiveFunction(EnvironmentPtr envPtr)
    : TaskBase(envPtr)
{
}

TaskSelectHyperplanePointsObjectiveFunction::~TaskSelectHyperplanePointsObjectiveFunction() = default;

void TaskSelectHyperplanePointsObjectiveFunction::run()
{
    this->run(env->results->getPreviousIteration()->solutionPoints);
}

void TaskSelectHyperplanePointsObjectiveFunction::run(std::vector<SolutionPoint> sourcePoints)
{
    if(sourcePoints.size() == 0)
        return;

    int numHyperplaneAdded = 0;

    // Add cutting plane if the dual has stagnated
    if(env->solutionStatistics.numberOfIterationsWithDualStagnation > 2
        && env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Convex)
    {
        Hyperplane hyperplane;
        hyperplane.isObjectiveHyperplane = true;
        hyperplane.sourceConstraintIndex = -1;
        hyperplane.generatedPoint = sourcePoints[0].point;
        hyperplane.source = E_HyperplaneSource::ObjectiveRootsearch;
        hyperplane.objectiveFunctionValue = 0.0;
        hyperplane.isSourceConvex = true;

        env->dualSolver->addHyperplane(hyperplane);
        numHyperplaneAdded++;

        env->output->outputWarning("         Adding objective cutting plane since the dual has stagnated.");
    }

    bool isConvex = env->reformulatedProblem->objectiveFunction->properties.convexity == E_Convexity::Linear
        || ((env->reformulatedProblem->objectiveFunction->properties.isMinimize
                && env->reformulatedProblem->objectiveFunction->properties.convexity == E_Convexity::Convex)
               || (env->reformulatedProblem->objectiveFunction->properties.isMaximize
                      && env->reformulatedProblem->objectiveFunction->properties.convexity == E_Convexity::Concave));

    if(!isConvex && (env->results->getCurrentIteration()->numHyperplanesAdded > 0 || numHyperplaneAdded > 0))
    {
        // Nonconvex objective function, do not add a cut if not necessary
        env->output->outputDebug("         No need to add cut to nonconvex objective function.");
        return;
    }
    else if(!isConvex)
    {
        env->output->outputDebug("         Will add cut to nonconvex objective function.");
    }

    auto strategy = static_cast<ES_ObjectiveRootsearch>(
        env->settings->getSetting<int>("HyperplaneCuts.ObjectiveRootSearch", "Dual"));

    bool useRootsearch = true;

    if(strategy == ES_ObjectiveRootsearch::Never)
        useRootsearch = false;
    else if(strategy == ES_ObjectiveRootsearch::IfConvex
        && env->reformulatedProblem->properties.convexity > E_ProblemConvexity::Convex)
        useRootsearch = false;

    if(useRootsearch)
    {
        env->timing->startTimer("DualObjectiveRootSearch");

        for(auto& SOLPT : sourcePoints)
        {
            double objectiveLinearizationError = std::abs(
                env->reformulatedProblem->objectiveFunction->calculateValue(SOLPT.point) - SOLPT.objectiveValue);

            if(objectiveLinearizationError < 1e-7)
                continue;

            auto exactValue = env->reformulatedProblem->objectiveFunction->calculateValue(SOLPT.point);

            double factor = std::min(0.01, 1 / std::abs(SOLPT.objectiveValue));
            double objectiveLB = SOLPT.objectiveValue;
            double objectiveUB = (exactValue < 0)
                ? (1 - factor) * env->reformulatedProblem->objectiveFunction->calculateValue(SOLPT.point)
                : (1 + factor) * env->reformulatedProblem->objectiveFunction->calculateValue(SOLPT.point);

            try
            {
                PairDouble rootBound;

                if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
                {
                    rootBound = env->rootsearchMethod->findZero(SOLPT.point, objectiveLB, objectiveUB,
                        env->settings->getSetting<int>("Rootsearch.MaxIterations", "Subsolver"),
                        env->settings->getSetting<double>("Rootsearch.TerminationTolerance", "Subsolver"), 0,
                        env->reformulatedProblem->objectiveFunction);
                }
                else
                {
                    rootBound = env->rootsearchMethod->findZero(SOLPT.point, objectiveUB, objectiveLB,
                        env->settings->getSetting<int>("Rootsearch.MaxIterations", "Subsolver"),
                        env->settings->getSetting<double>("Rootsearch.TerminationTolerance", "Subsolver"), 0,
                        env->reformulatedProblem->objectiveFunction);
                }

                Hyperplane hyperplane;
                hyperplane.isObjectiveHyperplane = true;
                hyperplane.source = E_HyperplaneSource::ObjectiveRootsearch;
                hyperplane.sourceConstraintIndex = -1;
                hyperplane.generatedPoint = SOLPT.point;
                hyperplane.objectiveFunctionValue = rootBound.second;
                hyperplane.isSourceConvex = isConvex;

                env->dualSolver->addHyperplane(hyperplane);
                numHyperplaneAdded++;
            }
            catch(std::exception& e)
            {
                env->output->outputWarning(
                    "        Cannot find solution with root search for generating supporting objective hyperplane.");
                env->output->outputDebug(fmt::format("        {}", e.what()));
            }
        }

        env->timing->stopTimer("DualObjectiveRootSearch");
    }

    if(numHyperplaneAdded > 0)
    {
        env->output->outputDebug(fmt::format(
            "        Added {} separating hyperplanes for objective function to waiting list.", numHyperplaneAdded));
    }
    else
    {
        for(auto& SOLPT : sourcePoints)
        {
            Hyperplane hyperplane;
            hyperplane.isObjectiveHyperplane = true;
            hyperplane.sourceConstraintIndex = -1;
            hyperplane.generatedPoint = SOLPT.point;
            hyperplane.source = E_HyperplaneSource::ObjectiveRootsearch;
            hyperplane.isSourceConvex = isConvex;

            hyperplane.objectiveFunctionValue
                = env->reformulatedProblem->objectiveFunction->calculateValue(hyperplane.generatedPoint);

            env->dualSolver->addHyperplane(hyperplane);
        }

        env->output->outputDebug(
            fmt::format("        Added {} cutting planes for objective function to waiting list.", numHyperplaneAdded));
    }
}

std::string TaskSelectHyperplanePointsObjectiveFunction::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT