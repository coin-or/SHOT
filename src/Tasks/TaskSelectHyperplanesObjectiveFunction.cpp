/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectHyperplanesObjectiveFunction.h"

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

TaskSelectHyperplanesObjectiveFunction::TaskSelectHyperplanesObjectiveFunction(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
}

TaskSelectHyperplanesObjectiveFunction::~TaskSelectHyperplanesObjectiveFunction() = default;

void TaskSelectHyperplanesObjectiveFunction::run() { this->run(env->results->getPreviousIteration()->solutionPoints); }

void TaskSelectHyperplanesObjectiveFunction::run(std::vector<SolutionPoint> sourcePoints)
{
    if(sourcePoints.size() == 0)
        return;

    int numHyperplaneAdded = 0;

    // Add cutting plane if the dual has stagnated
    if(env->solutionStatistics.numberOfIterationsWithDualStagnation > 2
        && env->reformulatedProblem->properties.convexity == E_ProblemConvexity::Convex)
    {
        ObjectiveHyperplanePtr hyperplane = std::make_shared<ObjectiveHyperplane>();
        hyperplane->generatedPoint = sourcePoints[0].point;
        hyperplane->source = E_HyperplaneSource::ObjectiveRootsearch;
        hyperplane->objectiveFunctionValue
            = env->reformulatedProblem->objectiveFunction->calculateValue(sourcePoints[0].point);
        hyperplane->isGlobal = true;

        env->dualSolver->addHyperplane(hyperplane);
        numHyperplaneAdded++;

        env->output->outputDebug("         Adding objective cutting plane since the dual has stagnated.");
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

    for(auto& SOLPT : sourcePoints)
    {
        if(useRootsearch)
        {
            env->timing->startTimer("DualObjectiveRootSearch");

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

                ObjectiveHyperplanePtr hyperplane = std::make_shared<ObjectiveHyperplane>();
                hyperplane->source = E_HyperplaneSource::ObjectiveRootsearch;
                hyperplane->generatedPoint = SOLPT.point;
                hyperplane->objectiveFunctionValue = rootBound.second;
                hyperplane->isGlobal = isConvex;

                env->dualSolver->addHyperplane(hyperplane);
                numHyperplaneAdded++;

                env->timing->stopTimer("DualObjectiveRootSearch");
                continue;
            }
            catch(std::exception& e)
            {
                env->output->outputDebug("        Cannot find solution with root search for generating "
                                         "supporting objective hyperplane. Adding cutting plane instead.");
                env->output->outputDebug(fmt::format("        {}", e.what()));

                env->timing->stopTimer("DualObjectiveRootSearch");
            }
        }

        ObjectiveHyperplanePtr hyperplane = std::make_shared<ObjectiveHyperplane>();
        hyperplane->generatedPoint = SOLPT.point;
        hyperplane->source = E_HyperplaneSource::ObjectiveCuttingPlane;
        hyperplane->isGlobal = isConvex;

        hyperplane->objectiveFunctionValue
            = env->reformulatedProblem->objectiveFunction->calculateValue(hyperplane->generatedPoint);

        env->dualSolver->addHyperplane(hyperplane);
        numHyperplaneAdded++;
    }

    if(numHyperplaneAdded > 0)
    {
        env->output->outputDebug(
            fmt::format("        Added {} linearizations for objective function to waiting list.", numHyperplaneAdded));
    }
    else
    {
        env->output->outputDebug(fmt::format("        No linearizations for objective function added."));
    }
}

std::string TaskSelectHyperplanesObjectiveFunction::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT