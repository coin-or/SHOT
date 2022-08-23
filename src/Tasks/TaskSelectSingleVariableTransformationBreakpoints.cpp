/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSelectSingleVariableTransformationBreakpoints.h"

#include "../Model/Problem.h"
#include "../DualSolver.h"
#include "../Enums.h"
#include "../Environment.h"
#include "../Iteration.h"
#include "../Results.h"
#include "../Timing.h"

namespace SHOT
{

TaskSelectSingleVariableTransformationBreakpoints::TaskSelectSingleVariableTransformationBreakpoints(
    EnvironmentPtr envPtr)
    : TaskBase(envPtr)
{
    // Only initialize once
    if(env->dualSolver->singleVariableTransformations.size() > 0)
        return;

    env->timing->startTimer("ProblemReformulation");

    for(auto& T : env->reformulatedProblem->singleVariableTransformations)
    {
        /*if(T->originalVariable->properties.type == E_VariableType::Integer)
        {
            for(int PT = T->originalVariable->lowerBound; PT <= T->originalVariable->upperBound; PT++)
            {
                T->addBreakpoint(PT, 0, E_BreakpointSource::Initial);
            }
        }
        else
        {*/
        T->addBreakpoint(T->originalVariable->lowerBound, 0, E_BreakpointSource::Initial);
        T->addBreakpoint(T->originalVariable->upperBound, 0, E_BreakpointSource::Initial);
        //}
        env->dualSolver->singleVariableTransformations.push_back(T);
    }

    env->dualSolver->itersWithoutAddedBreakpoints = 0;

    env->timing->stopTimer("ProblemReformulation");
}

TaskSelectSingleVariableTransformationBreakpoints::~TaskSelectSingleVariableTransformationBreakpoints() = default;

void TaskSelectSingleVariableTransformationBreakpoints::run()
{
    if(env->results->getNumberOfIterations() < 1)
        return;

    auto prevIter = env->results->getPreviousIteration();

    if(prevIter->solutionPoints.size() == 0)
        return;

    bool added = false;

    if(env->dualSolver->itersWithoutAddedBreakpoints > 20 || env->dualSolver->itersWithoutAddedBreakpoints > 2)
    {

        for(auto T : env->dualSolver->singleVariableTransformations)
        {
            /*if(T->originalVariable->properties.type == E_VariableType::Binary)
            {
                // No need to add breakpoint, all breakpoints already added
            }
            else if(T->originalVariable->properties.type == E_VariableType::Integer)
            {
                added = T->addBreakpoint(round(prevIter->solutionPoints.at(0).point.at(T->originalVariable->index)),
                            prevIter->iterationNumber, E_BreakpointSource::DualSolution)
                    || added;
            }
            else
            {*/
            added = T->addBreakpoint(prevIter->solutionPoints.at(0).point.at(T->originalVariable->index),
                        prevIter->iterationNumber, E_BreakpointSource::DualSolution)
                || added;
            //}
        }

        if(!added)
        {
            for(auto T : env->dualSolver->singleVariableTransformations)
            {
                T->addMidIntervalBreakpoint(prevIter->solutionPoints.at(0).point.at(T->originalVariable->index),
                    prevIter->iterationNumber, E_BreakpointSource::DualSolutionMidpoint);
            }
        }
    }

    std::cout << "iters without breakpoint updates " << env->dualSolver->itersWithoutAddedBreakpoints << std::endl;

    if(!added)
        env->dualSolver->itersWithoutAddedBreakpoints++;
    else
        env->dualSolver->itersWithoutAddedBreakpoints = 0;
}

std::string TaskSelectSingleVariableTransformationBreakpoints::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

} // namespace SHOT