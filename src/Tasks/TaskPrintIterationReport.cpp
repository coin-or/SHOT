/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskPrintIterationReport.h"

TaskPrintIterationReport::TaskPrintIterationReport(EnvironmentPtr envPtr): TaskBase(envPtr)
{
    lastNumHyperplane = 0;
}

TaskPrintIterationReport::~TaskPrintIterationReport()
{
}

void TaskPrintIterationReport::run()
{
    auto currIter = env->process->getCurrentIteration();

    std::stringstream tmpType;

    bool hasSolution = true;

    bool isMIQP = (env->process->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic);
    bool isMIQCP = (env->process->originalProblem->getQuadraticConstraintIndexes().size() > 0);
    bool isDiscrete = (currIter->type == E_IterationProblemType::MIP) && env->process->originalProblem->isProblemDiscrete();

    if (isMIQCP && isDiscrete)
        tmpType << "MIQCP";
    else if (isMIQCP)
        tmpType << "QCP";
    else if (isMIQP && isDiscrete)
        tmpType << "MIQP";
    else if (isMIQP)
        tmpType << "QP";
    else if (isDiscrete)
        tmpType << "MIP";
    else
        tmpType << "LP";

    if (currIter->solutionPoints.size() == 0)
        hasSolution = false;

    if (currIter->solutionStatus == E_ProblemSolutionStatus::Error)
    {
        tmpType << "-E";
        hasSolution = false;
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::Feasible)
    {
        tmpType << "-F";
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible)
    {
        tmpType << "-I";
        hasSolution = false;
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::IterationLimit)
    {
        tmpType << "-IL";
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
    {
        tmpType << "-O";
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
    {
        tmpType << "-SL";
        if (currIter->usedMIPSolutionLimit > 100)
        {
            tmpType << " ";
        }
        else
        {
            tmpType << std::to_string(currIter->usedMIPSolutionLimit);
        }
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit)
    {
        tmpType << "-TL";
        hasSolution = false;
    }
    else if (currIter->solutionStatus == E_ProblemSolutionStatus::Unbounded)
    {
        tmpType << "-U";
        hasSolution = false;
    }

    env->output->outputIterationDetail(currIter->iterationNumber,
                                                tmpType.str(),
                                                env->process->getElapsedTime("Total"),
                                                currIter->numHyperplanesAdded,
                                                currIter->totNumHyperplanes,
                                                env->process->getDualBound(),
                                                env->process->getPrimalBound(),
                                                env->process->getAbsoluteObjectiveGap(),
                                                env->process->getRelativeObjectiveGap(),
                                                currIter->objectiveValue,
                                                currIter->maxDeviationConstraint,
                                                currIter->maxDeviation,
                                                E_IterationLineType::DualSolution);
}

std::string TaskPrintIterationReport::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
