/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskPrintIterationReport.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Output.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"

#include "../Model/Problem.h"

namespace SHOT
{

TaskPrintIterationReport::TaskPrintIterationReport(EnvironmentPtr envPtr) : TaskBase(envPtr) { lastNumHyperplane = 0; }

TaskPrintIterationReport::~TaskPrintIterationReport() = default;

void TaskPrintIterationReport::run()
{
    auto currIter = env->results->getCurrentIteration();

    std::stringstream tmpType;

    bool hasSolution = true;

    auto dualProblemClass = env->dualSolver->MIPSolver->getProblemClass();

    switch(dualProblemClass)
    {
    case E_DualProblemClass::LP:
        tmpType << "LP";
        break;
    case E_DualProblemClass::QP:
        tmpType << "QP";
        break;
    case E_DualProblemClass::QCQP:
        tmpType << "QCQP";
        break;
    case E_DualProblemClass::MIP:
        tmpType << "MIP";
        break;
    case E_DualProblemClass::MIQP:
        tmpType << "MIQP";
        break;
    case E_DualProblemClass::MIQCQP:
        tmpType << "MIQPCP";
        break;
    default:
        break;
    }

    if(currIter->solutionPoints.size() == 0)
        hasSolution = false;

    if(currIter->solutionStatus == E_ProblemSolutionStatus::Error)
    {
        tmpType << "-E";
        hasSolution = false;
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Feasible)
    {
        tmpType << "-F";
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible)
    {
        tmpType << "-I";
        hasSolution = false;
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::IterationLimit)
    {
        tmpType << "-IL";
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
    {
        tmpType << "-O";
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
    {
        tmpType << "-SL";
        if(currIter->usedMIPSolutionLimit > 100)
        {
            tmpType << " ";
        }
        else
        {
            tmpType << std::to_string(currIter->usedMIPSolutionLimit);
        }
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit)
    {
        tmpType << "-TL";
        hasSolution = false;
    }
    else if(currIter->solutionStatus == E_ProblemSolutionStatus::Unbounded)
    {
        tmpType << "-U";
        hasSolution = false;
    }
    else
    {
        tmpType << "";
        hasSolution = false;
    }

    env->report->outputIterationDetail(currIter->iterationNumber, tmpType.str(), env->timing->getElapsedTime("Total"),
        currIter->numHyperplanesAdded, currIter->totNumHyperplanes, env->results->getCurrentDualBound(),
        env->results->getPrimalBound(), env->results->getAbsoluteCurrentObjectiveGap(),
        env->results->getRelativeCurrentObjectiveGap(), currIter->objectiveValue, currIter->maxDeviationConstraint,
        currIter->maxDeviation, E_IterationLineType::DualSolution);
}

std::string TaskPrintIterationReport::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT