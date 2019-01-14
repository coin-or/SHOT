/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSolveFixedDualProblem.h"

namespace SHOT
{

TaskSolveFixedDualProblem::TaskSolveFixedDualProblem(EnvironmentPtr envPtr)
    : TaskBase(envPtr)
{
    env->timing->startTimer("DualProblemsIntegerFixed");

    for(auto& V : env->problem->binaryVariables)
    {
        discreteVariableIndexes.push_back(V->index);
    }

    for(auto& V : env->problem->integerVariables)
    {
        discreteVariableIndexes.push_back(V->index);
    }

    env->timing->stopTimer("DualProblemsIntegerFixed");
}

TaskSolveFixedDualProblem::~TaskSolveFixedDualProblem() {}

void TaskSolveFixedDualProblem::run()
{
    env->timing->startTimer("DualProblemsIntegerFixed");
    auto currIter = env->results->getCurrentIteration(); // The one not solved yet

    if(currIter->MIPSolutionLimitUpdated)
    {
        env->timing->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    if(currIter->iterationNumber < 5)
    {
        env->timing->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    if(currIter->maxDeviation <= env->settings->getDoubleSetting("FixedInteger.ConstraintTolerance", "Dual"))
    {
        env->timing->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    auto prevIter = env->results->getPreviousIteration();

    if(prevIter->iterationNumber < 4)
    {
        env->timing->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    auto prevIter2 = env->results->iterations.at(prevIter->iterationNumber - 2);
    auto prevIter3 = env->results->iterations.at(prevIter->iterationNumber - 3);

    if(!prevIter->isMIP() && !prevIter2->isMIP() && !prevIter3->isMIP())
    {
        env->timing->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    if(currIter->numHyperplanesAdded == 0)
    {
        env->timing->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    auto currSolPt = prevIter->solutionPoints.at(0).point;

    bool isDifferent1 = UtilityFunctions::isDifferentSelectedElements(
        currSolPt, prevIter2->solutionPoints.at(0).point, discreteVariableIndexes);

    bool isDifferent2 = UtilityFunctions::isDifferentSelectedElements(
        currSolPt, prevIter3->solutionPoints.at(0).point, discreteVariableIndexes);

    if(isDifferent1 || isDifferent2)
    {
        env->timing->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    VectorDouble fixValues(discreteVariableIndexes.size());

    for(int i = 0; i < discreteVariableIndexes.size(); i++)
    {
        fixValues.at(i) = currSolPt.at(discreteVariableIndexes.at(i));
    }

    env->dualSolver->MIPSolver->fixVariables(discreteVariableIndexes, fixValues);

    bool isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;

    double prevObjVal = SHOT_DBL_MAX;

    int iterLastObjUpdate = 0;
    int maxIter = env->settings->getIntSetting("FixedInteger.MaxIterations", "Dual");
    double objTol = env->settings->getDoubleSetting("FixedInteger.ObjectiveTolerance", "Dual");
    double constrTol = env->settings->getDoubleSetting("FixedInteger.ConstraintTolerance", "Dual");

    bool isMIQP = env->reformulatedProblem->properties.isMIQPProblem;
    bool isMIQCP = env->reformulatedProblem->properties.isMIQCQPProblem;
    bool isDiscrete = false;

    for(int k = 0; k < maxIter; k++)
    {
        std::stringstream tmpType;

        if(isMIQCP)
        {
            tmpType << "QCP-FIX";
            env->solutionStatistics.numberOfProblemsQCQP++;
        }
        else if(isMIQP)
        {
            tmpType << "QP-FIX";
            env->solutionStatistics.numberOfProblemsQP++;
        }
        else
        {
            tmpType << "LP-FIX";
            env->solutionStatistics.numberOfProblemsLP++;
        }

        totalIters++;
        auto solStatus = env->dualSolver->MIPSolver->solveProblem();

        if(solStatus != E_ProblemSolutionStatus::Optimal)
        {

            tmpType << "-I";

            env->report->outputIterationDetail(totalIters, tmpType.str(), env->timing->getElapsedTime("Total"), 1,
                currIter->totNumHyperplanes, env->results->getDualBound(), env->results->getPrimalBound(),
                env->results->getAbsoluteObjectiveGap(), env->results->getRelativeObjectiveGap(), NAN, NAN, NAN,
                E_IterationLineType::DualIntegerFixed);

            break;
        }
        else
        {
            auto varSol = env->dualSolver->MIPSolver->getVariableSolution(0);
            auto objVal = env->dualSolver->MIPSolver->getObjectiveValue(0);

            auto mostDevConstraint = env->reformulatedProblem->getMaxNumericConstraintValue(
                varSol, env->reformulatedProblem->nonlinearConstraints);

            VectorDouble externalPoint = varSol;

            if(env->dualSolver->MIPSolver->interiorPts.size() > 0)
            {
                VectorDouble internalPoint = env->dualSolver->MIPSolver->interiorPts.at(0)->point;

                try
                {
                    auto xNewc = env->rootsearchMethod->findZero(internalPoint, externalPoint,
                        env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                        env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
                        env->settings->getDoubleSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver"),
                        env->reformulatedProblem->nonlinearConstraints);

                    env->timing->stopTimer("DualCutGenerationRootSearch");
                    internalPoint = xNewc.first;
                    externalPoint = xNewc.second;
                }
                catch(std::exception& e)
                {

                    env->output->outputWarning(
                        "     Cannot find solution with linesearch for fixed LP, using solution point instead:");
                    env->output->outputWarning(e.what());
                }
            }

            auto errorExternal = env->reformulatedProblem->getMaxNumericConstraintValue(
                externalPoint, env->reformulatedProblem->nonlinearConstraints);

            Hyperplane hyperplane;
            hyperplane.sourceConstraint = errorExternal.constraint;
            hyperplane.sourceConstraintIndex = errorExternal.constraint->index;
            hyperplane.generatedPoint = externalPoint;
            hyperplane.source = E_HyperplaneSource::LPFixedIntegers;

            env->dualSolver->MIPSolver->createHyperplane(hyperplane);

            bool hasSolution = true;

            if(varSol.size() == 0)
                hasSolution = false;

            if(solStatus == E_ProblemSolutionStatus::Error)
            {
                tmpType << "-E";
                hasSolution = false;
            }
            else if(solStatus == E_ProblemSolutionStatus::Feasible)
            {
                tmpType << "-F";
            }
            else if(solStatus == E_ProblemSolutionStatus::Infeasible)
            {
                tmpType << "-I";
                hasSolution = false;
            }
            else if(solStatus == E_ProblemSolutionStatus::IterationLimit)
            {
                tmpType << "-I";
            }
            else if(solStatus == E_ProblemSolutionStatus::Optimal)
            {
                tmpType << "-O";
            }
            else if(solStatus == E_ProblemSolutionStatus::TimeLimit)
            {
                tmpType << "-TL";
                hasSolution = false;
            }
            else if(solStatus == E_ProblemSolutionStatus::Unbounded)
            {
                tmpType << "-U";
                hasSolution = false;
            }

            env->report->outputIterationDetail(totalIters, tmpType.str(), env->timing->getElapsedTime("Total"), 1,
                currIter->totNumHyperplanes, env->results->getDualBound(), env->results->getPrimalBound(),
                env->results->getAbsoluteObjectiveGap(), env->results->getRelativeObjectiveGap(), objVal,
                mostDevConstraint.constraint->index, mostDevConstraint.normalizedValue,
                E_IterationLineType::DualIntegerFixed);

            if(mostDevConstraint.normalizedValue <= constrTol || k - iterLastObjUpdate > 10
                || objVal > env->results->getPrimalBound())
            {
                break;
            }

            if(abs(prevObjVal - objVal) > prevObjVal * objTol)
            {
                iterLastObjUpdate = k;
                prevObjVal = objVal;
            }
        }
    }

    env->dualSolver->MIPSolver->activateDiscreteVariables(true);

    env->dualSolver->MIPSolver->unfixVariables();

    env->timing->stopTimer("DualProblemsIntegerFixed");
    return;
}

std::string TaskSolveFixedDualProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT