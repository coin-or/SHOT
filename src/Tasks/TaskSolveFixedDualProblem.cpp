/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSolveFixedDualProblem.h"

TaskSolveFixedDualProblem::TaskSolveFixedDualProblem(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("DualProblemsIntegerFixed");

    discreteVariableIndexes = env->model->originalProblem->getDiscreteVariableIndices();

    env->process->stopTimer("DualProblemsIntegerFixed");
}

TaskSolveFixedDualProblem::~TaskSolveFixedDualProblem()
{
}

void TaskSolveFixedDualProblem::run()
{
    env->process->startTimer("DualProblemsIntegerFixed");
    auto currIter = env->process->getCurrentIteration(); //The one not solved yet

    if (currIter->MIPSolutionLimitUpdated)
    {
        env->process->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    if (currIter->iterationNumber < 5)
    {
        env->process->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    if (currIter->maxDeviation <= env->settings->getDoubleSetting("FixedInteger.ConstraintTolerance", "Dual"))
    {
        env->process->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    auto prevIter = env->process->getPreviousIteration();

    if (prevIter->iterationNumber < 4)
    {
        env->process->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    auto prevIter2 = &env->process->iterations.at(prevIter->iterationNumber - 2);
    auto prevIter3 = &env->process->iterations.at(prevIter->iterationNumber - 3);

    if (!prevIter->isMIP() && !prevIter2->isMIP() && !prevIter3->isMIP())
    {
        env->process->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    if (currIter->numHyperplanesAdded == 0)
    {
        env->process->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    auto discreteIdxs = env->model->originalProblem->getDiscreteVariableIndices();

    auto currSolPt = prevIter->solutionPoints.at(0).point;

    bool isDifferent1 = UtilityFunctions::isDifferentSelectedElements(currSolPt, prevIter2->solutionPoints.at(0).point,
                                                                      discreteIdxs);

    bool isDifferent2 = UtilityFunctions::isDifferentSelectedElements(currSolPt, prevIter3->solutionPoints.at(0).point,
                                                                      discreteIdxs);

    if (isDifferent1 || isDifferent2)
    {
        env->process->stopTimer("DualProblemsIntegerFixed");
        return;
    }

    DoubleVector fixValues(discreteVariableIndexes.size());

    for (int i = 0; i < discreteVariableIndexes.size(); i++)
    {
        fixValues.at(i) = currSolPt.at(discreteVariableIndexes.at(i));
    }

    env->dualSolver->fixVariables(discreteVariableIndexes, fixValues);

    int numVar = env->model->originalProblem->getNumberOfVariables();

    bool isMinimization = env->model->originalProblem->isTypeOfObjectiveMinimize();

    double prevObjVal = COIN_DBL_MAX;

    int iterLastObjUpdate = 0;
    int maxIter = env->settings->getIntSetting("FixedInteger.MaxIterations", "Dual");
    double objTol = env->settings->getDoubleSetting("FixedInteger.ObjectiveTolerance", "Dual");
    double constrTol = env->settings->getDoubleSetting("FixedInteger.ConstraintTolerance", "Dual");

    bool isMIQP = (env->model->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic);
    bool isMIQCP = (env->model->originalProblem->getQuadraticConstraintIndexes().size() > 0);
    bool isDiscrete = false;

    for (int k = 0; k < maxIter; k++)
    {
        std::stringstream tmpType;

        if (isMIQCP)
        {
            tmpType << "QCP-FIX";
            env->solutionStatistics.numberOfProblemsQCQP++;
        }
        else if (isMIQP)
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
        auto solStatus = env->dualSolver->solveProblem();

        if (solStatus != E_ProblemSolutionStatus::Optimal)
        {

            tmpType << "-I";

            env->report->outputIterationDetail(totalIters,
                                               tmpType.str(),
                                               env->process->getElapsedTime("Total"),
                                               1,
                                               currIter->totNumHyperplanes,
                                               env->process->getDualBound(),
                                               env->process->getPrimalBound(),
                                               env->process->getAbsoluteObjectiveGap(),
                                               env->process->getRelativeObjectiveGap(),
                                               NAN,
                                               NAN,
                                               NAN,
                                               E_IterationLineType::DualIntegerFixed);

            break;
        }
        else
        {
            auto varSol = env->dualSolver->getVariableSolution(0);
            auto objVal = env->dualSolver->getObjectiveValue(0);

            auto mostDevConstr = env->model->originalProblem->getMostDeviatingConstraint(varSol);

            DoubleVector externalPoint = varSol;
            IndexValuePair errorExternal;

            if (env->process->interiorPts.size() > 0)
            {
                DoubleVector internalPoint = env->process->interiorPts.at(0)->point;

                auto tmpMostDevConstr2 = env->model->originalProblem->getMostDeviatingConstraint(internalPoint);

                try
                {
                    auto xNewc = env->process->linesearchMethod->findZero(internalPoint, externalPoint,
                                                                          env->settings->getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                          env->settings->getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
                                                                          env->settings->getDoubleSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver"));

                    env->process->stopTimer("DualCutGenerationRootSearch");
                    internalPoint = xNewc.first;
                    externalPoint = xNewc.second;
                }
                catch (std::exception &e)
                {

                    env->output->outputWarning(
                        "     Cannot find solution with linesearch for fixed LP, using solution point instead:");
                    env->output->outputWarning(e.what());
                }
            }

            errorExternal = env->model->originalProblem->getMostDeviatingConstraint(externalPoint);

            Hyperplane hyperplane;
            hyperplane.sourceConstraintIndex = errorExternal.idx;
            hyperplane.generatedPoint = externalPoint;
            hyperplane.source = E_HyperplaneSource::LPFixedIntegers;

            env->dualSolver->createHyperplane(hyperplane);

            bool hasSolution = true;

            if (varSol.size() == 0)
                hasSolution = false;

            if (solStatus == E_ProblemSolutionStatus::Error)
            {
                tmpType << "-E";
                hasSolution = false;
            }
            else if (solStatus == E_ProblemSolutionStatus::Feasible)
            {
                tmpType << "-F";
            }
            else if (solStatus == E_ProblemSolutionStatus::Infeasible)
            {
                tmpType << "-I";
                hasSolution = false;
            }
            else if (solStatus == E_ProblemSolutionStatus::IterationLimit)
            {
                tmpType << "-I";
            }
            else if (solStatus == E_ProblemSolutionStatus::Optimal)
            {
                tmpType << "-O";
            }
            else if (solStatus == E_ProblemSolutionStatus::TimeLimit)
            {
                tmpType << "-TL";
                hasSolution = false;
            }
            else if (solStatus == E_ProblemSolutionStatus::Unbounded)
            {
                tmpType << "-U";
                hasSolution = false;
            }

            env->report->outputIterationDetail(totalIters,
                                               tmpType.str(),
                                               env->process->getElapsedTime("Total"),
                                               1,
                                               currIter->totNumHyperplanes,
                                               env->process->getDualBound(),
                                               env->process->getPrimalBound(),
                                               env->process->getAbsoluteObjectiveGap(),
                                               env->process->getRelativeObjectiveGap(),
                                               objVal,
                                               mostDevConstr.idx,
                                               mostDevConstr.value,
                                               E_IterationLineType::DualIntegerFixed);

            if (mostDevConstr.value <= constrTol || k - iterLastObjUpdate > 10 || objVal > env->process->getPrimalBound())
            {
                break;
            }

            if (abs(prevObjVal - objVal) > prevObjVal * objTol)
            {
                iterLastObjUpdate = k;
                prevObjVal = objVal;
            }
        }
    }

    env->dualSolver->activateDiscreteVariables(true);

    env->dualSolver->unfixVariables();

    env->process->stopTimer("DualProblemsIntegerFixed");
    return;
}

std::string TaskSolveFixedDualProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
