/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSolveFixedDualProblem.h"

TaskSolveFixedDualProblem::TaskSolveFixedDualProblem(IMIPSolver *MIPSolver)
{
    ProcessInfo::getInstance().startTimer("DualProblemsIntegerFixed");

    this->MIPSolver = MIPSolver;

    discreteVariableIndexes = ProcessInfo::getInstance().originalProblem->getDiscreteVariableIndices();

    ProcessInfo::getInstance().stopTimer("DualProblemsIntegerFixed");
}

TaskSolveFixedDualProblem::~TaskSolveFixedDualProblem()
{
}

void TaskSolveFixedDualProblem::run()
{
    ProcessInfo::getInstance().startTimer("DualProblemsIntegerFixed");
    auto currIter = ProcessInfo::getInstance().getCurrentIteration(); //The one not solved yet

    if (currIter->MIPSolutionLimitUpdated)
    {
        ProcessInfo::getInstance().stopTimer("DualProblemsIntegerFixed");
        return;
    }

    if (currIter->iterationNumber < 5)
    {
        ProcessInfo::getInstance().stopTimer("DualProblemsIntegerFixed");
        return;
    }

    if (currIter->maxDeviation <= Settings::getInstance().getDoubleSetting("FixedInteger.ConstraintTolerance", "Dual"))
    {
        ProcessInfo::getInstance().stopTimer("DualProblemsIntegerFixed");
        return;
    }

    auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

    if (prevIter->iterationNumber < 4)
    {
        ProcessInfo::getInstance().stopTimer("DualProblemsIntegerFixed");
        return;
    }

    auto prevIter2 = &ProcessInfo::getInstance().iterations.at(prevIter->iterationNumber - 2);
    auto prevIter3 = &ProcessInfo::getInstance().iterations.at(prevIter->iterationNumber - 3);

    if (!prevIter->isMIP() && !prevIter2->isMIP() && !prevIter3->isMIP())
    {
        ProcessInfo::getInstance().stopTimer("DualProblemsIntegerFixed");
        return;
    }

    if (currIter->numHyperplanesAdded == 0)
    {
        ProcessInfo::getInstance().stopTimer("DualProblemsIntegerFixed");
        return;
    }

    auto discreteIdxs = ProcessInfo::getInstance().originalProblem->getDiscreteVariableIndices();

    auto currSolPt = prevIter->solutionPoints.at(0).point;

    bool isDifferent1 = UtilityFunctions::isDifferentSelectedElements(currSolPt, prevIter2->solutionPoints.at(0).point,
                                                                      discreteIdxs);

    bool isDifferent2 = UtilityFunctions::isDifferentSelectedElements(currSolPt, prevIter3->solutionPoints.at(0).point,
                                                                      discreteIdxs);

    if (isDifferent1 || isDifferent2)
    {
        ProcessInfo::getInstance().stopTimer("DualProblemsIntegerFixed");
        return;
    }

    std::vector<double> fixValues(discreteVariableIndexes.size());

    for (int i = 0; i < discreteVariableIndexes.size(); i++)
    {
        fixValues.at(i) = currSolPt.at(discreteVariableIndexes.at(i));
    }

    MIPSolver->fixVariables(discreteVariableIndexes, fixValues);

    int numVar = ProcessInfo::getInstance().originalProblem->getNumberOfVariables();

    bool isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

    double prevObjVal = COIN_DBL_MAX;

    int iterLastObjUpdate = 0;
    int maxIter = Settings::getInstance().getIntSetting("FixedInteger.MaxIterations", "Dual");
    double objTol = Settings::getInstance().getDoubleSetting("FixedInteger.ObjectiveTolerance", "Dual");
    double constrTol = Settings::getInstance().getDoubleSetting("FixedInteger.ConstraintTolerance", "Dual");

    bool isMIQP = (ProcessInfo::getInstance().originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic);
    bool isMIQCP = (ProcessInfo::getInstance().originalProblem->getQuadraticConstraintIndexes().size() > 0);
    bool isDiscrete = false;

    for (int k = 0; k < maxIter; k++)
    {
        std::stringstream tmpType;

        if (isMIQCP)
        {
            tmpType << "QCP-FIX";
            ProcessInfo::getInstance().solutionStatistics.numberOfProblemsQCQP++;
        }
        else if (isMIQP)
        {
            tmpType << "QP-FIX";
            ProcessInfo::getInstance().solutionStatistics.numberOfProblemsQP++;
        }
        else
        {
            tmpType << "LP-FIX";
            ProcessInfo::getInstance().solutionStatistics.numberOfProblemsLP++;
        }

        totalIters++;
        auto solStatus = MIPSolver->solveProblem();

        if (solStatus != E_ProblemSolutionStatus::Optimal)
        {

            tmpType << "-I";

            Output::getInstance().outputIterationDetail(totalIters,
                                                        tmpType.str(),
                                                        ProcessInfo::getInstance().getElapsedTime("Total"),
                                                        1,
                                                        currIter->totNumHyperplanes,
                                                        ProcessInfo::getInstance().getDualBound(),
                                                        ProcessInfo::getInstance().getPrimalBound(),
                                                        ProcessInfo::getInstance().getAbsoluteObjectiveGap(),
                                                        ProcessInfo::getInstance().getRelativeObjectiveGap(),
                                                        NAN,
                                                        NAN,
                                                        NAN,
                                                        E_IterationLineType::DualIntegerFixed);

            break;
        }
        else
        {
            auto varSol = MIPSolver->getVariableSolution(0);
            auto objVal = MIPSolver->getObjectiveValue(0);

            auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(varSol);

            std::vector<double> externalPoint = varSol;
            IndexValuePair errorExternal;

            if (ProcessInfo::getInstance().interiorPts.size() > 0)
            {
                std::vector<double> internalPoint = ProcessInfo::getInstance().interiorPts.at(0)->point;

                auto tmpMostDevConstr2 = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(internalPoint);

                try
                {
                    auto xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(internalPoint, externalPoint,
                                                                                       Settings::getInstance().getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
                                                                                       Settings::getInstance().getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
                                                                                       Settings::getInstance().getDoubleSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver"));

                    ProcessInfo::getInstance().stopTimer("DualCutGenerationRootSearch");
                    internalPoint = xNewc.first;
                    externalPoint = xNewc.second;
                }
                catch (std::exception &e)
                {

                    Output::getInstance().outputWarning(
                        "     Cannot find solution with linesearch for fixed LP, using solution point instead:");
                    Output::getInstance().outputWarning(e.what());
                }
            }

            errorExternal = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(externalPoint);

            Hyperplane hyperplane;
            hyperplane.sourceConstraintIndex = errorExternal.idx;
            hyperplane.generatedPoint = externalPoint;
            hyperplane.source = E_HyperplaneSource::LPFixedIntegers;

            MIPSolver->createHyperplane(hyperplane);

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

            Output::getInstance().outputIterationDetail(totalIters,
                                                        tmpType.str(),
                                                        ProcessInfo::getInstance().getElapsedTime("Total"),
                                                        1,
                                                        currIter->totNumHyperplanes,
                                                        ProcessInfo::getInstance().getDualBound(),
                                                        ProcessInfo::getInstance().getPrimalBound(),
                                                        ProcessInfo::getInstance().getAbsoluteObjectiveGap(),
                                                        ProcessInfo::getInstance().getRelativeObjectiveGap(),
                                                        objVal,
                                                        mostDevConstr.idx,
                                                        mostDevConstr.value,
                                                        E_IterationLineType::DualIntegerFixed);

            if (mostDevConstr.value <= constrTol || k - iterLastObjUpdate > 10 || objVal > ProcessInfo::getInstance().getPrimalBound())
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

    MIPSolver->activateDiscreteVariables(true);

    MIPSolver->unfixVariables();

    ProcessInfo::getInstance().stopTimer("DualProblemsIntegerFixed");
    return;
}

std::string TaskSolveFixedDualProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
