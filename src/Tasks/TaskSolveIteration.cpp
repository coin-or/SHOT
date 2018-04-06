/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSolveIteration.h"

TaskSolveIteration::TaskSolveIteration(IMIPSolver *MIPSolver)
{
    this->MIPSolver = MIPSolver;
}

TaskSolveIteration::~TaskSolveIteration()
{
}

void TaskSolveIteration::run()
{
    ProcessInfo::getInstance().startTimer("DualStrategy");
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    bool isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

    // Sets the iteration time limit
    auto timeLim = Settings::getInstance().getDoubleSetting("TimeLimit", "Termination") - ProcessInfo::getInstance().getElapsedTime("Total");
    MIPSolver->setTimeLimit(timeLim);

    if (ProcessInfo::getInstance().primalSolutions.size() > 0)
    {
        if (isMinimization)
        {
            MIPSolver->setCutOff(
                ProcessInfo::getInstance().getPrimalBound() + Settings::getInstance().getDoubleSetting("MIP.CutOffTolerance", "Dual"));
        }
        else
        {
            MIPSolver->setCutOff(
                ProcessInfo::getInstance().getPrimalBound() - Settings::getInstance().getDoubleSetting("MIP.CutOffTolerance", "Dual"));
        }
    }

    if (Settings::getInstance().getBoolSetting("MIP.UpdateObjectiveBounds", "Dual") && !currIter->MIPSolutionLimitUpdated)
    {
        MIPSolver->updateNonlinearObjectiveFromPrimalDualBounds();
    }

    if (MIPSolver->getDiscreteVariableStatus() && ProcessInfo::getInstance().primalSolutions.size() > 0)
    {
        MIPSolver->addMIPStart(ProcessInfo::getInstance().primalSolution);
    }

    if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
    {
        stringstream ss;
        ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
        ss << "/lp";
        ss << currIter->iterationNumber - 1;
        ss << ".lp";
        MIPSolver->writeProblemToFile(ss.str());
    }

    Output::getInstance().outputInfo("Solving MIP problem.");
    auto solStatus = MIPSolver->solveProblem();
    Output::getInstance().outputInfo("MIP problem solved.");

    // Must update the pointer to the current iteration if we use the lazy strategy since new iterations have been created when solving
    if (static_cast<ES_TreeStrategy>(Settings::getInstance().getIntSetting("TreeStrategy", "Dual")) == ES_TreeStrategy::SingleTree)
    {
        currIter = ProcessInfo::getInstance().getCurrentIteration();
    }

    if (solStatus == E_ProblemSolutionStatus::Infeasible ||
        solStatus == E_ProblemSolutionStatus::Error ||
        solStatus == E_ProblemSolutionStatus::Abort ||
        solStatus == E_ProblemSolutionStatus::CutOff ||
        solStatus == E_ProblemSolutionStatus::Numeric ||
        solStatus == E_ProblemSolutionStatus::Unbounded)
    {
        currIter->solutionStatus = solStatus;
    }
    else
    {
        currIter->solutionStatus = solStatus;

        auto sols = MIPSolver->getAllVariableSolutions();
        currIter->solutionPoints = sols;

        if (sols.size() > 0)
        {
            if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
            {
                stringstream ss;
                ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
                ss << "/lpsolpt";
                ss << currIter->iterationNumber - 1;
                ss << ".txt";
                UtilityFunctions::saveVariablePointVectorToFile(sols.at(0).point,
                                                                ProcessInfo::getInstance().originalProblem->getVariableNames(), ss.str());
            }

            currIter->objectiveValue = MIPSolver->getObjectiveValue();

            if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
            {
                std::vector<double> tmpObjValue;
                std::vector<std::string> tmpObjName;

                tmpObjValue.push_back(MIPSolver->getObjectiveValue());
                tmpObjName.push_back("objective");

                stringstream ss;
                ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
                ss << "/lpobjsol";
                ss << currIter->iterationNumber - 1;
                ss << ".txt";
                UtilityFunctions::saveVariablePointVectorToFile(tmpObjValue, tmpObjName, ss.str());
            }

            auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(sols.at(0).point);

            currIter->maxDeviationConstraint = mostDevConstr.idx;
            currIter->maxDeviation = mostDevConstr.value;

            if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
            {
                std::vector<double> tmpMostDevValue;
                std::vector<std::string> tmpConstrIndex;

                tmpMostDevValue.push_back(mostDevConstr.value);
                tmpConstrIndex.push_back(std::to_string(mostDevConstr.idx));

                stringstream ss;
                ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
                ss << "/lpmostdevm";
                ss << currIter->iterationNumber - 1;
                ss << ".txt";
                UtilityFunctions::saveVariablePointVectorToFile(tmpMostDevValue, tmpConstrIndex, ss.str());
            }

            double tmpDualObjBound = MIPSolver->getDualObjectiveValue();
            if (currIter->isMIP())
            {
                DualSolution sol =
                    {sols.at(0).point, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound,
                     currIter->iterationNumber};
                ProcessInfo::getInstance().addDualSolutionCandidate(sol);

                if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
                {
                    DualSolution sol =
                        {sols.at(0).point, E_DualSolutionSource::MIPSolutionOptimal, currIter->objectiveValue,
                         currIter->iterationNumber};
                    ProcessInfo::getInstance().addDualSolutionCandidate(sol);
                }
            }
            else
            {
                DualSolution sol =
                    {sols.at(0).point, E_DualSolutionSource::LPSolution, tmpDualObjBound,
                     currIter->iterationNumber};
                ProcessInfo::getInstance().addDualSolutionCandidate(sol);
            }
        }
    }

    currIter->usedMIPSolutionLimit = MIPSolver->getSolutionLimit();

    // Update solution stats
    if (currIter->type == E_IterationProblemType::MIP && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
    {
        if (ProcessInfo::getInstance().originalProblem->isConstraintQuadratic(-1))
        {
            ProcessInfo::getInstance().solutionStatistics.numberOfProblemsOptimalMIQP++;
        }
        else
        {
            ProcessInfo::getInstance().solutionStatistics.numberOfProblemsOptimalMILP++;
        }
    }
    else if (currIter->type == E_IterationProblemType::Relaxed)
    {
        if (ProcessInfo::getInstance().originalProblem->isConstraintQuadratic(-1))
        {
            ProcessInfo::getInstance().solutionStatistics.numberOfProblemsQP++;
        }
        else
        {
            ProcessInfo::getInstance().solutionStatistics.numberOfProblemsLP++;
        }
    }
    else if (currIter->type == E_IterationProblemType::MIP && (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit || currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit || currIter->solutionStatus == E_ProblemSolutionStatus::NodeLimit))
    {
        if (ProcessInfo::getInstance().originalProblem->isConstraintQuadratic(-1))
        {
            ProcessInfo::getInstance().solutionStatistics.numberOfProblemsFeasibleMIQP++;
        }
        else
        {
            ProcessInfo::getInstance().solutionStatistics.numberOfProblemsFeasibleMILP++;
        }
    }

    ProcessInfo::getInstance().stopTimer("DualStrategy");
}

std::string TaskSolveIteration::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
