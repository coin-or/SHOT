/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSolveIteration.h"

namespace SHOT
{

TaskSolveIteration::TaskSolveIteration(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
}

TaskSolveIteration::~TaskSolveIteration()
{
}

void TaskSolveIteration::run()
{
    env->process->startTimer("DualStrategy");
    auto currIter = env->process->getCurrentIteration();

    bool isMinimization = env->reformulatedProblem->objectiveFunction->direction == E_ObjectiveFunctionDirection::Minimize;

    // Sets the iteration time limit
    auto timeLim = env->settings->getDoubleSetting("TimeLimit", "Termination") - env->process->getElapsedTime("Total");
    env->dualSolver->setTimeLimit(timeLim);

    if (env->process->primalSolutions.size() > 0)
    {
        if (isMinimization)
        {
            env->dualSolver->setCutOff(
                env->process->getPrimalBound() + env->settings->getDoubleSetting("MIP.CutOffTolerance", "Dual"));
        }
        else
        {
            env->dualSolver->setCutOff(
                env->process->getPrimalBound() - env->settings->getDoubleSetting("MIP.CutOffTolerance", "Dual"));
        }
    }

    if (env->dualSolver->hasAuxilliaryObjectiveVariable && env->settings->getBoolSetting("MIP.UpdateObjectiveBounds", "Dual") && !currIter->MIPSolutionLimitUpdated)
    {
        auto newLB = env->process->getDualBound();
        auto newUB = env->process->getPrimalBound();

        auto currBounds = env->dualSolver->getCurrentVariableBounds(env->dualSolver->auxilliaryObjectiveVariableIndex);

        if (newLB > currBounds.first || newUB < currBounds.second)
        {
            env->dualSolver->updateVariableBound(env->dualSolver->auxilliaryObjectiveVariableIndex, newLB, newUB);
            env->output->outputInfo(
                "     Bounds for nonlinear objective function updated to " + UtilityFunctions::toString(newLB) + " and " + UtilityFunctions::toString(newUB));
        }
    }

    if (env->dualSolver->getDiscreteVariableStatus() && env->process->primalSolutions.size() > 0)
    {
        env->dualSolver->addMIPStart(env->process->primalSolution);
    }

    if (env->settings->getBoolSetting("Debug.Enable", "Output"))
    {
        std::stringstream ss;
        ss << env->settings->getStringSetting("Debug.Path", "Output");
        ss << "/lp";
        ss << currIter->iterationNumber - 1;
        ss << ".lp";
        env->dualSolver->writeProblemToFile(ss.str());
    }

    env->output->outputInfo("     Solving dual problem.");
    auto solStatus = env->dualSolver->solveProblem();
    env->output->outputInfo("     Dual problem solved.");

    // Must update the pointer to the current iteration if we use the lazy strategy since new iterations have been created when solving
    if (static_cast<ES_TreeStrategy>(env->settings->getIntSetting("TreeStrategy", "Dual")) == ES_TreeStrategy::SingleTree)
    {
        currIter = env->process->getCurrentIteration();
    }
    else // Must update the node stats if multi-tree strategy (otherwise it is done in the callbacks)
    {
        currIter->numberOfExploredNodes = env->dualSolver->getNumberOfExploredNodes();
        env->solutionStatistics.numberOfExploredNodes += currIter->numberOfExploredNodes;
        env->solutionStatistics.numberOfOpenNodes = currIter->numberOfOpenNodes;
        //std::cout << "Nodes: " << env->solutionStatistics.numberOfExploredNodes << std::endl;
    }

    currIter->solutionStatus = solStatus;
    env->output->outputInfo("     Dual problem return code: " + std::to_string((int)solStatus));

    if (solStatus == E_ProblemSolutionStatus::Infeasible ||
        solStatus == E_ProblemSolutionStatus::Error ||
        solStatus == E_ProblemSolutionStatus::Abort ||
        solStatus == E_ProblemSolutionStatus::CutOff ||
        solStatus == E_ProblemSolutionStatus::Numeric ||
        solStatus == E_ProblemSolutionStatus::Unbounded)
    {
    }
    else
    {
        auto sols = env->dualSolver->getAllVariableSolutions();
        currIter->solutionPoints = sols;

        if (sols.size() > 0)
        {
            if (env->settings->getBoolSetting("Debug.Enable", "Output"))
            {
                std::stringstream ss;
                ss << env->settings->getStringSetting("Debug.Path", "Output");
                ss << "/lpsolpt";
                ss << currIter->iterationNumber - 1;
                ss << ".txt";
                UtilityFunctions::saveVariablePointVectorToFile(sols.at(0).point, env->problem->allVariables, ss.str());
            }

            currIter->objectiveValue = env->dualSolver->getObjectiveValue();

            if (env->settings->getBoolSetting("Debug.Enable", "Output"))
            {
                VectorDouble tmpObjValue;
                VectorString tmpObjName;

                tmpObjValue.push_back(env->dualSolver->getObjectiveValue());
                tmpObjName.push_back("objective");

                std::stringstream ss;
                ss << env->settings->getStringSetting("Debug.Path", "Output");
                ss << "/lpobjsol";
                ss << currIter->iterationNumber - 1;
                ss << ".txt";
                UtilityFunctions::saveVariablePointVectorToFile(tmpObjValue, tmpObjName, ss.str());
            }

            auto mostDevConstr = env->model->originalProblem->getMostDeviatingConstraint(sols.at(0).point);

            currIter->maxDeviationConstraint = mostDevConstr.index;
            currIter->maxDeviation = mostDevConstr.value;

            if (env->settings->getBoolSetting("Debug.Enable", "Output"))
            {
                VectorDouble tmpMostDevValue;
                VectorString tmpConstrIndex;

                tmpMostDevValue.push_back(mostDevConstr.value);
                tmpConstrIndex.push_back(std::to_string(mostDevConstr.index));

                std::stringstream ss;
                ss << env->settings->getStringSetting("Debug.Path", "Output");
                ss << "/lpmostdevm";
                ss << currIter->iterationNumber - 1;
                ss << ".txt";
                UtilityFunctions::saveVariablePointVectorToFile(tmpMostDevValue, tmpConstrIndex, ss.str());
            }

            double tmpDualObjBound = env->dualSolver->getDualObjectiveValue();
            if (currIter->isMIP())
            {
                DualSolution sol =
                    {sols.at(0).point, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound,
                     currIter->iterationNumber};
                env->process->addDualSolutionCandidate(sol);

                if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
                {
                    DualSolution sol =
                        {sols.at(0).point, E_DualSolutionSource::MIPSolutionOptimal, currIter->objectiveValue,
                         currIter->iterationNumber};
                    env->process->addDualSolutionCandidate(sol);
                }
            }
            else
            {
                DualSolution sol =
                    {sols.at(0).point, E_DualSolutionSource::LPSolution, tmpDualObjBound,
                     currIter->iterationNumber};
                env->process->addDualSolutionCandidate(sol);
            }
        }
    }

    currIter->usedMIPSolutionLimit = env->dualSolver->getSolutionLimit();

    // Update solution stats
    if (currIter->type == E_IterationProblemType::MIP && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
    {
        if (env->model->originalProblem->isConstraintQuadratic(-1))
        {
            env->solutionStatistics.numberOfProblemsOptimalMIQP++;
        }
        else
        {
            env->solutionStatistics.numberOfProblemsOptimalMILP++;
        }
    }
    else if (currIter->type == E_IterationProblemType::Relaxed)
    {
        if (env->model->originalProblem->isConstraintQuadratic(-1))
        {
            env->solutionStatistics.numberOfProblemsQP++;
        }
        else
        {
            env->solutionStatistics.numberOfProblemsLP++;
        }
    }
    else if (currIter->type == E_IterationProblemType::MIP && (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit || currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit || currIter->solutionStatus == E_ProblemSolutionStatus::NodeLimit))
    {
        if (env->model->originalProblem->isConstraintQuadratic(-1))
        {
            env->solutionStatistics.numberOfProblemsFeasibleMIQP++;
        }
        else
        {
            env->solutionStatistics.numberOfProblemsFeasibleMILP++;
        }
    }

    env->process->stopTimer("DualStrategy");
}

std::string TaskSolveIteration::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT