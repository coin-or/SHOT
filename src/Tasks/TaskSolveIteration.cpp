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
    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        for(auto& V : env->problem->allVariables)
        {

            variableNames.push_back(V->name);
        }
    }
}

TaskSolveIteration::~TaskSolveIteration() {}

void TaskSolveIteration::run()
{
    env->timing->startTimer("DualStrategy");
    auto currIter = env->results->getCurrentIteration();

    bool isMinimization
        = env->reformulatedProblem->objectiveFunction->direction == E_ObjectiveFunctionDirection::Minimize;

    // Sets the iteration time limit
    auto timeLim = env->settings->getSetting<double>("TimeLimit", "Termination") - env->timing->getElapsedTime("Total");
    env->dualSolver->MIPSolver->setTimeLimit(timeLim);

    if(env->dualSolver->useCutOff)
    {
        double cutOffValue;
        double cutOffValueConstraint;

        if(isMinimization)
        {
            cutOffValue
                = env->dualSolver->cutOffToUse + env->settings->getSetting<double>("MIP.CutOffTolerance", "Dual");
            cutOffValueConstraint = env->dualSolver->cutOffToUse;
        }
        else
        {
            cutOffValue
                = env->dualSolver->cutOffToUse - env->settings->getSetting<double>("MIP.CutOffTolerance", "Dual");
            cutOffValueConstraint = env->dualSolver->cutOffToUse;
        }

        env->dualSolver->MIPSolver->setCutOff(cutOffValue);

        if(env->reformulatedProblem->objectiveFunction->properties.classification
            < E_ObjectiveFunctionClassification::Quadratic)
            env->dualSolver->MIPSolver->setCutOffAsConstraint(cutOffValueConstraint);
    }

    if(env->dualSolver->MIPSolver->hasAuxiliaryObjectiveVariable()
        && env->settings->getSetting<bool>("MIP.UpdateObjectiveBounds", "Dual") && !currIter->MIPSolutionLimitUpdated)
    {
        auto newLB = env->results->getDualBound();
        auto newUB = env->results->getPrimalBound();

        auto currBounds = env->dualSolver->MIPSolver->getCurrentVariableBounds(
            env->dualSolver->MIPSolver->getAuxiliaryObjectiveVariableIndex());

        if(newLB > currBounds.first || newUB < currBounds.second)
        {
            env->dualSolver->MIPSolver->updateVariableBound(
                env->dualSolver->MIPSolver->getAuxiliaryObjectiveVariableIndex(), newLB, newUB);
            env->output->outputDebug("     Bounds for nonlinear objective function updated to "
                + Utilities::toString(newLB) + " and " + Utilities::toString(newUB));
        }
    }

    if(env->dualSolver->MIPSolver->getDiscreteVariableStatus() && env->results->primalSolutions.size() > 0)
    {
        auto tmpPrimal = env->results->primalSolution;

        env->dualSolver->MIPSolver->addMIPStart(tmpPrimal);
    }

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        std::stringstream ss;
        ss << env->settings->getSetting<std::string>("Debug.Path", "Output");
        ss << "/lp";
        ss << currIter->iterationNumber - 1;
        ss << ".lp";
        env->dualSolver->MIPSolver->writeProblemToFile(ss.str());
    }

    if(env->reformulatedProblem->properties.isLPProblem || env->reformulatedProblem->properties.isMILPProblem)
    {
        env->dualSolver->MIPSolver->setSolutionLimit(2100000000);
    }

    env->output->outputDebug("     Solving dual problem.");
    auto solStatus = env->dualSolver->MIPSolver->solveProblem();
    env->output->outputDebug("     Dual problem solved.");

    // Must update the pointer to the current iteration if we use the lazy
    // strategy since new iterations have been created when solving
    if(static_cast<ES_TreeStrategy>(env->settings->getSetting<int>("TreeStrategy", "Dual"))
        == ES_TreeStrategy::SingleTree)
    {
        currIter = env->results->getCurrentIteration();
    }
    else // Must update the node stats if multi-tree strategy (otherwise it is
         // done in the callbacks)
    {
        currIter->numberOfExploredNodes = env->dualSolver->MIPSolver->getNumberOfExploredNodes();
        env->solutionStatistics.numberOfExploredNodes += currIter->numberOfExploredNodes;
        env->solutionStatistics.numberOfOpenNodes = currIter->numberOfOpenNodes;
    }

    currIter->solutionStatus = solStatus;
    env->output->outputDebug("     Dual problem return code: " + std::to_string((int)solStatus));

    auto sols = env->dualSolver->MIPSolver->getAllVariableSolutions();

    if(sols.size() > 0)
    {
        env->output->outputDebug("        Number of solutions in solution pool: " + std::to_string(sols.size()));

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            std::stringstream ss;
            ss << env->settings->getSetting<std::string>("Debug.Path", "Output");
            ss << "/lpsolpt";
            ss << currIter->iterationNumber - 1;
            ss << ".txt";
            Utilities::saveVariablePointVectorToFile(sols.at(0).point, variableNames, ss.str());
        }

        if(env->reformulatedProblem->auxiliaryObjectiveVariable)
        {
            for(auto& S : sols)
            {
                S.point.pop_back();
            }
        }

        currIter->solutionPoints = sols;

        currIter->objectiveValue = env->dualSolver->MIPSolver->getObjectiveValue();

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            VectorDouble tmpObjValue;
            VectorString tmpObjName;

            tmpObjValue.push_back(env->dualSolver->MIPSolver->getObjectiveValue());
            tmpObjName.push_back("objective");

            std::stringstream ss;
            ss << env->settings->getSetting<std::string>("Debug.Path", "Output");
            ss << "/lpobjsol";
            ss << currIter->iterationNumber - 1;
            ss << ".txt";
            Utilities::saveVariablePointVectorToFile(tmpObjValue, tmpObjName, ss.str());
        }

        if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
        {
            auto mostDevConstr = env->reformulatedProblem->getMaxNumericConstraintValue(
                sols.at(0).point, env->reformulatedProblem->nonlinearConstraints);

            currIter->maxDeviationConstraint = mostDevConstr.constraint->index;
            currIter->maxDeviation = mostDevConstr.normalizedValue;

            if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
            {
                VectorDouble tmpMostDevValue;
                VectorString tmpConstrIndex;

                tmpMostDevValue.push_back(currIter->maxDeviation);
                tmpConstrIndex.push_back(std::to_string(currIter->maxDeviationConstraint));

                std::stringstream ss;
                ss << env->settings->getSetting<std::string>("Debug.Path", "Output");
                ss << "/lpmostdevm";
                ss << currIter->iterationNumber - 1;
                ss << ".txt";
                Utilities::saveVariablePointVectorToFile(tmpMostDevValue, tmpConstrIndex, ss.str());
            }
        }

        double currentDualBound = env->dualSolver->MIPSolver->getDualObjectiveValue();
        if(currIter->isMIP())
        {
            DualSolution sol = { sols.at(0).point, E_DualSolutionSource::MIPSolverBound, currentDualBound,
                currIter->iterationNumber };
            env->dualSolver->addDualSolutionCandidate(sol);

            if(currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
            {
                DualSolution sol = { sols.at(0).point, E_DualSolutionSource::MIPSolutionOptimal,
                    currIter->objectiveValue, currIter->iterationNumber };
                env->dualSolver->addDualSolutionCandidate(sol);
            }
        }
        else
        {
            DualSolution sol
                = { sols.at(0).point, E_DualSolutionSource::LPSolution, currentDualBound, currIter->iterationNumber };
            env->dualSolver->addDualSolutionCandidate(sol);
        }
    }

    currIter->usedMIPSolutionLimit = env->dualSolver->MIPSolver->getSolutionLimit();

    // Update solution stats
    if(currIter->type == E_IterationProblemType::MIP && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
    {
        if(env->reformulatedProblem->properties.isMIQPProblem)
        {
            env->solutionStatistics.numberOfProblemsOptimalMIQP++;
        }
        else if(env->reformulatedProblem->properties.isMIQCQPProblem)
        {
            env->solutionStatistics.numberOfProblemsOptimalMIQCQP++;
        }
        else
        {
            env->solutionStatistics.numberOfProblemsOptimalMILP++;
        }
    }
    else if(currIter->type == E_IterationProblemType::Relaxed)
    {
        if(env->reformulatedProblem->properties.isMIQPProblem)
        {
            env->solutionStatistics.numberOfProblemsQP++;
        }
        else if(env->reformulatedProblem->properties.isMIQCQPProblem)
        {
            env->solutionStatistics.numberOfProblemsQCQP++;
        }
        else
        {
            env->solutionStatistics.numberOfProblemsLP++;
        }
    }
    else if(currIter->type == E_IterationProblemType::MIP
        && (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
               || currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit
               || currIter->solutionStatus == E_ProblemSolutionStatus::NodeLimit))
    {

        if(env->reformulatedProblem->properties.isMIQPProblem)
        {
            env->solutionStatistics.numberOfProblemsFeasibleMIQP++;
        }
        else if(env->reformulatedProblem->properties.isMIQCQPProblem)
        {
            env->solutionStatistics.numberOfProblemsFeasibleMIQCQP++;
        }
        else
        {
            env->solutionStatistics.numberOfProblemsFeasibleMILP++;
        }
    }

    env->timing->stopTimer("DualStrategy");
}

std::string TaskSolveIteration::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT