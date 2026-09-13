/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskSolveIteration.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../Report.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../MIPSolver/IMIPSolver.h"

#include "../Model/Problem.h"

#include <algorithm>
#include <optional>

namespace SHOT
{

namespace
{
// The variables of the problem that are at a bound that has replaced a missing bound in the point
std::vector<VariablePtr> getVariablesAtArtificialBounds(EnvironmentPtr env, const VectorDouble& point)
{
    std::vector<VariablePtr> variables;

    for(auto& V : env->problem->allVariables)
    {
        if(V->getIndex() >= (int)point.size())
            continue;

        double value = point.at(V->getIndex());

        // The variables are integer, so a smaller difference means that the variable is at the bound
        if((V->properties.hasArtificialLowerBound && value < V->lowerBound + 0.5)
            || (V->properties.hasArtificialUpperBound && value > V->upperBound - 0.5))
            variables.push_back(V);
    }

    return (variables);
}
} // namespace

TaskSolveIteration::TaskSolveIteration(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    if(env->settings->getSetting<bool>("Output.Debug.Enable"))
    {
        for(auto& V : env->reformulatedProblem->allVariables)
        {
            variableNames.push_back(V->name);
        }

        if(env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
        {
            variableNames.push_back("shot_dual_objvar");
        }
    }
}

TaskSolveIteration::~TaskSolveIteration() = default;

void TaskSolveIteration::run()
{
    if(!env->report->firstIterationHeaderPrinted)
    {
        env->report->outputPreReport();
        env->report->outputIterationDetailHeader();
    }

    env->timing->startTimer("DualStrategy");
    auto currIter = env->results->getCurrentIteration();

    bool isMinimization
        = env->reformulatedProblem->objectiveFunction->direction == E_ObjectiveFunctionDirection::Minimize;

    // The reformulated problem's objective can have a different direction than the original problem's (e.g. a
    // maximize objective reformulated into an equivalent minimize one); raw values read off the MIP/LP solver are
    // in the reformulated problem's sense, while DualSolver/Results interpret DualSolution values in the
    // original problem's sense, so they need to be translated here.
    double objectiveSignFactor
        = (env->reformulatedProblem->objectiveFunction->direction == env->problem->objectiveFunction->direction)
        ? 1.0
        : -1.0;

    // Sets the iteration time limit
    auto timeLim = env->settings->getSetting<double>("Termination.TimeLimit") - env->timing->getElapsedTime("Total");
    env->dualSolver->MIPSolver->setTimeLimit(timeLim);

    // The cutoff in the original problem's sense, if one is used when solving
    std::optional<double> usedCutOff;

    if(env->dualSolver->useCutOff && !currIter->MIPSolutionLimitUpdated)
    {
        usedCutOff = env->dualSolver->cutOffToUse;

        double cutOffValue;
        double cutOffValueConstraint;

        // cutOffToUse is a primal bound in the original problem's sense; translate it into the reformulated
        // problem's sense before using it to bound the reformulated MIP's own objective/objective variable.
        double reformulatedCutOff = objectiveSignFactor * env->dualSolver->cutOffToUse;

        if(isMinimization)
        {
            cutOffValue = reformulatedCutOff + env->settings->getSetting<double>("Dual.MIP.CutOff.Tolerance");
            cutOffValueConstraint = reformulatedCutOff;
        }
        else
        {
            cutOffValue = reformulatedCutOff - env->settings->getSetting<double>("Dual.MIP.CutOff.Tolerance");
            cutOffValueConstraint = reformulatedCutOff;
        }

        env->output->outputDebug(fmt::format("        Setting cutoff value to {}.", reformulatedCutOff));

        env->dualSolver->MIPSolver->setCutOff(cutOffValue);

        if(env->reformulatedProblem->objectiveFunction->properties.classification
            != E_ObjectiveFunctionClassification::Quadratic)
            env->dualSolver->MIPSolver->setCutOffAsConstraint(cutOffValueConstraint);
    }

    if(env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable()
        && env->settings->getSetting<bool>("Dual.MIP.UpdateObjectiveBounds") && !currIter->MIPSolutionLimitUpdated)
    {
        auto newLB = env->results->getCurrentDualBound();
        auto newUB = env->results->getPrimalBound();

        auto currBounds = env->dualSolver->MIPSolver->getCurrentVariableBounds(
            env->dualSolver->MIPSolver->getDualAuxiliaryObjectiveVariableIndex());

        if(newLB > currBounds.first || newUB < currBounds.second)
        {
            env->dualSolver->MIPSolver->updateVariableBound(
                env->dualSolver->MIPSolver->getDualAuxiliaryObjectiveVariableIndex(), newLB, newUB);
            env->output->outputDebug(
                fmt::format("        Bounds for nonlinear objective function updated to {} and {}", newLB, newUB));
        }
    }

    if(env->dualSolver->MIPSolver->getDiscreteVariableStatus() && env->results->hasPrimalSolution())
    {
        auto primalSol = env->results->primalSolution;
        assert(primalSol.size() == env->problem->properties.numberOfVariables);

        env->reformulatedProblem->augmentAuxiliaryVariableValues(primalSol);

        if(env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
        {
            primalSol.push_back(env->results->getPrimalBound());
        }

        env->dualSolver->MIPSolver->addMIPStart(primalSol);

        if(env->settings->getSetting<bool>("Output.Debug.Enable"))
        {
            auto filename = fmt::format("{}/dualiter{}_mipstart.txt",
                env->settings->getSetting<std::string>("Output.Debug.Path"), currIter->iterationNumber - 1);
            Utilities::saveVariablePointVectorToFile(primalSol, variableNames, filename);
        }
    }

    if(env->settings->getSetting<bool>("Output.Debug.Enable"))
    {
        auto filename = fmt::format("{}/dualiter{}_problem.lp",
            env->settings->getSetting<std::string>("Output.Debug.Path"), currIter->iterationNumber - 1);

        env->dualSolver->MIPSolver->writeProblemToFile(filename);
    }

    if(env->reformulatedProblem->properties.isLPProblem || env->reformulatedProblem->properties.isMILPProblem
        || env->reformulatedProblem->properties.isMIQPProblem)
    {
        env->dualSolver->MIPSolver->setSolutionLimit(2100000000);
    }

    env->output->outputDebug("        Solving dual problem.");
    auto solStatus = env->dualSolver->MIPSolver->solveProblem();

    // Must update the pointer to the current iteration if we use the lazy
    // strategy since new iterations have been created when solving
    if(static_cast<ES_TreeStrategy>(env->settings->getSetting<int>("Dual.TreeStrategy")) == ES_TreeStrategy::SingleTree)
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

    env->output->outputDebug(fmt::format("        Dual problem solved with return code: {}", (int)solStatus));

    auto sols = env->dualSolver->MIPSolver->getAllVariableSolutions();

    // A solution with a variable at a bound that has only replaced a missing bound of the problem does not show what
    // the optimal objective value is, since the problem may be unbounded or have its optimum beyond the bound. If the
    // dual problem is exact, the bounds are removed and it is solved again, and otherwise the solution gives no dual
    // bound.
    bool isSolutionAtArtificialBound = false;

    if(sols.size() > 0)
    {
        auto variables = getVariablesAtArtificialBounds(env, sols.at(0).point);

        if(variables.size() > 0 && env->dualSolver->isDualProblemExact())
        {
            double lowerLimit = env->settings->getSetting<double>("Model.Variables.Continuous.MinimumLowerBound");
            double upperLimit = env->settings->getSetting<double>("Model.Variables.Continuous.MaximumUpperBound");
            double unboundedValue = env->dualSolver->MIPSolver->getUnboundedVariableBoundValue();

            for(auto& V : variables)
            {
                env->output->outputDebug(fmt::format(
                    "        Removing artificial bounds of variable {} since the solution is at them.", V->name));

                if(V->properties.hasArtificialLowerBound)
                    V->lowerBound = lowerLimit;

                if(V->properties.hasArtificialUpperBound)
                    V->upperBound = upperLimit;

                auto reformulatedVariable = env->reformulatedProblem->getVariable(V->getIndex());
                reformulatedVariable->lowerBound = V->lowerBound;
                reformulatedVariable->upperBound = V->upperBound;

                env->dualSolver->MIPSolver->updateVariableBound(V->getIndex(),
                    V->properties.hasArtificialLowerBound ? -unboundedValue : V->lowerBound,
                    V->properties.hasArtificialUpperBound ? unboundedValue : V->upperBound);

                V->properties.hasArtificialLowerBound = false;
                V->properties.hasArtificialUpperBound = false;
            }

            solStatus = env->dualSolver->MIPSolver->solveProblem();
            currIter->solutionStatus = solStatus;

            env->output->outputDebug(
                fmt::format("        Dual problem solved again with return code: {}", (int)solStatus));

            sols = env->dualSolver->MIPSolver->getAllVariableSolutions();

            if(sols.size() > 0)
                variables = getVariablesAtArtificialBounds(env, sols.at(0).point);
            else
                variables.clear();
        }

        isSolutionAtArtificialBound = variables.size() > 0;
    }

    if(sols.size() > 0)
    {
        env->output->outputDebug(fmt::format("        Number of solutions in solution pool: {} ", sols.size()));

        if(env->settings->getSetting<bool>("Output.Debug.Enable"))
        {
            auto debugPath = env->settings->getSetting<std::string>("Output.Debug.Path");

            for(size_t i = 0; i < sols.size(); i++)
            {
                auto filename = fmt::format("{}/dualiter{}_solpt_{}.txt", debugPath, currIter->iterationNumber - 1, i);
                Utilities::saveVariablePointVectorToFile(sols.at(i).point, variableNames, filename);
            }

            for(size_t i = 0; i < sols.size(); i++)
            {
                auto filename
                    = fmt::format("{}/dualiter{}_solinfo_{}.txt", debugPath, currIter->iterationNumber - 1, i);
                auto filecontents
                    = fmt::format("objective function value\t\t{}\nmax constr. dev. ([index]: value)\t[{}]: {}\n",
                        sols.at(i).objectiveValue, sols.at(i).maxDeviation.index, sols.at(i).maxDeviation.value);
                Utilities::writeStringToFile(filename, filecontents);
            }
        }

        currIter->objectiveValue = env->dualSolver->MIPSolver->getObjectiveValue();

        if(env->reformulatedProblem->antiEpigraphObjectiveVariable)
        {
            for(auto& SOL : sols)
                SOL.point.at(env->reformulatedProblem->antiEpigraphObjectiveVariable->getIndex())
                    = currIter->objectiveValue;
        }

        currIter->solutionPoints = sols;

        if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
        {
            auto mostDevConstr = env->reformulatedProblem->getMaxNumericConstraintValue(
                sols.at(0).point, env->reformulatedProblem->nonlinearConstraints);

            currIter->maxDeviationConstraint = mostDevConstr.constraint->getIndex();
            currIter->maxDeviation = mostDevConstr.normalizedValue;

            if(env->settings->getSetting<bool>("Output.Debug.Enable"))
            {
                auto filename = fmt::format("{}/dualiter{}_mostdev.txt",
                    env->settings->getSetting<std::string>("Output.Debug.Path"), currIter->iterationNumber - 1);
                auto filecontents = fmt::format("most dev. constraint ([index]: value)\t[{}]: {}\n",
                    currIter->maxDeviationConstraint, currIter->maxDeviation);

                Utilities::writeStringToFile(filename, filecontents);
            }
        }
        else
        {
            currIter->maxDeviationConstraint = -1;
            currIter->maxDeviation = 0.0;
        }

        if(isSolutionAtArtificialBound)
        {
            env->output->outputDebug(
                "        Dual bound ignored since the solution is at an artificial bound of a variable.");
        }
        else if(!env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed)
        {
            double currentDualBound = objectiveSignFactor * env->dualSolver->MIPSolver->getDualObjectiveValue();

            if(currIter->isMIP())
            {
                // The objective value of the solution is only a valid dual bound if the dual problem has been solved
                // to proven optimality. The MIP solvers however also report optimality when their own gap tolerance
                // has been met, in which case the optimal value can be anywhere between the bound reported by the
                // solver and the objective value of its solution, so the reported bound is used in both cases.
                auto source = (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
                    ? E_DualSolutionSource::MIPSolutionOptimal
                    : E_DualSolutionSource::MIPSolverBound;

                DualSolution sol
                    = { sols.at(0).point, source, currentDualBound, currIter->iterationNumber, false };
                env->dualSolver->addDualSolutionCandidate(sol);
            }
            else
            {
                DualSolution sol = { sols.at(0).point, E_DualSolutionSource::LPSolution, currentDualBound,
                    currIter->iterationNumber, false };
                env->dualSolver->addDualSolutionCandidate(sol);
            }
        }
    }
    else
    {
        env->output->outputDebug("        Dual solver reports no solutions found.");

        // If the dual problem is a relaxation of the problem, which it is as long as all cuts are valid everywhere
        // and it has not been repaired, it being infeasible with the cutoff shows that no solution is better than the
        // cutoff. The cutoff is then a dual bound, which e.g. closes the gap when it is the primal bound. Artificial
        // bounds restrict the problem, so the dual problem is then not a relaxation.
        bool hasArtificialBounds = std::any_of(env->problem->allVariables.begin(), env->problem->allVariables.end(),
            [](const VariablePtr& V)
            { return (V->properties.hasArtificialLowerBound || V->properties.hasArtificialUpperBound); });

        if(solStatus == E_ProblemSolutionStatus::Infeasible && usedCutOff && env->results->solutionIsGlobal
            && !currIter->hasInfeasibilityRepairBeenPerformed && env->results->hasPrimalSolution()
            && !hasArtificialBounds)
        {
            env->output->outputDebug(
                fmt::format("        Dual problem infeasible with the cutoff, so {} is a dual bound.", *usedCutOff));

            DualSolution sol
                = { { }, E_DualSolutionSource::InfeasibleWithCutOff, *usedCutOff, currIter->iterationNumber, false };
            env->dualSolver->addDualSolutionCandidate(sol);
        }
        // The bound returned by the MIP solver is not valid for the original problem if the dual problem has been
        // repaired, e.g. by replacing its objective function with a constant one
        else if(!currIter->hasInfeasibilityRepairBeenPerformed)
        {
            DualSolution sol = { { }, E_DualSolutionSource::MIPSolverBound,
                objectiveSignFactor * env->dualSolver->MIPSolver->getDualObjectiveValue(), currIter->iterationNumber,
                false };
            env->dualSolver->addDualSolutionCandidate(sol);
        }
        else
        {
            env->output->outputDebug("        Dual bound ignored since the dual problem has been repaired.");
        }
    }

    currIter->usedMIPSolutionLimit = env->dualSolver->MIPSolver->getSolutionLimit();

    // Update solution stats
    if(currIter->isDualProblemDiscrete && currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
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
    else if(!currIter->isDualProblemDiscrete)
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
    else if(currIter->isDualProblemDiscrete
        && (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
            || currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit
            || currIter->solutionStatus == E_ProblemSolutionStatus::NodeLimit
            || currIter->solutionStatus == E_ProblemSolutionStatus::Abort))
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