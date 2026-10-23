/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskEnumerateFixedIntegerCombinations.h"

#include "TaskSelectPrimalCandidatesFromNLP.h"

#include "../Iteration.h"
#include "../Output.h"
#include "../PrimalSolver.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"

#include <algorithm>
#include <cmath>

namespace SHOT
{

TaskEnumerateFixedIntegerCombinations::TaskEnumerateFixedIntegerCombinations(EnvironmentPtr envPtr, bool isFallback)
    : TaskBase(envPtr), isFallback(isFallback)
{
}

TaskEnumerateFixedIntegerCombinations::~TaskEnumerateFixedIntegerCombinations() = default;

void TaskEnumerateFixedIntegerCombinations::run()
{
    // Both instances are run at most once
    if(isFallback ? env->solutionStatistics.hasFixedIntegerEnumerationFallbackBeenRun
                  : env->solutionStatistics.hasFixedIntegerEnumerationBeenRun)
        return;

    if(isFallback && !isFallbackNeeded())
        return;

    if(!initializeDomains())
        return;

    // Nothing is left for the fallback if all combinations were solved in the search before the dual strategy, or in
    // the fixed-integer strategy
    if(isFallback && !hasUnusedCombinations())
        return;

    // The NLP tasks start and stop the timers when they are created
    createNLPTasks();

    if(NLPTasks.size() == 0)
        return;

    env->timing->startTimer("PrimalStrategy");
    env->timing->startTimer("PrimalBoundStrategyNLP");

    enumerateCombinations();

    env->timing->stopTimer("PrimalBoundStrategyNLP");
    env->timing->stopTimer("PrimalStrategy");
}

std::string TaskEnumerateFixedIntegerCombinations::getType()
{
    std::string type = typeid(this).name();
    return (type);
}

bool TaskEnumerateFixedIntegerCombinations::isFallbackNeeded()
{
    if(env->results->isAbsoluteObjectiveGapToleranceMet() || env->results->isRelativeObjectiveGapToleranceMet())
        return (false);

    // A limit or a termination by the user does not mean that the dual strategy cannot close the gap, and all NLP
    // problems are infeasible if the problem is
    switch(env->results->terminationReason)
    {
    case E_TerminationReason::Error:
    case E_TerminationReason::NumericIssues:
    case E_TerminationReason::ObjectiveStagnation:
    case E_TerminationReason::NoDualCutsAdded:
        return (true);
    case E_TerminationReason::InfeasibleProblem:
        // The cuts of nonconvex constraints can make the dual problem infeasible although the problem is feasible
        return (env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex
            && !env->results->hasPrimalSolution());
    default:
        return (false);
    }
}

bool TaskEnumerateFixedIntegerCombinations::initializeDomains()
{
    if(env->problem->properties.numberOfSemicontinuousVariables > 0
        || env->problem->properties.numberOfSemiintegerVariables > 0
        || env->problem->properties.numberOfSpecialOrderedSets > 0)
    {
        env->output->outputDebug("        Exhaustive fixed-integer search not performed since the problem has "
                                 "semicontinuous or semiinteger variables, or special ordered sets.");
        return (false);
    }

    discreteVariableIndexes.clear();
    domainLowerBounds.clear();
    domainUpperBounds.clear();

    double maxCombinations = env->settings->getSetting<int>("Primal.FixedInteger.Enumeration.MaxCombinations");
    double combinations = 1.0;

    for(auto& V : env->problem->allVariables)
    {
        if(V->properties.type != E_VariableType::Binary && V->properties.type != E_VariableType::Integer)
            continue;

        // The bounds can be slightly off an integer value after bound tightening
        double lowerBound = std::ceil(V->lowerBound - 1e-9);
        double upperBound = std::floor(V->upperBound + 1e-9);

        if(upperBound < lowerBound)
        {
            env->output->outputDebug(fmt::format(
                "        Exhaustive fixed-integer search not performed since variable {} has no integer value within "
                "its bounds.",
                V->name));
            return (false);
        }

        // The number of combinations is compared in each step, so the product does not grow further than this
        combinations *= (upperBound - lowerBound + 1.0);

        if(combinations > maxCombinations)
        {
            env->output->outputDebug(fmt::format(
                "        Exhaustive fixed-integer search not performed since there are more than {} combinations of "
                "the discrete variables.",
                env->settings->getSetting<int>("Primal.FixedInteger.Enumeration.MaxCombinations")));
            return (false);
        }

        discreteVariableIndexes.push_back(V->getIndex());
        domainLowerBounds.push_back(lowerBound);
        domainUpperBounds.push_back(upperBound);
    }

    // A problem without discrete variables has one combination, the NLP problem itself. It is only solved in the
    // fallback, where it is the problem without the starting point from the dual strategy
    if(discreteVariableIndexes.size() == 0 && !isFallback)
        return (false);

    numberOfCombinations = (int)combinations;

    return (true);
}

PrimalFixedNLPCandidate TaskEnumerateFixedIntegerCombinations::createCandidate(const VectorDouble& combination)
{
    // The continuous variables are given the value closest to zero within their bounds. This is not a starting point,
    // the values are needed to calculate those of the auxiliary variables in the reformulated problem
    VectorDouble point(env->problem->properties.numberOfVariables);

    for(auto& V : env->problem->allVariables)
        point[V->getIndex()] = std::max(V->lowerBound, std::min(V->upperBound, 0.0));

    for(size_t i = 0; i < discreteVariableIndexes.size(); i++)
        point[discreteVariableIndexes[i]] = combination[i];

    return (env->primalSolver->createFixedNLPCandidate(point, E_PrimalNLPSource::Enumeration, NAN,
        env->results->getCurrentIteration()->iterationNumber, PairIndexValue(-1, 0.0)));
}

bool TaskEnumerateFixedIntegerCombinations::selectNextCombination(VectorDouble& combination)
{
    // The last variable changes fastest
    for(int i = (int)combination.size() - 1; i >= 0; i--)
    {
        if(combination[i] < domainUpperBounds[i])
        {
            combination[i] += 1.0;
            return (true);
        }

        combination[i] = domainLowerBounds[i];
    }

    return (false);
}

bool TaskEnumerateFixedIntegerCombinations::hasCombinationBeenSolved(const PrimalFixedNLPCandidate& candidate)
{
    // Without discrete variables, all NLP problems solved in the fixed-integer strategy have the same combination,
    // and they were solved from another starting point
    if(discreteVariableIndexes.size() > 0
        && env->primalSolver->hasFixedNLPCandidateBeenTested(candidate.discreteVariablePointHashes))
        return (true);

    auto& hashes = env->primalSolver->enumeratedPrimalNLPCandidateHashes;

    return (std::any_of(hashes.begin(), hashes.end(),
        [&candidate](auto& H) { return (Utilities::haveSameHashes(H, candidate.discreteVariablePointHashes)); }));
}

bool TaskEnumerateFixedIntegerCombinations::hasUnusedCombinations()
{
    VectorDouble combination(domainLowerBounds);

    do
    {
        if(!hasCombinationBeenSolved(createCandidate(combination)))
            return (true);
    } while(selectNextCombination(combination));

    return (false);
}

void TaskEnumerateFixedIntegerCombinations::createNLPTasks()
{
    if(NLPTasks.size() > 0)
        return;

    auto NLPProblemSource
        = static_cast<ES_PrimalNLPProblemSource>(env->settings->getSetting<int>("Primal.FixedInteger.SourceProblem"));

    if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
        || NLPProblemSource == ES_PrimalNLPProblemSource::OriginalProblem)
        NLPTasks.push_back(std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, false));

    if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
        || NLPProblemSource == ES_PrimalNLPProblemSource::ReformulatedProblem)
        NLPTasks.push_back(std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, true));

    NLPTasks.erase(
        std::remove_if(NLPTasks.begin(), NLPTasks.end(), [](auto& T) { return (!T->hasNLPSolver()); }), NLPTasks.end());
}

void TaskEnumerateFixedIntegerCombinations::enumerateCombinations()
{
    auto& statistics = env->solutionStatistics;

    statistics.hasFixedIntegerEnumerationBeenRun = true;

    if(isFallback)
        statistics.hasFixedIntegerEnumerationFallbackBeenRun = true;

    // The numbers are those of the last search
    statistics.numberOfFixedIntegerEnumerationCombinations = numberOfCombinations;
    statistics.numberOfFixedIntegerEnumerationCombinationsFeasible = 0;
    statistics.numberOfFixedIntegerEnumerationCombinationsInfeasible = 0;
    statistics.numberOfFixedIntegerEnumerationCombinationsUnresolved = 0;
    statistics.numberOfFixedIntegerEnumerationCombinationsSkipped = 0;

    double timeLimit = env->settings->getSetting<double>("Primal.FixedInteger.Enumeration.TimeLimit");
    double timeUsed = 0.0;

    VectorDouble combination(domainLowerBounds);

    std::string endReason = "All combinations solved.";
    bool hasMoreCombinations = true;

    while(hasMoreCombinations)
    {
        if(env->tasks->isTerminated())
        {
            endReason = "Terminated by user.";
            break;
        }

        auto candidate = createCandidate(combination);

        if(isFallback && hasCombinationBeenSolved(candidate))
        {
            statistics.numberOfFixedIntegerEnumerationCombinationsSkipped++;
        }
        else
        {
            bool isFeasible = false;
            bool isConclusive = true;
            int numberSolved = 0;
            bool isOutOfTime = false;

            for(auto& T : NLPTasks)
            {
                // The auxiliary discrete variables are fixed in the NLP problem of the reformulated problem
                if(T->usesReformulatedProblem()
                    && std::any_of(env->reformulatedProblem->allVariables.begin(),
                        env->reformulatedProblem->allVariables.end(), [&candidate](auto& V) {
                            return ((V->properties.type == E_VariableType::Binary
                                        || V->properties.type == E_VariableType::Integer)
                                && !std::isfinite(candidate.point[V->getIndex()]));
                        }))
                {
                    isConclusive = false;
                    continue;
                }

                double timeLeft = std::min(timeLimit - timeUsed,
                    env->settings->getSetting<double>("Termination.TimeLimit") - env->timing->getElapsedTime("Total"));

                if(timeLeft <= 0.0)
                {
                    isOutOfTime = true;
                    break;
                }

                double timeStart = env->timing->getElapsedTime("Total");

                auto status = T->solveFixedNLPCandidates({ candidate }, true,
                    std::min(timeLeft, env->settings->getSetting<double>("Primal.FixedInteger.TimeLimit")));

                timeUsed += env->timing->getElapsedTime("Total") - timeStart;
                numberSolved++;

                if(status == E_NLPSolutionStatus::Optimal || status == E_NLPSolutionStatus::Feasible)
                    isFeasible = true;
                else if(status != E_NLPSolutionStatus::Infeasible)
                    isConclusive = false;
            }

            if(isOutOfTime)
            {
                endReason = (timeUsed >= timeLimit) ? "Time limit of the search reached." : "Time limit reached.";

                if(numberSolved == 0)
                    break;

                isConclusive = false;
            }

            // Solving it again from the same starting point would give the same result
            if(!isOutOfTime)
                env->primalSolver->enumeratedPrimalNLPCandidateHashes.push_back(candidate.discreteVariablePointHashes);

            if(isFeasible)
                statistics.numberOfFixedIntegerEnumerationCombinationsFeasible++;
            else if(isConclusive)
                statistics.numberOfFixedIntegerEnumerationCombinationsInfeasible++;
            else
                statistics.numberOfFixedIntegerEnumerationCombinationsUnresolved++;

            // A combination that could not be solved can still be used in the fixed-integer strategy with a point from
            // the dual solver
            if(isConclusive)
                env->primalSolver->usedPrimalNLPCandidates.push_back(candidate);

            if(isOutOfTime)
                break;
        }

        hasMoreCombinations = selectNextCombination(combination);
    }

    int numberSolved = statistics.numberOfFixedIntegerEnumerationCombinationsFeasible
        + statistics.numberOfFixedIntegerEnumerationCombinationsInfeasible
        + statistics.numberOfFixedIntegerEnumerationCombinationsUnresolved;

    env->output->outputInfo(fmt::format("        Exhaustive fixed-integer search{}: {} of {} combinations solved in {:.2f} "
                                        "s ({} feasible, {} infeasible, {} unresolved, {} skipped). {}",
        isFallback ? " (fallback)" : "", numberSolved, numberOfCombinations, timeUsed, statistics.numberOfFixedIntegerEnumerationCombinationsFeasible,
        statistics.numberOfFixedIntegerEnumerationCombinationsInfeasible,
        statistics.numberOfFixedIntegerEnumerationCombinationsUnresolved,
        statistics.numberOfFixedIntegerEnumerationCombinationsSkipped, endReason));
}
} // namespace SHOT
