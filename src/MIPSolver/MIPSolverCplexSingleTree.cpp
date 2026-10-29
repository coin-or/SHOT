/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplexSingleTree.h"

#include "../Callback.h"
#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../PrimalSolver.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"

namespace SHOT
{

CplexCallback::CplexCallback(EnvironmentPtr envPtr, const IloNumVarArray& vars, double feasibilityTolerance)
    : feasibilityTolerance(feasibilityTolerance)
{
    env = envPtr;
    lastUpdatedPrimal = env->results->getPrimalBound();

    cplexVars = vars;

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("Dual.CutStrategy"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            tUpdateInteriorPoint = std::make_shared<TaskUpdateInteriorPoint>(env);
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanesESH>(env);
        }
        else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("Dual.CutStrategy"))
            == ES_HyperplaneCutStrategy::ECP)
        {
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanesECP>(env);
        }
    }

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        taskSelectHPPtsByObjectiveRootsearch = std::make_shared<TaskSelectHyperplanesObjectiveFunction>(env);
    }

    taskSelectExternalHPs = std::make_shared<TaskSelectHyperplanesExternal>(env);

    auto NLPProblemSource = static_cast<ES_PrimalNLPProblemSource>(
        env->settings->getSetting<int>("Primal.FixedInteger.SourceProblem"));

    // The tasks create the NLP problem, which is not needed when the fixed-integer NLP strategy is not used
    bool useFixedNLP = env->settings->getSetting<bool>("Primal.FixedInteger.Use");

    if(useFixedNLP
        && (NLPProblemSource == ES_PrimalNLPProblemSource::Both
            || NLPProblemSource == ES_PrimalNLPProblemSource::OriginalProblem))
    {
        taskSelectPrimNLPOriginal = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, false);
    }

    if(useFixedNLP
        && (NLPProblemSource == ES_PrimalNLPProblemSource::Both
            || NLPProblemSource == ES_PrimalNLPProblemSource::ReformulatedProblem))
    {
        taskSelectPrimNLPReformulated = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, true);
    }

    if(env->settings->getSetting<bool>("Primal.Rootsearch.Use")
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        taskSelectPrimalSolutionFromRootsearch = std::make_shared<TaskSelectPrimalCandidatesFromRootsearch>(env);
    }

    if(env->callbacks->isActive(E_CallbackLocation::PrimalCandidateSearch))
    {
        taskSelectPrimalSolutionFromExternal = std::make_shared<TaskSelectPrimalCandidatesFromExternalSource>(env);
    }

    isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;
}

CPXLONG CplexCallback::getContextMask()
{
    CPXLONG contextMask = IloCplex::Callback::Context::Id::Candidate | IloCplex::Callback::Context::Id::GlobalProgress
        | IloCplex::Callback::Context::Id::LocalProgress;

    // The relaxation context is needed to add the hyperplanes as user cuts, as Gurobi's MIPNODE context is always used
    contextMask |= IloCplex::Callback::Context::Id::Relaxation;

    return (contextMask);
}

void CplexCallback::invoke(const IloCplex::Callback::Context& context)
{
    try
    {
        // Only the bound in this context is global. In the other contexts it is the bound of the current thread,
        // which can be stronger than the global one, and can also decrease later.
        if(context.inGlobalProgress())
            globalDualBound = context.getDoubleInfo(IloCplex::Callback::Context::Info::BestBound);

        // The state of SHOT is shared by all threads, and Cplex may invoke the callback in several threads at once.
        // The progress contexts are invoked often and only check the termination, so they skip it if another thread
        // uses the state. Cplex also holds a lock of its own in the global progress context, so waiting there for a
        // thread that may need that lock could deadlock. The bound stored above is then used by the next invocation.
        std::unique_lock<std::mutex> lock(callbackMutex, std::defer_lock);

        if(context.inCandidate() || context.inRelaxation())
            lock.lock();
        else if(!lock.try_lock())
            return;

        updateDualBound();

        bool isAborted = false;

        if(context.inCandidate())
            isAborted = !handleCandidate(context);
        else if(context.inRelaxation())
            isAborted = !handleRelaxation(context);
        else
            isAborted = checkTermination(context);

        // Heuristic solutions can only be posted in these contexts
        std::optional<std::pair<VectorDouble, double>> primalSolution;

        if(!isAborted && !context.inLocalProgress())
            primalSolution = getPrimalSolutionToPost();

        // Posted after the lock is released, so that it is not held while Cplex processes the solution
        lock.unlock();

        if(primalSolution)
            postPrimalSolution(context, primalSolution->first, primalSolution->second);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Cplex error when invoking general callback", e.getMessage());
    }
}

bool CplexCallback::checkTermination(const IloCplex::Callback::Context& context)
{
    if(env->results->isAbsoluteObjectiveGapToleranceMet() || env->results->isRelativeObjectiveGapToleranceMet()
        || checkIterationLimit() || checkUserTermination())
    {
        context.abort();
        return (true);
    }

    return (false);
}

void CplexCallback::updateDualBound()
{
    double bound = globalDualBound.exchange(NAN);

    // CPLEX gives a missing bound as 1e75 or -1e75, which is a finite value to SHOT
    if(std::isnan(bound) || std::abs(bound) >= 1e75)
        return;

    if((isMinimization && bound > env->results->getCurrentDualBound())
        || (!isMinimization && bound < env->results->getCurrentDualBound()))
    {
        VectorDouble doubleSolution; // Empty since we have no point

        DualSolution sol = { doubleSolution, E_DualSolutionSource::MIPSolverBound, bound,
            env->results->getCurrentIteration()->iterationNumber, false };

        env->dualSolver->addDualSolutionCandidate(sol);
    }
}

bool CplexCallback::handleRelaxation(const IloCplex::Callback::Context& context)
{
    statistics.relaxationCalls++;

    if(checkTermination(context))
        return (false);

    addPendingUserCuts(context);

    if(env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
        >= env->settings->getSetting<int>("Dual.Relaxation.MaxLazyConstraints"))
        return (true);

    int waitingListSize = env->dualSolver->hyperplaneWaitingList.size();

    IloNumArray tmpVals(context.getEnv());

    context.getRelaxationPoint(cplexVars, tmpVals);

    int numberOfVariables
        = (env->dualSolver->hasObjectiveVariableOnlyInMIPSolver()) ? tmpVals.getSize() - 1 : tmpVals.getSize();

    VectorDouble solution(numberOfVariables);

    for(int i = 0; i < numberOfVariables; i++)
    {
        solution.at(i) = tmpVals[i];
    }

    tmpVals.end();

    SolutionPoint solutionRelaxed;

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
            solution, env->reformulatedProblem->nonlinearConstraints);
        solutionRelaxed.maxDeviation = PairIndexValue(maxDev.constraint->getIndex(), maxDev.normalizedValue);
    }
    else
    {
        solutionRelaxed.maxDeviation = PairIndexValue(-1, 0.0);
    }

    solutionRelaxed.point = solution;
    solutionRelaxed.objectiveValue = context.getRelaxationObjective();
    solutionRelaxed.iterFound = env->results->getCurrentIteration()->iterationNumber;
    solutionRelaxed.isRelaxedPoint = true;

    std::vector<SolutionPoint> solutionPoints = { solutionRelaxed };

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("Dual.CutStrategy"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            tUpdateInteriorPoint->run();
            static_cast<TaskSelectHyperplanesESH*>(taskSelectHPPts.get())->run(solutionPoints);
        }
        else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("Dual.CutStrategy"))
            == ES_HyperplaneCutStrategy::ECP)
        {
            static_cast<TaskSelectHyperplanesECP*>(taskSelectHPPts.get())->run(solutionPoints);
        }
    }

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        taskSelectHPPtsByObjectiveRootsearch->run(solutionPoints);
    }

    taskSelectExternalHPs->run(solutionPoints);

    env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
        += (env->dualSolver->hyperplaneWaitingList.size() - waitingListSize);

    return (true);
}

void CplexCallback::addPendingUserCuts(const IloCplex::Callback::Context& context)
{
    // The hyperplanes that Cplex has not kept, see handleCandidate
    for(auto i : pendingUserCuts)
    {
        try
        {
            IloExpr expr(context.getEnv());

            for(auto& T : rejectionCuts[i].terms)
                expr += T.value * cplexVars[T.index];

            IloRange tmpRange(context.getEnv(), -IloInfinity, expr, rejectionCuts[i].rightHandSide);

            context.addUserCut(tmpRange, IloCplex::UseCutForce, false);
            statistics.addedUserCuts++;

            tmpRange.end();
            expr.end();
        }
        catch(IloException& e)
        {
            env->output->outputError("        Error when adding hyperplane as user cut in Cplex", e.getMessage());
        }
    }

    pendingUserCuts.clear();
}

bool CplexCallback::handleCandidate(const IloCplex::Callback::Context& context)
{
    statistics.candidateCalls++;
    rejectionConstraints.clear();

    // A candidate can also be an unbounded ray, which is not rejected so that Cplex reports the problem as unbounded
    if(!context.isCandidatePoint())
    {
        statistics.candidateRays++;
        env->output->outputDebug("        Cplex reported an unbounded ray as candidate in the callback.");
        return (true);
    }

    IloNumArray tmpVals(context.getEnv());

    context.getCandidatePoint(cplexVars, tmpVals);

    VectorDouble candidatePoint(tmpVals.getSize());

    for(int i = 0; i < tmpVals.getSize(); i++)
    {
        candidatePoint.at(i) = tmpVals[i];
    }

    tmpVals.end();

    bool isRepeatedPoint = !candidatePointHashes.insert(Utilities::calculateHashes(candidatePoint)).second;

    if(isRepeatedPoint)
        statistics.repeatedCandidatePoints++;

    // Cplex does not guarantee that it keeps the constraints given when a candidate is rejected, and for some problems
    // it does not: it then reports points again that these hyperplanes cut off, and the relaxations, and so the dual
    // bound, do not improve. Its heuristics also find such points again. Gurobi's lazy constraints and the forced lazy
    // constraints of the legacy callback are kept instead. A hyperplane cutting off a point reported again is
    // therefore added as a user cut that Cplex never purges in the next relaxation context, which is the only way to
    // keep it in the generic callback. A user cut must not cut off a feasible solution of the problem, which the
    // hyperplanes do not, since the single-tree strategy is only used for convex problems, where they are valid for
    // all feasible solutions. Only these hyperplanes are added, since adding all of them makes the relaxations large.
    if(isRepeatedPoint)
    {
        for(size_t i = 0; i < rejectionCuts.size(); i++)
        {
            auto& C = rejectionCuts[i];

            double value = -C.rightHandSide;
            double largestCoefficient = 1.0;

            for(auto& T : C.terms)
            {
                value += T.value * candidatePoint.at(T.index);
                largestCoefficient = std::max(largestCoefficient, std::abs(T.value));
            }

            if(value <= feasibilityTolerance * largestCoefficient)
                continue;

            if(!C.isAddedAsUserCut)
            {
                C.isAddedAsUserCut = true;
                pendingUserCuts.push_back(i);
            }

            rejectionConstraints.push_back({ C.terms, -IloInfinity, C.rightHandSide, false });
        }

        // The point has been processed already, so it is only rejected again with the hyperplanes cutting it off
        if(rejectionConstraints.size() > 0)
        {
            statistics.candidatesViolatingRejectionCuts++;
            rejectCandidate(context);
            return (true);
        }
    }

    double candidateObjective = context.getCandidateObjective();

    // Check for new primal solution
    if((candidateObjective < 1e74)
        && ((isMinimization && candidateObjective < env->results->getPrimalBound())
            || (!isMinimization && candidateObjective > env->results->getPrimalBound())))
    {
        VectorDouble primalSolution(
            candidatePoint.begin(), candidatePoint.begin() + env->problem->properties.numberOfVariables);

        SolutionPoint tmpPt;

        if(env->problem->properties.numberOfNonlinearConstraints > 0)
        {
            auto maxDev
                = env->problem->getMaxNumericConstraintValue(primalSolution, env->problem->nonlinearConstraints);
            tmpPt.maxDeviation = PairIndexValue(maxDev.constraint->getIndex(), maxDev.normalizedValue);
        }
        else
        {
            tmpPt.maxDeviation = PairIndexValue(-1, 0.0);
        }

        tmpPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
        tmpPt.objectiveValue = env->problem->objectiveFunction->calculateValue(primalSolution);
        tmpPt.point = primalSolution;

        env->primalSolver->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::MIPCallback);
    }

    if(checkTermination(context))
        return (false);

    auto currIter = env->results->getCurrentIteration();

    if(currIter->isSolved)
    {
        env->results->createIteration();
        currIter = env->results->getCurrentIteration();
        currIter->isDualProblemDiscrete = true;
        currIter->dualProblemClass = env->dualSolver->MIPSolver->getProblemClass();
    }

    int numberOfVariables
        = (env->dualSolver->hasObjectiveVariableOnlyInMIPSolver()) ? candidatePoint.size() - 1 : candidatePoint.size();

    VectorDouble solution(candidatePoint.begin(), candidatePoint.begin() + numberOfVariables);

    // The bounds cannot be changed in the callback, so the MIP solver is interrupted and the artificial
    // bounds are removed before it solves again
    if(auto variables = env->problem->getVariablesAtArtificialBounds(solution); variables.size() > 0)
    {
        env->dualSolver->variablesAtArtificialBounds = variables;
        context.abort();
        return (false);
    }

    SolutionPoint solutionCandidate;

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
            solution, env->reformulatedProblem->nonlinearConstraints);

        solutionCandidate.maxDeviation = PairIndexValue(maxDev.constraint->getIndex(), maxDev.normalizedValue);
    }
    else
    {
        solutionCandidate.maxDeviation = PairIndexValue(-1, 0.0);
    }

    solutionCandidate.point = solution;
    solutionCandidate.objectiveValue = candidateObjective;
    solutionCandidate.iterFound = currIter->iterationNumber;

    std::vector<SolutionPoint> candidatePoints = { solutionCandidate };

    currIter->maxDeviation = solutionCandidate.maxDeviation.value;
    currIter->maxDeviationConstraint = solutionCandidate.maxDeviation.index;
    currIter->solutionStatus = E_ProblemSolutionStatus::Feasible;
    currIter->objectiveValue = candidateObjective;

    currIter->numberOfOpenNodes = static_cast<int>(context.getLongInfo(IloCplex::Callback::Context::Info::NodesLeft));
    env->solutionStatistics.numberOfExploredNodes
        = std::max(static_cast<int>(context.getLongInfo(IloCplex::Callback::Context::Info::NodeCount)),
            env->solutionStatistics.numberOfExploredNodes);

    auto bounds = std::make_pair(env->results->getCurrentDualBound(), env->results->getPrimalBound());
    currIter->currentObjectiveBounds = bounds;

    currIter->isSolved = true;

    auto threadId = std::to_string(context.getIntInfo(IloCplex::Callback::Context::Info::ThreadId));
    printIterationReport(candidatePoints.at(0), threadId);

    // As in the Gurobi callback, the hyperplanes are added before the primal heuristics are run
    addLazyConstraint(candidatePoints, context);

    addExternalDualBoundLazyConstraint(context);

    if(env->settings->getSetting<bool>("Primal.Rootsearch.Use")
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        taskSelectPrimalSolutionFromRootsearch->run(candidatePoints);
        env->primalSolver->checkPrimalSolutionCandidates();
    }

    // Unlike Gurobi, Cplex can report the same point again after it has been rejected, e.g. at another node whose
    // relaxation the hyperplanes have not reached yet. Such a point is not passed to the fixed NLP strategy, so that
    // the strategy counts the same candidates as with Gurobi.
    if(isRepeatedPoint)
    {
        statistics.fixedNLPRepeatedCandidates++;
    }
    else if(checkFixedNLPStrategy(candidatePoints.at(0)))
    {
        statistics.fixedNLPStrategyCalls++;

        env->primalSolver->addFixedNLPCandidate(candidatePoints.at(0).point, E_PrimalNLPSource::FirstSolution,
            candidateObjective, currIter->iterationNumber, candidatePoints.at(0).maxDeviation);

        // Solved on both problems before it is cleared, since a tested candidate is not added again
        if(taskSelectPrimNLPOriginal)
            taskSelectPrimNLPOriginal->run();

        if(taskSelectPrimNLPReformulated)
            taskSelectPrimNLPReformulated->run();

        env->primalSolver->fixedPrimalNLPCandidates.clear();

        env->primalSolver->checkPrimalSolutionCandidates();
    }

    if(env->callbacks->isActive(E_CallbackLocation::PrimalCandidateSearch))
    {
        taskSelectPrimalSolutionFromExternal->run();
    }

    if(env->settings->getSetting<bool>("Dual.HyperplaneCuts.UseIntegerCuts"))
    {
        int addedIntegerCuts = 0;

        for(auto& IC : env->dualSolver->integerCutWaitingList)
        {
            if(this->createIntegerCut(IC))
            {
                env->dualSolver->addGeneratedIntegerCut(IC);
                addedIntegerCuts++;
            }
        }

        if(addedIntegerCuts > 0)
            env->output->outputDebug(fmt::format("        Added {} integer cut(s)", addedIntegerCuts));

        env->dualSolver->integerCutWaitingList.clear();
    }

    // As with Gurobi and the legacy callback, a candidate that no constraint cuts off is accepted. Rejecting it without
    // a constraint would not move the relaxation away from it, and Cplex could give up the part of the tree containing
    // it, and with it optimal solutions.
    if(rejectionConstraints.size() > 0)
        rejectCandidate(context);

    return (true);
}

std::optional<std::pair<VectorDouble, double>> CplexCallback::getPrimalSolutionToPost()
{
    auto primalBound = env->results->getPrimalBound();

    if(!env->results->hasPrimalSolution()
        || !((isMinimization && lastUpdatedPrimal > primalBound)
            || (!isMinimization && lastUpdatedPrimal < primalBound)))
        return (std::nullopt);

    auto primalSol = env->results->primalSolution;

    if((int)primalSol.size() < env->reformulatedProblem->properties.numberOfVariables)
        env->reformulatedProblem->augmentAuxiliaryVariableValues(primalSol);

    if(env->dualSolver->hasObjectiveVariableOnlyInMIPSolver())
        primalSol.push_back(env->reformulatedProblem->objectiveFunction->calculateValue(primalSol));

    assert(static_cast<size_t>(cplexVars.getSize()) == primalSol.size());

    // Updated here, so that only one thread posts the solution
    lastUpdatedPrimal = primalBound;

    return (std::make_pair(primalSol, primalBound));
}

void CplexCallback::postPrimalSolution(
    const IloCplex::Callback::Context& context, const VectorDouble& solution, double objectiveValue)
{
    IloNumArray tmpVals(context.getEnv());

    for(double S : solution)
        tmpVals.add(S);

    try
    {
        context.postHeuristicSolution(
            cplexVars, tmpVals, objectiveValue, IloCplex::Callback::Context::SolutionStrategy::CheckFeasible);
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Error when setting primal solution as starting point in heuristic callback:", e.getMessage());
    }

    tmpVals.end();
}

void CplexCallback::outputStatistics()
{
    env->output->outputDebug(fmt::format("        Cplex generic callback: {} candidates ({} repeated points, {} rays), "
                                         "{} rejected, {} hyperplanes added",
        statistics.candidateCalls, statistics.repeatedCandidatePoints, statistics.candidateRays,
        statistics.rejectedCandidates, statistics.submittedHyperplanes));

    env->output->outputDebug(fmt::format(
        "        Cplex generic callback: {} relaxation calls, {} user cuts added for {} "
        "candidates violating earlier hyperplanes, fixed NLP strategy activated {} times, {} repeated points not "
        "passed to it",
        statistics.relaxationCalls, statistics.addedUserCuts, statistics.candidatesViolatingRejectionCuts,
        statistics.fixedNLPStrategyCalls, statistics.fixedNLPRepeatedCandidates));
}

void CplexCallback::rejectCandidate(const IloCplex::Callback::Context& context)
{
    IloRangeArray globalConstraints(context.getEnv());
    IloRangeArray localConstraints(context.getEnv());

    for(auto& C : rejectionConstraints)
    {
        IloExpr expr(context.getEnv());

        for(auto& T : C.terms)
            expr += T.value * cplexVars[T.index];

        if(C.isLocal)
            localConstraints.add(IloRange(context.getEnv(), C.lowerBound, expr, C.upperBound));
        else
            globalConstraints.add(IloRange(context.getEnv(), C.lowerBound, expr, C.upperBound));

        expr.end();
    }

    if(globalConstraints.getSize() > 0)
        context.rejectCandidate(globalConstraints);

    if(localConstraints.getSize() > 0)
        context.rejectCandidateLocal(localConstraints);

    statistics.rejectedCandidates++;

    globalConstraints.endElements();
    globalConstraints.end();
    localConstraints.endElements();
    localConstraints.end();

    rejectionConstraints.clear();
}

/// Destructor
CplexCallback::~CplexCallback() = default;

bool CplexCallback::createHyperplane(HyperplanePtr hyperplane, bool isNodeCandidate)
{
    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration
    auto optionalHyperplanes = env->dualSolver->MIPSolver->createHyperplaneTerms(hyperplane);

    if(!optionalHyperplanes)
    {
        return (false);
    }

    auto tmpPair = optionalHyperplanes.value();

    if(auto numericHyperplane = std::dynamic_pointer_cast<NumericHyperplane>(hyperplane))
    {
        for(auto& E : tmpPair.first)
        {
            if(E.second != E.second || std::isinf(E.second)) // Check for NaN or inf
            {
                env->output->outputError("        Warning: hyperplane not generated, NaN or inf "
                                         "found in linear terms for "
                    + env->reformulatedProblem->getVariable(E.first)->name + " = "
                    + std::to_string(numericHyperplane->generatedPoint.at(E.first)));

                return (false);
            }
        }
    }
    else if(auto externalHyperplane = std::dynamic_pointer_cast<ExternalHyperplane>(hyperplane))
    {
        for(auto& E : tmpPair.first)
        {
            if(E.second != E.second || std::isinf(E.second)) // Check for NaN or inf
            {
                env->output->outputError("        Warning: external hyperplane not generated, NaN or inf "
                                         "found in linear terms for "
                    + env->reformulatedProblem->getVariable(E.first)->name);

                return (false);
            }
        }
    }

    // Small fix to fix badly scaled cuts.
    // TODO: this should be made so it also takes into account small/large coefficients of the linear terms
    if(abs(tmpPair.second) > 1e15)
    {
        double scalingFactor = abs(tmpPair.second) - 1e15;

        for(auto& E : tmpPair.first)
            E.second /= scalingFactor;

        tmpPair.second /= scalingFactor;

        if(!warningMessageShownLargeRHS)
        {
            env->output->outputWarning(
                "        Large values found in RHS of cut, you might want to consider reducing the "
                "bounds of the nonlinear variables.");
            warningMessageShownLargeRHS = true;
        }
    }

    try
    {
        // Cplex only accepts local constraints for a candidate from the solution of a node
        bool isLocal = isNodeCandidate && hyperplane->source == E_HyperplaneSource::MIPCallbackRelaxed
            && env->settings->getSetting<bool>("Subsolver.Cplex.AddRelaxedLazyConstraintsAsLocal");

        std::vector<PairIndexValue> terms;
        terms.reserve(tmpPair.first.size());

        for(auto& P : tmpPair.first)
            terms.emplace_back(P.first, P.second);

        if(!isLocal)
            rejectionCuts.push_back({ terms, -tmpPair.second });

        rejectionConstraints.push_back({ std::move(terms), -IloInfinity, -tmpPair.second, isLocal });
        statistics.submittedHyperplanes++;

        env->dualSolver->addGeneratedHyperplane(hyperplane);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when creating hyperplane in Cplex", e.getMessage());
        return (false);
    }

    return (true);
}

bool CplexCallback::createIntegerCut(IntegerCut& integerCut)
{
    if(!integerCut.areAllVariablesBinary)
    {
        env->output->outputDebug("        Integer cut for nonbinary variables not supported in single-tree strategy.");
        return (false);
    }

    try
    {
        std::vector<PairIndexValue> terms;
        double constant = 0.0;
        size_t index = 0;

        for(auto& VAR : env->reformulatedProblem->allVariables)
        {
            if(!(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
                   || VAR->properties.type == E_VariableType::Semiinteger))
                continue;

            int variableValue = integerCut.variableValues[index];

            if(variableValue == VAR->upperBound)
            {
                constant += variableValue;
                terms.emplace_back(VAR->getIndex(), -1.0);
            }
            else if(variableValue == VAR->lowerBound)
            {
                terms.emplace_back(VAR->getIndex(), 1.0);
            }

            index++;
        }

        rejectionConstraints.push_back({ std::move(terms), 1.0 - constant, IloInfinity, false });
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when creating integer cut in Cplex", e.getMessage());
        return (false);
    }

    return (true);
}

void CplexCallback::addLazyConstraint(
    std::vector<SolutionPoint> candidatePoints, const IloCplex::Callback::Context& context)
{
    try
    {
        if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
        {

            if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("Dual.CutStrategy"))
                == ES_HyperplaneCutStrategy::ESH)
            {
                tUpdateInteriorPoint->run();
                static_cast<TaskSelectHyperplanesESH*>(taskSelectHPPts.get())->run(candidatePoints);
            }
            else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("Dual.CutStrategy"))
                == ES_HyperplaneCutStrategy::ECP)
            {
                static_cast<TaskSelectHyperplanesECP*>(taskSelectHPPts.get())->run(candidatePoints);
            }
        }

        if(env->reformulatedProblem->objectiveFunction->properties.classification
            > E_ObjectiveFunctionClassification::Quadratic)
        {
            taskSelectHPPtsByObjectiveRootsearch->run(candidatePoints);
        }

        taskSelectExternalHPs->run(candidatePoints);

        // Cplex discards a rejected candidate whether or not the constraints added with it are violated there. The
        // candidate is therefore only rejected if one of the hyperplanes cuts it off by more than the feasibility
        // tolerance of Cplex relative to its largest coefficient, which is how Cplex itself treats the lazy
        // constraints added in the legacy callback, and like Gurobi treats its lazy constraints. Otherwise a candidate
        // near the boundary, which the hyperplanes generated near it violate by less, is discarded without the
        // relaxation moving away from it, since Cplex does not use such a hyperplane. It then reports the same point
        // again, and eventually gives up that part of the tree, which can contain the optimal solutions. The
        // hyperplanes are also generated for the relaxed points, which the candidate need not violate.
        bool candidateIsCutOff = false;

        if(candidatePoints.size() > 0)
        {
            auto& candidate = candidatePoints.at(0);

            for(auto& hp : env->dualSolver->hyperplaneWaitingList)
            {
                auto terms = env->dualSolver->MIPSolver->createHyperplaneTerms(hp);

                if(!terms)
                    continue;

                double value = terms->second;
                double largestCoefficient = 1.0;

                for(auto& E : terms->first)
                {
                    value += E.second
                        * ((E.first < (int)candidate.point.size()) ? candidate.point.at(E.first)
                                                                   : candidate.objectiveValue);
                    largestCoefficient = std::max(largestCoefficient, std::abs(E.second));
                }

                if(value > feasibilityTolerance * largestCoefficient)
                {
                    candidateIsCutOff = true;
                    break;
                }
            }
        }

        if(!candidateIsCutOff)
        {
            if(env->dualSolver->hyperplaneWaitingList.size() > 0)
            {
                env->output->outputDebug("        Not rejecting the candidate, since none of the "
                    + std::to_string(env->dualSolver->hyperplaneWaitingList.size())
                    + " hyperplanes in the waiting list cuts it off.");
            }

            return;
        }

        bool isNodeCandidate
            = context.getIntInfo(IloCplex::Callback::Context::Info::CandidateSource) == CPX_LAZYCONSTRAINTCALLBACK_NODE;

        for(auto& hp : env->dualSolver->hyperplaneWaitingList)
        {
            if(this->createHyperplane(hp, isNodeCandidate))
                this->lastNumAddedHyperplanes++;
        }

        env->dualSolver->hyperplaneWaitingList.clear();
    }
    catch(IloException& e)
    {
        env->output->outputError("        Cplex error when invoking general lazy callback", e.getMessage());
    }
}

MIPSolverCplexSingleTree::MIPSolverCplexSingleTree(EnvironmentPtr envPtr)
{
    env = envPtr;

    discreteVariablesActivated = true;

    cplexModel = IloModel(cplexEnv);

    cplexVars = IloNumVarArray(cplexEnv);
    cplexConstrs = IloRangeArray(cplexEnv);

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;
    modelUpdated = false;
    checkParameters();
}

MIPSolverCplexSingleTree::~MIPSolverCplexSingleTree() = default;

void MIPSolverCplexSingleTree::initializeSolverSettings()
{
    try
    {
        MIPSolverCplex::initializeSolverSettings();

        // cplexInstance.setParam(IloCplex::NumericalEmphasis, 1);
    }
    catch(IloException& e)
    {
        env->output->outputError(" Error when initializing Cplex parameters: ", e.getMessage());
    }
}

E_ProblemSolutionStatus MIPSolverCplexSingleTree::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    MIPSolverCplex::cachedSolutionHasChanged = true;

    try
    { // If we in previous iteration solved a feasibility problem since the objective was unbounded, the original
        // objective needs to be restored
        if(objectiveFunctionReplacedWithZero)
        {
            cplexModel.remove(cplexInstance.getObjective());

            if(isMinimizationProblem)
                cplexModel.add(IloMinimize(cplexEnv, cplexObjectiveExpression));
            else
                cplexModel.add(IloMaximize(cplexEnv, cplexObjectiveExpression));

            modelUpdated = true;
        }

        if(modelUpdated)
        {
            // Extract the model if we have updated the constraints
            cplexInstance.extract(cplexModel);
            modelUpdated = false;
        }

        if(objectiveFunctionReplacedWithZero || !getDiscreteVariableStatus())
        {
            objectiveFunctionReplacedWithZero = false;

            // Fixes a deadlock bug in Cplex 12.7 and 12.8
            cplexEnv.setNormalizer(false);

            cplexInstance.solve();
            MIPSolutionStatus = MIPSolverCplex::getSolutionStatus();
        }
        else
        {
            CplexCallback cCallback(
                env, cplexVars, cplexInstance.getParam(IloCplex::Param::Simplex::Tolerances::Feasibility));

            // The callback is removed after this solve, also on an exception, since the instance may be solved again
            // below, e.g. as a feasibility problem, after the callback has been destroyed
            auto removeCallback = [&](CplexCallback*) { cplexInstance.use(nullptr, 0); };
            std::unique_ptr<CplexCallback, decltype(removeCallback)> callbackRegistration(nullptr, removeCallback);

            cplexInstance.use(&cCallback, cCallback.getContextMask());
            callbackRegistration.reset(&cCallback);

            // Fixes a deadlock bug in Cplex 12.7 and 12.8
            cplexEnv.setNormalizer(false);

            cplexInstance.solve();

            MIPSolutionStatus = MIPSolverCplex::getSolutionStatus();

            cCallback.outputStatistics();
        }

        // An unbounded exact dual problem means that the problem is unbounded, so no point is needed
        bool isUnboundedExactDualProblem
            = MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded && env->dualSolver->isDualProblemExact();

        if(isUnboundedExactDualProblem)
            MIPSolutionStatus = resolveInfeasibleOrUnbounded(MIPSolutionStatus);

        // Try to solve a feasibility problem to get a valid solution point if unbounded
        if(!isUnboundedExactDualProblem && MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded)
        {
            cplexModel.remove(cplexInstance.getObjective());

            if(isMinimizationProblem)
                cplexModel.add(IloMinimize(cplexEnv, SHOT_DBL_MIN));
            else
                cplexModel.add(IloMaximize(cplexEnv, SHOT_DBL_MAX));

            cplexInstance.extract(cplexModel);

            cplexInstance.solve();
            MIPSolutionStatus = getSolutionStatus();

            if(MIPSolutionStatus == E_ProblemSolutionStatus::Optimal)
                MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;

            objectiveFunctionReplacedWithZero = true;
            modelUpdated = true;
        }

        // If the previous repair failed, we can try this
        if(!isUnboundedExactDualProblem && MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded)
        {
            repairInfeasibility();
            MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
            env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed = true;
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

int MIPSolverCplexSingleTree::increaseSolutionLimit(int increment)
{
    int sollim = 0;

    try
    {
        cplexInstance.setParam(IloCplex::Param::MIP::Limits::Solutions,
            cplexInstance.getParam(IloCplex::Param::MIP::Limits::Solutions) + increment);
        sollim = cplexInstance.getParam(IloCplex::Param::MIP::Limits::Solutions);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when increasing solution limit", e.getMessage());
    }

    return (sollim);
}

void MIPSolverCplexSingleTree::setSolutionLimit(long limit)
{
    try
    {
        cplexInstance.setParam(IloCplex::Param::MIP::Limits::Solutions, limit);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when setting solution limit", e.getMessage());
    }
}

int MIPSolverCplexSingleTree::getSolutionLimit()
{
    int solLim = 0;

    try
    {
        solLim = cplexInstance.getParam(IloCplex::Param::MIP::Limits::Solutions);
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when obtaining solution limit", e.getMessage());
    }

    return (solLim);
}

void MIPSolverCplexSingleTree::checkParameters() { }

void CplexCallback::addExternalDualBoundLazyConstraint(const IloCplex::Callback::Context& context)
{
    if(auto newBound = queryAndUpdateExternalDualBound())
        pendingExternalDualBound = newBound;

    // The bound is only added with a candidate violating it, since rejecting a candidate also discards it when it
    // fulfills the constraint given with it
    auto newBound = pendingExternalDualBound;

    if(!newBound.has_value())
        return;

    try
    {
        std::vector<PairIndexValue> terms;

        if(env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
        {
            terms.emplace_back(env->dualSolver->MIPSolver->getDualAuxiliaryObjectiveVariableIndex(), 1.0);
        }
        else if(auto linObj
            = std::dynamic_pointer_cast<LinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction))
        {
            for(auto& T : linObj->linearTerms)
                terms.emplace_back(T->variable->getIndex(), T->coefficient);
        }

        IloExpr objExpr(context.getEnv());

        for(auto& T : terms)
            objExpr += T.value * cplexVars[T.index];

        double candidateValue = context.getCandidateValue(objExpr);
        objExpr.end();

        double tolerance = env->settings->getSetting<double>("Termination.ConstraintTolerance");

        if((isMinimization && candidateValue < *newBound - tolerance)
            || (!isMinimization && candidateValue > *newBound + tolerance))
        {
            rejectionConstraints.push_back({ std::move(terms), isMinimization ? *newBound : -IloInfinity,
                isMinimization ? IloInfinity : *newBound, false });

            env->output->outputDebug(fmt::format("        Added external dual bound lazy constraint {} auxiliary "
                                                 "variable, new bound: {}",
                env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable() ? "with" : "without", *newBound));

            pendingExternalDualBound.reset();
        }
    }
    catch(IloException& e)
    {
        env->output->outputError(
            "        Cplex error when adding external dual bound lazy constraint", e.getMessage());
    }
}

} // namespace SHOT