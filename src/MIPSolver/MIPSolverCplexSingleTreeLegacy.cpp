/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplexSingleTreeLegacy.h"

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

HCallbackI::HCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2)
    : IloCplex::HeuristicCallbackI(iloEnv), cplexVars(xx2)
{
    env = envPtr;

    lastUpdatedPrimal = env->results->getPrimalBound();

    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexSingleTreeLegacy*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ESH)
    {
        tUpdateInteriorPoint = std::make_shared<TaskUpdateInteriorPoint>(env);
        taskSelectHPPts = std::make_shared<TaskSelectHyperplanesESH>(env);
    }
    else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ECP)
    {
        taskSelectHPPts = std::make_shared<TaskSelectHyperplanesECP>(env);
    }

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        taskSelectHPPtsByObjectiveRootsearch = std::make_shared<TaskSelectHyperplanesObjectiveFunction>(env);
    }

    taskSelectExternalHPs = std::make_shared<TaskSelectHyperplanesExternal>(env);
}

IloCplex::CallbackI* HCallbackI::duplicateCallback() const { return (new(getEnv()) HCallbackI(*this)); }

// This callback injects the best known primal solution into all threads.
void HCallbackI::main() // Called at each node...
{
    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexSingleTreeLegacy*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    bool isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;

    if(env->results->hasPrimalSolution()
        && ((isMinimization && lastUpdatedPrimal < env->results->getPrimalBound())
            || (!isMinimization && lastUpdatedPrimal > env->results->getPrimalBound())))
    {
        auto primalSol = env->results->primalSolution;

        IloNumArray tmpVals(this->getEnv());

        if((int)primalSol.size() < env->reformulatedProblem->properties.numberOfVariables)
            env->reformulatedProblem->augmentAuxiliaryVariableValues(primalSol);

        assert(env->reformulatedProblem->properties.numberOfVariables == primalSol.size());

        if(env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
            primalSol.push_back(env->reformulatedProblem->objectiveFunction->calculateValue(primalSol));

        for(double P : primalSol)
            tmpVals.add(P);

        for(auto& V : env->reformulatedProblem->auxiliaryVariables)
        {
            tmpVals.add(V->calculate(primalSol));
        }

        try
        {
            setSolution(cplexVars, tmpVals);
        }
        catch(IloException& e)
        {
            env->output->outputError(
                "        Error when setting primal solution as starting point in heuristic callback:", e.getMessage());
        }

        tmpVals.end();
        lastUpdatedPrimal = env->results->getPrimalBound();
    }

    if(env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
        < env->settings->getSetting<int>("Relaxation.MaxLazyConstraints", "Dual"))
    {
        int waitingListSize = env->dualSolver->hyperplaneWaitingList.size();

        std::vector<SolutionPoint> solutionPoints(1);

        IloNumArray tmpVals(getEnv());

        getValues(tmpVals, cplexVars);

        VectorDouble solution(tmpVals.getSize());

        for(int i = 0; i < tmpVals.getSize(); i++)
        {
            solution.at(i) = tmpVals[i];
        }

        tmpVals.end();

        SolutionPoint tmpSolPt;

        if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
        {
            auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
                solution, env->reformulatedProblem->nonlinearConstraints);
            tmpSolPt.maxDeviation = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);
        }
        else
        {
            tmpSolPt.maxDeviation = PairIndexValue(-1, 0.0);
        }

        tmpSolPt.point = solution;
        tmpSolPt.objectiveValue = getObjValue();
        tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
        tmpSolPt.isRelaxedPoint = true;

        solutionPoints.at(0) = tmpSolPt;

        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            static_cast<TaskSelectHyperplanesESH*>(taskSelectHPPts.get())->run(solutionPoints);
        }
        else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ECP)
        {
            static_cast<TaskSelectHyperplanesECP*>(taskSelectHPPts.get())->run(solutionPoints);
        }

        taskSelectExternalHPs->run(solutionPoints);

        env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
            += (env->dualSolver->hyperplaneWaitingList.size() - waitingListSize);
    }

    return;
}

InfoCallbackI::InfoCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv) : IloCplex::MIPInfoCallbackI(iloEnv)
{
    env = envPtr;
}

IloCplex::CallbackI* InfoCallbackI::duplicateCallback() const { return (new(getEnv()) InfoCallbackI(*this)); }

void InfoCallbackI::main() // Called at each node...
{
    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexSingleTreeLegacy*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    auto absObjGap = env->results->getAbsoluteGlobalObjectiveGap();
    auto relObjGap = env->results->getRelativeGlobalObjectiveGap();

    if(env->results->isRelativeObjectiveGapToleranceMet())
    {
        env->output->outputDebug(
            "        Terminated by relative objective gap tolerance in info callback: " + Utilities::toString(relObjGap)
            + " < " + Utilities::toString(env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination")));

        this->abort();
        return;
    }
    else if(env->results->isAbsoluteObjectiveGapToleranceMet())
    {
        env->output->outputDebug(
            "        Terminated by absolute objective gap tolerance in info callback: " + Utilities::toString(absObjGap)
            + " < " + Utilities::toString(env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination")));

        this->abort();
        return;
    }
    else if(checkIterationLimit())
    {
        env->output->outputDebug("        Terminated since iteration limit reached in info callback.");

        this->abort();
        return;
    }
    else if(checkUserTermination())
    {
        env->output->outputDebug("        Terminated by user.");

        this->abort();
        return;
    }

    return;
}

CtCallbackI::CtCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2)
    : IloCplex::LazyConstraintCallbackI(iloEnv), cplexVars(xx2)
{
    env = envPtr;

    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexSingleTreeLegacy*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    env->solutionStatistics.iterationLastLazyAdded = 0;
    isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            tUpdateInteriorPoint = std::make_shared<TaskUpdateInteriorPoint>(env);
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanesESH>(env);
        }
        else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ECP)
        {
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanesECP>(env);
        }
    }

    taskSelectExternalHPs = std::make_shared<TaskSelectHyperplanesExternal>(env);

    auto NLPProblemSource = static_cast<ES_PrimalNLPProblemSource>(
        env->settings->getSetting<int>("FixedInteger.SourceProblem", "Primal"));

    if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
        || NLPProblemSource == ES_PrimalNLPProblemSource::OriginalProblem)
    {
        taskSelectPrimNLPOriginal = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, false);
    }

    if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
        || NLPProblemSource == ES_PrimalNLPProblemSource::ReformulatedProblem)
    {
        taskSelectPrimNLPReformulated = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, true);
    }

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        taskSelectHPPtsByObjectiveRootsearch = std::make_shared<TaskSelectHyperplanesObjectiveFunction>(env);
    }

    if(env->settings->getSetting<bool>("Rootsearch.Use", "Primal")
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        taskSelectPrimalSolutionFromRootsearch = std::make_unique<TaskSelectPrimalCandidatesFromRootsearch>(env);
    }
}

IloCplex::CallbackI* CtCallbackI::duplicateCallback() const { return (new(getEnv()) CtCallbackI(*this)); }

void CtCallbackI::main()
{
    auto currIter = env->results->getCurrentIteration();

    if(currIter->isSolved)
    {
        env->results->createIteration();
        currIter = env->results->getCurrentIteration();
        currIter->isDualProblemDiscrete = true;
        currIter->dualProblemClass = env->dualSolver->MIPSolver->getProblemClass();
    }

    currIter->isSolved = true;

    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexSingleTreeLegacy*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    IloNumArray tmpVals(this->getEnv());

    this->getValues(tmpVals, cplexVars);

    int numberOfVariables
        = (env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable()) ? tmpVals.getSize() - 1 : tmpVals.getSize();

    VectorDouble solution(numberOfVariables);

    for(int i = 0; i < numberOfVariables; i++)
    {
        solution.at(i) = tmpVals[i];
    }

    tmpVals.end();

    SolutionPoint solutionCandidate;

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
            solution, env->reformulatedProblem->nonlinearConstraints);

        solutionCandidate.maxDeviation = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);
    }
    else
    {
        solutionCandidate.maxDeviation = PairIndexValue(-1, 0.0);
    }

    double tmpDualObjBound = this->getBestObjValue();

    solutionCandidate.point = solution;
    solutionCandidate.objectiveValue = getObjValue();
    solutionCandidate.iterFound = env->results->getCurrentIteration()->iterationNumber;

    // Check if better dual bound
    if((isMinimization && tmpDualObjBound > env->results->getCurrentDualBound())
        || (!isMinimization && tmpDualObjBound < env->results->getCurrentDualBound()))
    {
        DualSolution sol = { solution, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound,
            env->results->getCurrentIteration()->iterationNumber, false };
        env->dualSolver->addDualSolutionCandidate(sol);
    }

    // Check if better primal solution
    if(this->hasIncumbent())
    {
        double tmpPrimalObjBound = this->getIncumbentObjValue();

        if((isMinimization && tmpPrimalObjBound < env->results->getPrimalBound())
            || (!isMinimization && tmpPrimalObjBound > env->results->getPrimalBound()))
        {
            IloNumArray tmpPrimalVals(this->getEnv());
            this->getIncumbentValues(tmpPrimalVals, cplexVars);

            VectorDouble primalSolution(tmpPrimalVals.getSize());

            for(int i = 0; i < tmpPrimalVals.getSize(); i++)
            {
                primalSolution.at(i) = tmpPrimalVals[i];
            }

            SolutionPoint tmpPt;

            if(env->problem->properties.numberOfNonlinearConstraints > 0)
            {
                auto maxDev = env->problem->getMaxNumericConstraintValue(solution, env->problem->nonlinearConstraints);
                tmpPt.maxDeviation = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);
            }
            else
            {
                tmpPt.maxDeviation = PairIndexValue(-1, 0.0);
            }

            tmpPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
            tmpPt.objectiveValue = this->getIncumbentObjValue();
            tmpPt.point = primalSolution;
            env->primalSolver->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::MIPCallback);
            tmpPrimalVals.end();
            primalSolution.clear();
        }
    }

    std::vector<SolutionPoint> candidatePoints(1);

    candidatePoints.at(0) = solutionCandidate;

    auto threadId = std::to_string(this->getMyThreadNum());

    currIter->maxDeviation = solutionCandidate.maxDeviation.value;
    currIter->maxDeviationConstraint = solutionCandidate.maxDeviation.index;
    currIter->solutionStatus = E_ProblemSolutionStatus::Feasible;
    currIter->objectiveValue = this->getIncumbentObjValue();

    currIter->numberOfOpenNodes = this->getNremainingNodes();
    env->solutionStatistics.numberOfExploredNodes
        = std::max((int)this->getNnodes(), env->solutionStatistics.numberOfExploredNodes);

    auto bounds = std::make_pair(env->results->getCurrentDualBound(), env->results->getPrimalBound());
    currIter->currentObjectiveBounds = bounds;

    printIterationReport(candidatePoints.at(0), threadId);

    if(env->results->isAbsoluteObjectiveGapToleranceMet())
    {
        env->output->outputDebug("        Terminated by absolute objective gap tolerance in lazy callback");

        solution.clear();
        abort();
        return;
    }

    if(env->results->isRelativeObjectiveGapToleranceMet())
    {
        env->output->outputDebug("        Terminated by relative objective gap tolerance in lazy callback");

        solution.clear();
        abort();
        return;
    }

    if(checkIterationLimit())
    {
        env->output->outputDebug("        Terminated by iteration limit in lazy callback");

        solution.clear();
        abort();
        return;
    }

    if(checkUserTermination())
    {
        env->output->outputDebug("        Terminated by user in lazy callback");

        solution.clear();
        abort();
        return;
    }

    if(env->settings->getSetting<bool>("Rootsearch.Use", "Primal")
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        taskSelectPrimalSolutionFromRootsearch->run(candidatePoints);
    }

    if(checkFixedNLPStrategy(solutionCandidate))
    {
        if(taskSelectPrimNLPOriginal)
        {
            env->primalSolver->addFixedNLPCandidate(solution, E_PrimalNLPSource::FirstSolution, this->getObjValue(),
                env->results->getCurrentIteration()->iterationNumber, solutionCandidate.maxDeviation);

            taskSelectPrimNLPOriginal->run();
            env->primalSolver->fixedPrimalNLPCandidates.clear();
        }

        if(taskSelectPrimNLPReformulated)
        {
            env->primalSolver->addFixedNLPCandidate(solution, E_PrimalNLPSource::FirstSolution, this->getObjValue(),
                env->results->getCurrentIteration()->iterationNumber, solutionCandidate.maxDeviation);

            taskSelectPrimNLPReformulated->run();
            env->primalSolver->fixedPrimalNLPCandidates.clear();
        }

        env->primalSolver->checkPrimalSolutionCandidates();

        if(env->results->isAbsoluteObjectiveGapToleranceMet())
        {
            env->output->outputDebug("        Terminated by absolute objective gap tolerance in lazy callback");

            solution.clear();
            abort();
            return;
        }

        if(env->results->isRelativeObjectiveGapToleranceMet())
        {
            env->output->outputDebug("        Terminated by relative objective gap tolerance in lazy callback");

            solution.clear();
            abort();
            return;
        }
    }

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            tUpdateInteriorPoint->run();
            static_cast<TaskSelectHyperplanesESH*>(taskSelectHPPts.get())->run(candidatePoints);
        }
        else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
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

    for(auto& hp : env->dualSolver->hyperplaneWaitingList)
    {
        if(this->createHyperplane(hp))
            this->lastNumAddedHyperplanes++;
    }

    env->dualSolver->hyperplaneWaitingList.clear();

    if(env->settings->getSetting<bool>("HyperplaneCuts.UseIntegerCuts", "Dual"))
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

    candidatePoints.clear();
    solution.clear();
}

bool CtCallbackI::createHyperplane(HyperplanePtr hyperplane)
{
    auto optional = env->dualSolver->MIPSolver->createHyperplaneTerms(hyperplane);

    if(!optional)
    {
        return (false);
    }

    auto tmpPair = optional.value();

    for(auto& E : tmpPair.first)
    {
        if(E.second != E.second) // Check for NaN
        {
            env->output->outputError(
                "        Warning: hyperplane not generated, NaN found in linear terms for variable "
                + env->problem->getVariable(E.first)->name);
            return (false);
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

    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    try
    {
        IloExpr expr(this->getEnv());

        for(auto& P : tmpPair.first)
        {
            expr += P.second * cplexVars[P.first];
        }

        IloRange tmpRange(this->getEnv(), -IloInfinity, expr, -tmpPair.second);

        tmpPair.first.clear();
        expr.end();

        if(env->settings->getSetting<bool>("Cplex.AddRelaxedLazyConstraintsAsLocal", "Subsolver")
            && hyperplane->source == E_HyperplaneSource::MIPCallbackRelaxed)
        {
            addLocal(tmpRange).end();
        }
        else
        {
            add(tmpRange, IloCplex::CutManagement::UseCutForce).end();
        }

        env->dualSolver->addGeneratedHyperplane(hyperplane);

        optional.value().first.clear();
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when creating hyperplane in Cplex callback ", e.getMessage());
        return (false);
    }

    return (true);
}

bool CtCallbackI::createIntegerCut(IntegerCut& integerCut)
{
    if(!integerCut.areAllVariablesBinary)
    {
        env->output->outputDebug("        Integer cut for nonbinary variables not supported in single-tree strategy.");
        return (false);
    }

    try
    {
        IloExpr expr(this->getEnv());
        size_t index = 0;

        for(auto& VAR : env->reformulatedProblem->allVariables)
        {
            if(!(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
                   || VAR->properties.type == E_VariableType::Semiinteger))
                continue;

            int variableValue = integerCut.variableValues[index];

            if(variableValue == VAR->upperBound)
            {
                expr += (variableValue - cplexVars[VAR->index]);
            }
            else if(variableValue == VAR->lowerBound)
            {
                expr += cplexVars[VAR->index];
            }

            index++;
        }

        IloRange tmpRange(this->getEnv(), 1, expr, IloInfinity,
            fmt::format("IC{}", env->solutionStatistics.numberOfIntegerCuts).c_str());

        add(tmpRange);

        tmpRange.end();
        expr.end();
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when creating integer cut in Cplex callback ", e.getMessage());
        return (false);
    }

    return (true);
}

MIPSolverCplexSingleTreeLegacy::MIPSolverCplexSingleTreeLegacy(EnvironmentPtr envPtr)
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

void MIPSolverCplexSingleTreeLegacy::initializeSolverSettings()
{
    try
    {
        MIPSolverCplex::initializeSolverSettings();
    }
    catch(IloException& e)
    {
        env->output->outputError(" Error when initializing Cplex parameters: ", e.getMessage());
    }
}

E_ProblemSolutionStatus MIPSolverCplexSingleTreeLegacy::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    MIPSolverCplex::cachedSolutionHasChanged = true;

    try
    {
        // If we in previous iteration solved a feasibility problem since the objective was unbounded, the original
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

        if(objectiveFunctionReplacedWithZero)
        {
            // Do not want the callbacks the first iteration after tinkering with the objective
            objectiveFunctionReplacedWithZero = false;
        }
        else
        {
            if(getDiscreteVariableStatus())
            {
                ctCallback = new(cplexEnv) CtCallbackI(env, cplexEnv, cplexVars);
                hCallback = new(cplexEnv) HCallbackI(env, cplexEnv, cplexVars);
                infoCallback = new(cplexEnv) InfoCallbackI(env, cplexEnv);
                callbacksInitialized = true;

                cplexInstance.use(ctCallback);
                cplexInstance.use(hCallback);
                cplexInstance.use(infoCallback);
            }
        }

        cplexInstance.solve();
        MIPSolutionStatus = getSolutionStatus();

        // Try to solve a feasibility problem to get a valid solution point if unbounded
        if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded)
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

        if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded)
        {
            repairInfeasibility();
            MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
            env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed = true;
        }

        if(callbacksInitialized)
        {
            cplexInstance.remove(ctCallback);
            cplexInstance.remove(hCallback);
            cplexInstance.remove(infoCallback);
            delete ctCallback;
            delete hCallback;
            delete infoCallback;
            callbacksInitialized = false;
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

int MIPSolverCplexSingleTreeLegacy::increaseSolutionLimit(int increment)
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

void MIPSolverCplexSingleTreeLegacy::setSolutionLimit(long limit)
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

int MIPSolverCplexSingleTreeLegacy::getSolutionLimit()
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

void MIPSolverCplexSingleTreeLegacy::checkParameters() { }
} // namespace SHOT