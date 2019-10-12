/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplexLazyOriginalCallback.h"

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
        (static_cast<MIPSolverCplexLazyOriginalCallback*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ESH)
    {
        tUpdateInteriorPoint = std::make_shared<TaskUpdateInteriorPoint>(env);
        taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsESH>(env);
    }
    else
    {
        taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsECP>(env);
    }

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        taskSelectHPPtsByObjectiveRootsearch = std::make_shared<TaskSelectHyperplanePointsByObjectiveRootsearch>(env);
    }
}

HCallbackI::~HCallbackI() = default;

IloCplex::CallbackI* HCallbackI::duplicateCallback() const { return (new(getEnv()) HCallbackI(*this)); }

static IloCplex::Callback HCallback(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray cplexVars)
{
    return (IloCplex::Callback(new(iloEnv) HCallbackI(envPtr, iloEnv, cplexVars)));
}

// This callback injects the best known primal solution into all threads.
void HCallbackI::main() // Called at each node...
{
    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexLazyOriginalCallback*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    bool isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;

    if(env->results->hasPrimalSolution()
        && ((isMinimization && lastUpdatedPrimal < env->results->getPrimalBound())
               || (!isMinimization && lastUpdatedPrimal > env->results->getPrimalBound())))
    {
        auto primalSol = env->results->primalSolution;

        IloNumArray tmpVals(this->getEnv());

        for(double S : primalSol)
        {
            tmpVals.add(S);
        }

        for(auto& V : env->reformulatedProblem->auxiliaryVariables)
        {
            tmpVals.add(V->calculate(primalSol));
        }

        if(env->reformulatedProblem->auxiliaryObjectiveVariable)
            tmpVals.add(env->reformulatedProblem->auxiliaryObjectiveVariable->calculate(primalSol));
        else if(env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
            tmpVals.add(env->reformulatedProblem->objectiveFunction->calculateValue(primalSol));

        try
        {
            setSolution(cplexVars, tmpVals);
        }
        catch(IloException& e)
        {
            env->output->outputError(
                "Error when setting primal solution as starting point in heuristic callback:", e.getMessage());
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
            static_cast<TaskSelectHyperplanePointsESH*>(taskSelectHPPts.get())->run(solutionPoints);
        }
        else
        {
            static_cast<TaskSelectHyperplanePointsECP*>(taskSelectHPPts.get())->run(solutionPoints);
        }

        env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
            += (env->dualSolver->hyperplaneWaitingList.size() - waitingListSize);
    }

    return;
}

InfoCallbackI::InfoCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2)
    : IloCplex::MIPInfoCallbackI(iloEnv), cplexVars(xx2)
{
    env = envPtr;
}

InfoCallbackI::~InfoCallbackI() = default;

IloCplex::CallbackI* InfoCallbackI::duplicateCallback() const { return (new(getEnv()) InfoCallbackI(*this)); }

static IloCplex::Callback InfoCallback(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray cplexVars)
{
    return (IloCplex::Callback(new(iloEnv) InfoCallbackI(envPtr, iloEnv, cplexVars)));
}

void InfoCallbackI::main() // Called at each node...
{
    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexLazyOriginalCallback*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    auto absObjGap = env->results->getAbsoluteGlobalObjectiveGap();
    auto relObjGap = env->results->getRelativeGlobalObjectiveGap();

    if(env->results->isRelativeObjectiveGapToleranceMet())
    {
        env->output->outputCritical(
            "     Terminated by relative objective gap tolerance in info callback: " + Utilities::toString(relObjGap)
            + " < " + Utilities::toString(env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination")));

        this->abort();
        return;
    }
    else if(env->results->isAbsoluteObjectiveGapToleranceMet())
    {
        env->output->outputCritical(
            "     Terminated by absolute objective gap tolerance in info callback: " + Utilities::toString(absObjGap)
            + " < " + Utilities::toString(env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination")));

        this->abort();
        return;
    }
    else if(checkIterationLimit())
    {
        env->output->outputCritical("     Terminated since iteration limit reached in info callback.");

        this->abort();
        return;
    }
    else if(checkUserTermination())
    {
        env->output->outputCritical("     Terminated due to termination by user.");

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
        (static_cast<MIPSolverCplexLazyOriginalCallback*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    env->solutionStatistics.iterationLastLazyAdded = 0;
    isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;
    cbCalls = 0;

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            tUpdateInteriorPoint = std::make_shared<TaskUpdateInteriorPoint>(env);
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsESH>(env);
        }
        else
        {
            taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsECP>(env);
        }
    }

    tSelectPrimNLP = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env);

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        taskSelectHPPtsByObjectiveRootsearch = std::make_shared<TaskSelectHyperplanePointsByObjectiveRootsearch>(env);
    }

    if(env->settings->getSetting<bool>("Rootsearch.Use", "Primal")
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        taskSelectPrimalSolutionFromRootsearch = std::make_shared<TaskSelectPrimalCandidatesFromRootsearch>(env);
    }
}

CtCallbackI::~CtCallbackI() = default;

IloCplex::CallbackI* CtCallbackI::duplicateCallback() const { return (new(getEnv()) CtCallbackI(*this)); }

static IloCplex::Callback CtCallback(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray cplexVars)
{
    return (IloCplex::Callback(new(iloEnv) CtCallbackI(envPtr, iloEnv, cplexVars)));
}

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
        (static_cast<MIPSolverCplexLazyOriginalCallback*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    this->cbCalls++;

    IloNumArray tmpVals(this->getEnv());

    this->getValues(tmpVals, cplexVars);

    int size
        = (env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable()) ? tmpVals.getSize() - 1 : tmpVals.getSize();

    VectorDouble solution(size);

    for(int i = 0; i < size; i++)
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
            env->primalSolver->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::LazyConstraintCallback);
            tmpPrimalVals.end();
            primalSolution.clear();
        }
    }

    std::vector<SolutionPoint> candidatePoints(1);

    candidatePoints.at(0) = solutionCandidate;

    auto threadId = std::to_string(this->getMyThreadNum());

    env->results->getCurrentIteration()->numberOfOpenNodes = this->getNremainingNodes();

    env->solutionStatistics.numberOfExploredNodes
        = std::max((int)this->getNnodes(), env->solutionStatistics.numberOfExploredNodes);

    printIterationReport(candidatePoints.at(0), threadId);

    if(env->results->isAbsoluteObjectiveGapToleranceMet())
    {
        env->output->outputCritical("     Terminated by absolute objective gap tolerance in lazy callback");

        solution.clear();
        abort();
        return;
    }

    if(env->results->isRelativeObjectiveGapToleranceMet())
    {
        env->output->outputCritical("     Terminated by relative objective gap tolerance in lazy callback");

        solution.clear();
        abort();
        return;
    }

    if(checkIterationLimit())
    {
        env->output->outputCritical("     Terminated by iteration limit in lazy callback");

        solution.clear();
        abort();
        return;
    }

    if(checkUserTermination())
    {
        env->output->outputCritical("     Terminated by user in lazy callback");

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
        env->primalSolver->addFixedNLPCandidate(solution, E_PrimalNLPSource::FirstSolution, this->getObjValue(),
            env->results->getCurrentIteration()->iterationNumber, solutionCandidate.maxDeviation);

        tSelectPrimNLP.get()->run();

        env->primalSolver->checkPrimalSolutionCandidates();

        if(env->results->isAbsoluteObjectiveGapToleranceMet())
        {
            env->output->outputCritical("     Terminated by absolute objective gap tolerance in lazy callback");

            solution.clear();
            abort();
            return;
        }

        if(env->results->isRelativeObjectiveGapToleranceMet())
        {
            env->output->outputCritical("     Terminated by relative objective gap tolerance in lazy callback");

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
            static_cast<TaskSelectHyperplanePointsESH*>(taskSelectHPPts.get())->run(candidatePoints);
        }
        else
        {
            static_cast<TaskSelectHyperplanePointsECP*>(taskSelectHPPts.get())->run(candidatePoints);
        }
    }

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        taskSelectHPPtsByObjectiveRootsearch->run(candidatePoints);
    }

    for(auto& hp : env->dualSolver->hyperplaneWaitingList)
    {
        this->createHyperplane(hp);
        this->lastNumAddedHyperplanes++;
    }

    env->dualSolver->hyperplaneWaitingList.clear();

    if(env->settings->getSetting<bool>("HyperplaneCuts.UseIntegerCuts", "Dual"))
    {
        bool addedIntegerCut = false;

        for(auto& ic : env->dualSolver->integerCutWaitingList)
        {
            this->createIntegerCut(ic.first, ic.second);
            addedIntegerCut = true;
        }

        if(addedIntegerCut)
        {
            env->output->outputDebug("        Added " + std::to_string(env->dualSolver->integerCutWaitingList.size())
                + " integer cut(s).                                        ");
        }

        env->dualSolver->integerCutWaitingList.clear();
    }

    candidatePoints.clear();
    solution.clear();
}

bool CtCallbackI::createHyperplane(Hyperplane hyperplane)
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
            env->output->outputError("     Warning: hyperplane not generated, NaN found in linear terms for variable "
                + env->problem->getVariable(E.first)->name);
            return (false);
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
            && hyperplane.source == E_HyperplaneSource::MIPCallbackRelaxed)
        {
            addLocal(tmpRange).end();
        }
        else
        {
            add(tmpRange, IloCplex::CutManagement::UseCutPurge).end();
        }

        std::string identifier = env->dualSolver->MIPSolver->getConstraintIdentifier(hyperplane.source);

        if(hyperplane.sourceConstraint != nullptr)
            identifier = identifier + "_" + hyperplane.sourceConstraint->name;

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

bool CtCallbackI::createIntegerCut(VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes)
{
    try
    {
        IloExpr expr(this->getEnv());

        for(int I : binaryIndexesOnes)
        {
            expr += 1.0 * cplexVars[I];
        }

        for(int I : binaryIndexesZeroes)
        {
            expr += (1 - 1.0 * cplexVars[I]);
        }

        IloRange tmpRange(
            this->getEnv(), -IloInfinity, expr, binaryIndexesOnes.size() + binaryIndexesZeroes.size() - 1.0);
        tmpRange.setName("IC");

        add(tmpRange);
        env->solutionStatistics.numberOfIntegerCuts++;

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

MIPSolverCplexLazyOriginalCallback::MIPSolverCplexLazyOriginalCallback(EnvironmentPtr envPtr)
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

MIPSolverCplexLazyOriginalCallback::~MIPSolverCplexLazyOriginalCallback() = default;

void MIPSolverCplexLazyOriginalCallback::initializeSolverSettings()
{
    try
    {
        MIPSolverCplex::initializeSolverSettings();
    }
    catch(IloException& e)
    {
        env->output->outputError("Cplex error when initializing parameters for linear solver", e.getMessage());
    }
}

E_ProblemSolutionStatus MIPSolverCplexLazyOriginalCallback::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    MIPSolverCplex::cachedSolutionHasChanged = true;

    try
    {
        if(modelUpdated)
        {
            // Extract the model if we have updated the constraints
            cplexInstance.extract(cplexModel);
        }

        if(getDiscreteVariableStatus())
        {
            cplexInstance.use(CtCallback(env, cplexEnv, cplexVars));
            cplexInstance.use(HCallback(env, cplexEnv, cplexVars));
            cplexInstance.use(InfoCallback(env, cplexEnv, cplexVars));
        }

        // Fixes a deadlock bug in Cplex 12.7 and 12.8
        cplexEnv.setNormalizer(false);

        cplexInstance.solve();

        MIPSolutionStatus = MIPSolverCplex::getSolutionStatus();

        if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded)
        {
            repairInfeasibility();
            MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
            env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed = true;
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

int MIPSolverCplexLazyOriginalCallback::increaseSolutionLimit(int increment)
{
    int sollim = 0;

    try
    {
        cplexInstance.setParam(IloCplex::IntSolLim, cplexInstance.getParam(cplexInstance.IntSolLim) + increment);
        sollim = cplexInstance.getParam(cplexInstance.IntSolLim);
    }
    catch(IloException& e)
    {
        env->output->outputError("Error when increasing solution limit", e.getMessage());
    }

    return (sollim);
}

void MIPSolverCplexLazyOriginalCallback::setSolutionLimit(long limit)
{
    try
    {
        cplexInstance.setParam(IloCplex::IntSolLim, limit);
    }
    catch(IloException& e)
    {
        env->output->outputError("Error when setting solution limit", e.getMessage());
    }
}

int MIPSolverCplexLazyOriginalCallback::getSolutionLimit()
{
    int solLim = 0;

    try
    {
        solLim = cplexInstance.getParam(cplexInstance.IntSolLim);
    }
    catch(IloException& e)
    {

        env->output->outputError("Error when obtaining solution limit", e.getMessage());
    }

    return (solLim);
}

void MIPSolverCplexLazyOriginalCallback::checkParameters() {}
} // namespace SHOT