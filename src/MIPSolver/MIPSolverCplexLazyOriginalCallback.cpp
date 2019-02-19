/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplexLazyOriginalCallback.h"

namespace SHOT
{

HCallbackI::HCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2)
    : IloCplex::HeuristicCallbackI(iloEnv), cplexVars(xx2)
{
    env = envPtr;

    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexLazyOriginalCallback*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ESH)
    {
        taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsESH>(env);

        if(env->reformulatedProblem->objectiveFunction->properties.hasNonlinearExpression)
        {
            taskSelectHPPtsByObjectiveLinesearch
                = std::make_shared<TaskSelectHyperplanePointsByObjectiveLinesearch>(env);
        }
    }
    else
    {
        taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsECP>(env);
    }
}

HCallbackI::~HCallbackI() {}

IloCplex::CallbackI* HCallbackI::duplicateCallback() const { return (new(getEnv()) HCallbackI(*this)); }

IloCplex::Callback HCallback(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray cplexVars)
{
    return (IloCplex::Callback(new(iloEnv) HCallbackI(envPtr, iloEnv, cplexVars)));
}

// This callback injects the best known primal solution into all threads.
void HCallbackI::main() // Called at each node...
{
    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexLazyOriginalCallback*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    bool isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;

    if((env->results->primalSolutions.size() > 0)
        && ((isMinimization && this->getIncumbentObjValue() > env->results->getPrimalBound())
               || (!isMinimization && this->getIncumbentObjValue() < env->results->getPrimalBound())))
    {
        auto primalSol = env->results->primalSolution;

        IloNumArray tmpVals(this->getEnv());

        for(int i = 0; i < primalSol.size(); i++)
        {
            tmpVals.add(primalSol.at(i));
        }

        for(auto& V : env->reformulatedProblem->auxilliaryVariables)
        {
            // std::cout << "calculated value for auxilliary variable " << V->name << " " <<
            // V->calculateValue(primalSol) << '\n';
            tmpVals.add(V->calculateValue(primalSol));
        }

        try
        {
            // setSolution(cplexVars, tmpVals);
        }
        catch(IloException& e)
        {
            env->output->outputError(
                "Error when setting primal solution as starting point in heuristic callback:", e.getMessage());
        }

        tmpVals.end();
    }

    if(env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
        < env->settings->getIntSetting("Relaxation.MaxLazyConstraints", "Dual"))
    {
        int waitingListSize = env->dualSolver->MIPSolver->hyperplaneWaitingList.size();

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

        tmpSolPt.point = solution;
        tmpSolPt.objectiveValue = getObjValue();
        tmpSolPt.iterFound = 0;
        tmpSolPt.isRelaxedPoint = true;

        solutionPoints.at(0) = tmpSolPt;

        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            static_cast<TaskSelectHyperplanePointsESH*>(taskSelectHPPts.get())->run(solutionPoints);
        }
        else
        {
            static_cast<TaskSelectHyperplanePointsECP*>(taskSelectHPPts.get())->run(solutionPoints);
        }

        env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
            += (env->dualSolver->MIPSolver->hyperplaneWaitingList.size() - waitingListSize);
    }

    return;
}

InfoCallbackI::InfoCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2)
    : IloCplex::MIPInfoCallbackI(iloEnv), cplexVars(xx2)
{
    env = envPtr;
}

InfoCallbackI::~InfoCallbackI() {}

IloCplex::CallbackI* InfoCallbackI::duplicateCallback() const { return (new(getEnv()) InfoCallbackI(*this)); }

IloCplex::Callback InfoCallback(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray cplexVars)
{
    return (IloCplex::Callback(new(iloEnv) InfoCallbackI(envPtr, iloEnv, cplexVars)));
}

void InfoCallbackI::main() // Called at each node...
{
    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexLazyOriginalCallback*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    bool isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;

    auto absObjGap = env->results->getAbsoluteObjectiveGap();
    auto relObjGap = env->results->getRelativeObjectiveGap();

    if(env->results->isRelativeObjectiveGapToleranceMet())
    {
        env->output->outputCritical("     Terminated by relative objective gap tolerance in info callback: "
            + UtilityFunctions::toString(relObjGap) + " < "
            + UtilityFunctions::toString(env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination")));

        this->abort();
        return;
    }
    else if(env->results->isAbsoluteObjectiveGapToleranceMet())
    {
        env->output->outputCritical("     Terminated by absolute objective gap tolerance in info callback: "
            + UtilityFunctions::toString(absObjGap) + " < "
            + UtilityFunctions::toString(env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination")));

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

    if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ESH)
    {
        tUpdateInteriorPoint = std::make_shared<TaskUpdateInteriorPoint>(env);

        taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsESH>(env);
    }
    else
    {
        taskSelectHPPts = std::make_shared<TaskSelectHyperplanePointsECP>(env);
    }

    tSelectPrimNLP = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env);

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        taskSelectHPPtsByObjectiveLinesearch = std::make_shared<TaskSelectHyperplanePointsByObjectiveLinesearch>(env);
    }

    if(env->settings->getBoolSetting("Linesearch.Use", "Primal")
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        taskSelectPrimalSolutionFromLinesearch = std::make_shared<TaskSelectPrimalCandidatesFromLinesearch>(env);
    }
}

CtCallbackI::~CtCallbackI() {}

IloCplex::CallbackI* CtCallbackI::duplicateCallback() const { return (new(getEnv()) CtCallbackI(*this)); }

IloCplex::Callback CtCallback(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray cplexVars)
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
    }

    currIter->isSolved = true;

    std::lock_guard<std::mutex> lock(
        (static_cast<MIPSolverCplexLazyOriginalCallback*>(env->dualSolver->MIPSolver.get()))->callbackMutex2);

    this->cbCalls++;

    IloNumArray tmpVals(this->getEnv());

    this->getValues(tmpVals, cplexVars);

    int size
        = (env->dualSolver->MIPSolver->hasAuxilliaryObjectiveVariable()) ? tmpVals.getSize() - 1 : tmpVals.getSize();

    VectorDouble solution(size);

    for(int i = 0; i < size; i++)
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

    double tmpDualObjBound = this->getBestObjValue();

    tmpSolPt.point = solution;
    tmpSolPt.objectiveValue = getObjValue();
    tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;

    // Check if better dual bound
    if((isMinimization && tmpDualObjBound > env->results->getDualBound())
        || (!isMinimization && tmpDualObjBound < env->results->getDualBound()))
    {
        DualSolution sol = { solution, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound,
            env->results->getCurrentIteration()->iterationNumber };
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

            tmpPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
            tmpPt.objectiveValue = this->getIncumbentObjValue();
            tmpPt.point = primalSolution;
            env->primalSolver->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::LazyConstraintCallback);
            tmpPrimalVals.end();
            primalSolution.clear();
        }
    }

    std::vector<SolutionPoint> candidatePoints(1);

    candidatePoints.at(0) = tmpSolPt;

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

    if(env->settings->getBoolSetting("Linesearch.Use", "Primal")
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        taskSelectPrimalSolutionFromLinesearch->run(candidatePoints);
    }

    if(checkFixedNLPStrategy(tmpSolPt))
    {
        env->primalSolver->addFixedNLPCandidate(solution, E_PrimalNLPSource::FirstSolution, this->getObjValue(),
            env->results->getCurrentIteration()->iterationNumber, tmpSolPt.maxDeviation);

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

    if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual"))
        == ES_HyperplaneCutStrategy::ESH)
    {
        tUpdateInteriorPoint->run();

        static_cast<TaskSelectHyperplanePointsESH*>(taskSelectHPPts.get())->run(candidatePoints);
    }
    else
    {
        static_cast<TaskSelectHyperplanePointsECP*>(taskSelectHPPts.get())->run(candidatePoints);
    }

    if(env->reformulatedProblem->objectiveFunction->properties.hasNonlinearExpression)
    {
        taskSelectHPPtsByObjectiveLinesearch->run(candidatePoints);
    }

    for(auto& hp : env->dualSolver->MIPSolver->hyperplaneWaitingList)
    {
        this->createHyperplane(hp);

        this->lastNumAddedHyperplanes++;
    }

    env->dualSolver->MIPSolver->hyperplaneWaitingList.clear();

    if(env->settings->getBoolSetting("HyperplaneCuts.UseIntegerCuts", "Dual"))
    {
        bool addedIntegerCut = false;

        for(auto& ic : env->dualSolver->MIPSolver->integerCutWaitingList)
        {
            this->createIntegerCut(ic.first, ic.second);
            addedIntegerCut = true;
        }

        if(addedIntegerCut)
        {
            env->output->outputDebug("        Added "
                + std::to_string(env->dualSolver->MIPSolver->integerCutWaitingList.size())
                + " integer cut(s).                                        ");
        }

        env->dualSolver->MIPSolver->integerCutWaitingList.clear();
    }

    candidatePoints.clear();
    solution.clear();
}

void CtCallbackI::createHyperplane(Hyperplane hyperplane)
{
    auto optional = env->dualSolver->MIPSolver->createHyperplaneTerms(hyperplane);
    
    if(!optional)
    {
        return;
    }

    auto tmpPair = optional.value();

    bool hyperplaneIsOk = true;
    for(auto& E : tmpPair.first)
    {
        if(E.value != E.value) // Check for NaN
        {
            env->output->outputError("     Warning: hyperplane not generated, NaN found in linear terms for variable "
                + env->problem->getVariable(E.index)->name);
            hyperplaneIsOk = false;
            break;
        }
    }

    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration

    if(hyperplaneIsOk)
    {
        // GeneratedHyperplane genHyperplane;

        IloExpr expr(this->getEnv());

        for(int i = 0; i < tmpPair.first.size(); i++)
        {
            expr += tmpPair.first.at(i).value * cplexVars[tmpPair.first.at(i).index];
        }

        IloRange tmpRange(this->getEnv(), -IloInfinity, expr, -tmpPair.second);

        tmpPair.first.clear();
        expr.end();

        if(env->settings->getBoolSetting("Cplex.AddRelaxedLazyConstraintsAsLocal", "Subsolver")
            && hyperplane.source == E_HyperplaneSource::MIPCallbackRelaxed)
        {
            addLocal(tmpRange).end();
        }
        else
        {
            add(tmpRange, IloCplex::CutManagement::UseCutPurge).end();
        }

        // int constrIndex = 0;
        /*genHyperplane.generatedConstraintIndex = constrIndex;
                genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
                genHyperplane.generatedPoint = hyperplane.generatedPoint;
                genHyperplane.source = hyperplane.source;
                genHyperplane.generatedIter = currIter->iterationNumber;
                genHyperplane.isLazy = false;
                genHyperplane.isRemoved = false;*/

        // env->dualSolver->MIPSolver->generatedHyperplanes.push_back(genHyperplane);

        currIter->numHyperplanesAdded++;
        currIter->totNumHyperplanes++;
    }

    optional.value().first.clear();
}

void CtCallbackI::createIntegerCut(VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes)
{
    IloExpr expr(this->getEnv());

    for(int i = 0; i < binaryIndexesOnes.size(); i++)
    {
        expr += 1.0 * cplexVars[binaryIndexesOnes.at(i)];
    }

    for(int i = 0; i < binaryIndexesZeroes.size(); i++)
    {
        expr += (1 - 1.0 * cplexVars[binaryIndexesZeroes.at(i)]);
    }

    IloRange tmpRange(this->getEnv(), -IloInfinity, expr, binaryIndexesOnes.size() + binaryIndexesZeroes.size() - 1.0);
    tmpRange.setName("IC");

    add(tmpRange);
    env->solutionStatistics.numberOfIntegerCuts++;

    tmpRange.end();
    expr.end();
}

MIPSolverCplexLazyOriginalCallback::MIPSolverCplexLazyOriginalCallback(EnvironmentPtr envPtr)
{
    env = envPtr;

    discreteVariablesActivated = true;

    cplexModel = IloModel(cplexEnv);

    cplexVars = IloNumVarArray(cplexEnv);
    cplexConstrs = IloRangeArray(cplexEnv);
    cplexLazyConstrs = IloRangeArray(cplexEnv);

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();
    modelUpdated = false;
}

MIPSolverCplexLazyOriginalCallback::~MIPSolverCplexLazyOriginalCallback() { cplexLazyConstrs.end(); }

void MIPSolverCplexLazyOriginalCallback::initializeSolverSettings()
{
    try
    {
        MIPSolverCplex::initializeSolverSettings();

        cplexInstance.use(CtCallback(env, cplexEnv, cplexVars));
        cplexInstance.use(HCallback(env, cplexEnv, cplexVars));
        cplexInstance.use(InfoCallback(env, cplexEnv, cplexVars));
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

        // Fixes a deadlock bug in Cplex 12.7 and 12.8
        cplexEnv.setNormalizer(false);

        cplexInstance.solve();

        MIPSolutionStatus = MIPSolverCplex::getSolutionStatus();
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
    int sollim;

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