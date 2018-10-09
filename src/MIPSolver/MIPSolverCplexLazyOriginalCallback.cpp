/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplexLazyOriginalCallback.h"

HCallbackI::HCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2) : IloCplex::HeuristicCallbackI(iloEnv), cplexVars(xx2)
{
    env = envPtr;

    std::lock_guard<std::mutex> lock((static_cast<MIPSolverCplexLazyOriginalCallback *>(env->dualSolver.get()))->callbackMutex2);

    if (static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
    {
        if (static_cast<ES_RootsearchConstraintStrategy>(env->settings->getIntSetting(
                "ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
        {
            taskSelectHPPts = std::shared_ptr<TaskBase>(new TaskSelectHyperplanePointsLinesearch(env));
        }
        else
        {
            taskSelectHPPts = std::shared_ptr<TaskBase>(new TaskSelectHyperplanePointsIndividualLinesearch(env));
        }
    }
    else
    {
        taskSelectHPPts = std::shared_ptr<TaskBase>(new TaskSelectHyperplanePointsSolution(env));
    }
}

HCallbackI::~HCallbackI()
{
}

IloCplex::CallbackI *HCallbackI::duplicateCallback() const
{
    return (new (getEnv()) HCallbackI(*this));
}

IloCplex::Callback HCallback(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray cplexVars)
{
    return (IloCplex::Callback(new (iloEnv) HCallbackI(envPtr, iloEnv, cplexVars)));
}

// This callback injects the best known primal solution into all threads.
void HCallbackI::main() // Called at each node...
{
    std::lock_guard<std::mutex> lock((static_cast<MIPSolverCplexLazyOriginalCallback *>(env->dualSolver.get()))->callbackMutex2);

    bool isMinimization = env->model->originalProblem->isTypeOfObjectiveMinimize();

    if ((env->process->primalSolutions.size() > 0) && ((isMinimization && this->getIncumbentObjValue() > env->process->getPrimalBound()) || (!isMinimization && this->getIncumbentObjValue() < env->process->getPrimalBound())))
    {
        auto primalSol = env->process->primalSolution;

        IloNumArray tmpVals(this->getEnv());

        for (int i = 0; i < primalSol.size(); i++)
        {
            tmpVals.add(primalSol.at(i));
        }

        setSolution(cplexVars, tmpVals);

        tmpVals.end();
    }

    if (env->process->getCurrentIteration()->relaxedLazyHyperplanesAdded < env->settings->getIntSetting("Relaxation.MaxLazyConstraints", "Dual"))
    {
        int waitingListSize = env->process->hyperplaneWaitingList.size();

        std::vector<SolutionPoint> solutionPoints(1);

        IloNumArray tmpVals(getEnv());

        getValues(tmpVals, cplexVars);

        VectorDouble solution(tmpVals.getSize());

        for (int i = 0; i < tmpVals.getSize(); i++)
        {
            solution.at(i) = tmpVals[i];
        }

        tmpVals.end();

        auto mostDevConstr = env->model->originalProblem->getMostDeviatingConstraint(solution);

        SolutionPoint tmpSolPt;

        tmpSolPt.point = solution;
        tmpSolPt.objectiveValue = getObjValue();
        tmpSolPt.iterFound = 0;
        tmpSolPt.maxDeviation = mostDevConstr;
        tmpSolPt.isRelaxedPoint = true;

        solutionPoints.at(0) = tmpSolPt;

        if (static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
        {
            if (static_cast<ES_RootsearchConstraintStrategy>(env->settings->getIntSetting(
                    "ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
            {
                static_cast<TaskSelectHyperplanePointsLinesearch *>(taskSelectHPPts.get())->run(solutionPoints);
            }
            else
            {
                static_cast<TaskSelectHyperplanePointsIndividualLinesearch *>(taskSelectHPPts.get())->run(solutionPoints);
            }
        }
        else
        {
            static_cast<TaskSelectHyperplanePointsSolution *>(taskSelectHPPts.get())->run(solutionPoints);
        }

        env->process->getCurrentIteration()->relaxedLazyHyperplanesAdded += (env->process->hyperplaneWaitingList.size() - waitingListSize);
    }

    return;
}

InfoCallbackI::InfoCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2) : IloCplex::MIPInfoCallbackI(iloEnv), cplexVars(xx2)
{
    env = envPtr;
}

InfoCallbackI::~InfoCallbackI()
{
}

IloCplex::CallbackI *InfoCallbackI::duplicateCallback() const
{
    return (new (getEnv()) InfoCallbackI(*this));
}

IloCplex::Callback InfoCallback(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray cplexVars)
{
    return (IloCplex::Callback(new (iloEnv) InfoCallbackI(envPtr, iloEnv, cplexVars)));
}

void InfoCallbackI::main() // Called at each node...
{
    std::lock_guard<std::mutex> lock((static_cast<MIPSolverCplexLazyOriginalCallback *>(env->dualSolver.get()))->callbackMutex2);

    bool isMinimization = env->model->originalProblem->isTypeOfObjectiveMinimize();

    auto absObjGap = env->process->getAbsoluteObjectiveGap();
    auto relObjGap = env->process->getRelativeObjectiveGap();

    if (env->process->isRelativeObjectiveGapToleranceMet())
    {
        env->output->outputAlways(
            "     Terminated by relative objective gap tolerance in info callback: " + UtilityFunctions::toString(relObjGap) + " < " + UtilityFunctions::toString(env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination")));

        this->abort();
        return;
    }
    else if (env->process->isAbsoluteObjectiveGapToleranceMet())
    {
        env->output->outputAlways(
            "     Terminated by absolute objective gap tolerance in info callback: " + UtilityFunctions::toString(absObjGap) + " < " + UtilityFunctions::toString(env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination")));

        this->abort();
        return;
    }
    else if (checkIterationLimit())
    {
        env->output->outputAlways("     Terminated since iteration limit reached in info callback.");

        this->abort();
        return;
    }

    return;
}

CtCallbackI::CtCallbackI(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray xx2) : IloCplex::LazyConstraintCallbackI(iloEnv), cplexVars(xx2)
{
    env = envPtr;

    std::lock_guard<std::mutex> lock((static_cast<MIPSolverCplexLazyOriginalCallback *>(env->dualSolver.get()))->callbackMutex2);

    env->solutionStatistics.iterationLastLazyAdded = 0;
    isMinimization = env->model->originalProblem->isTypeOfObjectiveMinimize();
    cbCalls = 0;

    if (static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
    {
        tUpdateInteriorPoint = std::shared_ptr<TaskUpdateInteriorPoint>(new TaskUpdateInteriorPoint(env));

        if (static_cast<ES_RootsearchConstraintStrategy>(env->settings->getIntSetting("ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
        {
            taskSelectHPPts = std::shared_ptr<TaskSelectHyperplanePointsLinesearch>(new TaskSelectHyperplanePointsLinesearch(env));
        }
        else
        {
            taskSelectHPPts = std::shared_ptr<TaskSelectHyperplanePointsIndividualLinesearch>(new TaskSelectHyperplanePointsIndividualLinesearch(env));
        }
    }
    else
    {
        taskSelectHPPts = std::shared_ptr<TaskSelectHyperplanePointsSolution>(new TaskSelectHyperplanePointsSolution(env));
    }

    tSelectPrimNLP = std::shared_ptr<TaskSelectPrimalCandidatesFromNLP>(new TaskSelectPrimalCandidatesFromNLP(env));

    if (env->model->originalProblem->isObjectiveFunctionNonlinear() && env->settings->getBoolSetting("ObjectiveLinesearch.Use", "Dual"))
    {
        taskUpdateObjectiveByLinesearch = std::shared_ptr<TaskUpdateNonlinearObjectiveByLinesearch>(new TaskUpdateNonlinearObjectiveByLinesearch(env));
    }

    if (env->settings->getBoolSetting("Linesearch.Use", "Primal"))
    {
        taskSelectPrimalSolutionFromLinesearch = std::shared_ptr<TaskSelectPrimalCandidatesFromLinesearch>(new TaskSelectPrimalCandidatesFromLinesearch(env));
    }
}

CtCallbackI::~CtCallbackI()
{
}

IloCplex::CallbackI *CtCallbackI::duplicateCallback() const
{
    return (new (getEnv()) CtCallbackI(*this));
}

IloCplex::Callback CtCallback(EnvironmentPtr envPtr, IloEnv iloEnv, IloNumVarArray cplexVars)
{
    return (IloCplex::Callback(new (iloEnv) CtCallbackI(envPtr, iloEnv, cplexVars)));
}

void CtCallbackI::main()
{
    auto currIter = env->process->getCurrentIteration();

    if (currIter->isSolved)
    {
        env->process->createIteration();
        currIter = env->process->getCurrentIteration();
    }

    currIter->isSolved = true;

    std::lock_guard<std::mutex> lock((static_cast<MIPSolverCplexLazyOriginalCallback *>(env->dualSolver.get()))->callbackMutex2);

    this->cbCalls++;

    IloNumArray tmpVals(this->getEnv());

    this->getValues(tmpVals, cplexVars);

    VectorDouble solution(tmpVals.getSize());

    for (int i = 0; i < tmpVals.getSize(); i++)
    {
        solution.at(i) = tmpVals[i];
    }

    tmpVals.end();

    auto mostDevConstr = env->model->originalProblem->getMostDeviatingConstraint(solution);

    double tmpDualObjBound = this->getBestObjValue();

    SolutionPoint tmpSolPt;

    tmpSolPt.point = solution;
    tmpSolPt.objectiveValue = getObjValue();
    tmpSolPt.iterFound = env->process->getCurrentIteration()->iterationNumber;
    tmpSolPt.maxDeviation = mostDevConstr;

    // Check if better dual bound
    if ((isMinimization && tmpDualObjBound > env->process->getDualBound()) || (!isMinimization && tmpDualObjBound < env->process->getDualBound()))
    {
        DualSolution sol =
            {solution, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound, env->process->getCurrentIteration()->iterationNumber};
        env->process->addDualSolutionCandidate(sol);
    }

    // Check if better primal solution
    if (this->hasIncumbent())
    {
        double tmpPrimalObjBound = this->getIncumbentObjValue();

        if ((isMinimization && tmpPrimalObjBound < env->process->getPrimalBound()) || (!isMinimization && tmpPrimalObjBound > env->process->getPrimalBound()))
        {
            IloNumArray tmpPrimalVals(this->getEnv());

            this->getIncumbentValues(tmpPrimalVals, cplexVars);

            VectorDouble primalSolution(tmpPrimalVals.getSize());

            for (int i = 0; i < tmpPrimalVals.getSize(); i++)
            {
                primalSolution.at(i) = tmpPrimalVals[i];
            }

            SolutionPoint tmpPt;
            tmpPt.iterFound = env->process->getCurrentIteration()->iterationNumber;
            tmpPt.maxDeviation = env->model->originalProblem->getMostDeviatingConstraint(primalSolution);
            tmpPt.objectiveValue = this->getIncumbentObjValue();
            tmpPt.point = primalSolution;

            env->process->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::LazyConstraintCallback);

            tmpPrimalVals.end();
            primalSolution.clear();
        }
    }

    std::vector<SolutionPoint> candidatePoints(1);

    candidatePoints.at(0) = tmpSolPt;

    auto threadId = std::to_string(this->getMyThreadNum());

    env->process->getCurrentIteration()->numberOfOpenNodes = this->getNremainingNodes();

    env->solutionStatistics.numberOfExploredNodes = std::max((int)this->getNnodes(), env->solutionStatistics.numberOfExploredNodes);

    printIterationReport(candidatePoints.at(0), threadId);

    if (env->process->isAbsoluteObjectiveGapToleranceMet() || env->process->isRelativeObjectiveGapToleranceMet() || checkIterationLimit())
    {
        solution.clear();
        abort();
        return;
    }

    if (env->settings->getBoolSetting("Linesearch.Use", "Primal"))
    {
        taskSelectPrimalSolutionFromLinesearch->run(candidatePoints);
    }

    if (checkFixedNLPStrategy(tmpSolPt))
    {
        env->process->addPrimalFixedNLPCandidate(solution, E_PrimalNLPSource::FirstSolution,
                                                 this->getObjValue(), env->process->getCurrentIteration()->iterationNumber,
                                                 tmpSolPt.maxDeviation);

        tSelectPrimNLP.get()->run();

        env->process->checkPrimalSolutionCandidates();

        if (env->process->isAbsoluteObjectiveGapToleranceMet() || env->process->isRelativeObjectiveGapToleranceMet())
        {
            solution.clear();
            abort();
            return;
        }
    }

    if (static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
    {
        tUpdateInteriorPoint->run();

        if (static_cast<ES_RootsearchConstraintStrategy>(env->settings->getIntSetting(
                "ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
        {
            static_cast<TaskSelectHyperplanePointsLinesearch *>(taskSelectHPPts.get())->run(candidatePoints);
        }
        else
        {
            static_cast<TaskSelectHyperplanePointsIndividualLinesearch *>(taskSelectHPPts.get())->run(candidatePoints);
        }
    }
    else
    {
        static_cast<TaskSelectHyperplanePointsSolution *>(taskSelectHPPts.get())->run(candidatePoints);
    }

    if (env->model->originalProblem->isObjectiveFunctionNonlinear() && env->settings->getBoolSetting("ObjectiveLinesearch.Use", "Dual"))
    {
        taskUpdateObjectiveByLinesearch->updateObjectiveInPoint(candidatePoints.at(0));
    }

    for (auto hp : env->process->hyperplaneWaitingList)
    {
        this->createHyperplane(hp);

        this->lastNumAddedHyperplanes++;
    }

    env->process->hyperplaneWaitingList.clear();

    if (env->settings->getBoolSetting("HyperplaneCuts.UseIntegerCuts", "Dual"))
    {
        bool addedIntegerCut = false;

        for (auto ic : env->process->integerCutWaitingList)
        {
            this->createIntegerCut(ic);
            addedIntegerCut = true;
        }

        if (addedIntegerCut)
        {
            env->output->outputInfo(
                "     Added " + std::to_string(env->process->integerCutWaitingList.size()) + " integer cut(s).                                        ");
        }

        env->process->integerCutWaitingList.clear();
    }

    candidatePoints.clear();
    solution.clear();
}

void CtCallbackI::createHyperplane(Hyperplane hyperplane)
{
    auto optional = env->dualSolver->createHyperplaneTerms(hyperplane);
    if (!optional)
    {
        return;
    }

    auto tmpPair = optional.get();

    bool hyperplaneIsOk = true;
    for (auto E : tmpPair.first)
    {
        if (E.value != E.value) //Check for NaN
        {
            env->output->outputWarning(
                "     Warning: hyperplane not generated, NaN found in linear terms!");
            hyperplaneIsOk = false;
            break;
        }
    }

    auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration

    if (hyperplaneIsOk)
    {
        //GeneratedHyperplane genHyperplane;

        IloExpr expr(this->getEnv());

        for (int i = 0; i < tmpPair.first.size(); i++)
        {
            expr += tmpPair.first.at(i).value * cplexVars[tmpPair.first.at(i).index];
        }

        IloRange tmpRange(this->getEnv(), -IloInfinity, expr, -tmpPair.second);

        tmpPair.first.clear();
        expr.end();

        if (env->settings->getBoolSetting("Cplex.AddRelaxedLazyConstraintsAsLocal", "Subsolver") && hyperplane.source == E_HyperplaneSource::MIPCallbackRelaxed)
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

        //env->dualSolver->generatedHyperplanes.push_back(genHyperplane);

        currIter->numHyperplanesAdded++;
        currIter->totNumHyperplanes++;
    }

    optional.get().first.clear();
}

void CtCallbackI::createIntegerCut(VectorInteger binaryIndexes)
{
    IloExpr expr(this->getEnv());

    for (int i = 0; i < binaryIndexes.size(); i++)
    {
        expr += 1.0 * cplexVars[binaryIndexes.at(i)];
    }

    IloRange tmpRange(this->getEnv(), -IloInfinity, expr, binaryIndexes.size() - 1.0);

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

MIPSolverCplexLazyOriginalCallback::~MIPSolverCplexLazyOriginalCallback()
{
    cplexLazyConstrs.end();
}

void MIPSolverCplexLazyOriginalCallback::initializeSolverSettings()
{
    try
    {
        MIPSolverCplex::initializeSolverSettings();

        cplexInstance.use(CtCallback(env, cplexEnv, cplexVars));
        cplexInstance.use(HCallback(env, cplexEnv, cplexVars));
        cplexInstance.use(InfoCallback(env, cplexEnv, cplexVars));
    }
    catch (IloException &e)
    {
        env->output->outputError("Cplex error when initializing parameters for linear solver",
                                 e.getMessage());
    }
}

E_ProblemSolutionStatus MIPSolverCplexLazyOriginalCallback::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    MIPSolverCplex::cachedSolutionHasChanged = true;

    try
    {
        if (modelUpdated)
        {
            //Extract the model if we have updated the constraints
            cplexInstance.extract(cplexModel);
        }

        // Fixes a deadlock bug in Cplex 12.7 and 12.8
        cplexEnv.setNormalizer(false);

        cplexInstance.solve();

        MIPSolutionStatus = MIPSolverCplex::getSolutionStatus();
    }
    catch (IloException &e)
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
    catch (IloException &e)
    {
        env->output->outputError("Error when increasing solution limit", e.getMessage());
    }

    return (sollim);
}

void MIPSolverCplexLazyOriginalCallback::setSolutionLimit(long limit)
{
    if (MIPSolverBase::originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
    {
        limit = env->settings->getIntSetting("MIP.SolutionLimit.Initial", "Dual");
    }

    try
    {
        cplexInstance.setParam(IloCplex::IntSolLim, limit);
    }
    catch (IloException &e)
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
    catch (IloException &e)
    {

        env->output->outputError("Error when obtaining solution limit", e.getMessage());
    }

    return (solLim);
}

void MIPSolverCplexLazyOriginalCallback::checkParameters()
{
}
