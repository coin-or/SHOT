/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplexLazyOriginalCallback.h"

HCallbackI::HCallbackI(IloEnv env, IloNumVarArray xx2) : IloCplex::HeuristicCallbackI(env), cplexVars(xx2)
{
    std::lock_guard<std::mutex> lock((static_cast<MIPSolverCplexLazyOriginalCallback *>(ProcessInfo::getInstance().MIPSolver))->callbackMutex2);

    if (static_cast<ES_HyperplaneCutStrategy>(Settings::getInstance().getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
    {
        if (static_cast<ES_RootsearchConstraintStrategy>(Settings::getInstance().getIntSetting(
                "ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
        {
            taskSelectHPPts = std::shared_ptr<TaskBase>(new TaskSelectHyperplanePointsLinesearch());
        }
        else
        {
            taskSelectHPPts = std::shared_ptr<TaskBase>(new TaskSelectHyperplanePointsIndividualLinesearch());
        }
    }
    else
    {
        taskSelectHPPts = std::shared_ptr<TaskBase>(new TaskSelectHyperplanePointsSolution());
    }
}

HCallbackI::~HCallbackI()
{
}

IloCplex::CallbackI *HCallbackI::duplicateCallback() const
{
    return (new (getEnv()) HCallbackI(*this));
}

IloCplex::Callback HCallback(IloEnv env, IloNumVarArray cplexVars)
{
    return (IloCplex::Callback(new (env) HCallbackI(env, cplexVars)));
}

// This callback injects the best known primal solution into all threads.
void HCallbackI::main() // Called at each node...
{
    std::lock_guard<std::mutex> lock((static_cast<MIPSolverCplexLazyOriginalCallback *>(ProcessInfo::getInstance().MIPSolver))->callbackMutex2);

    bool isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

    if ((ProcessInfo::getInstance().primalSolutions.size() > 0) && ((isMinimization && this->getIncumbentObjValue() > ProcessInfo::getInstance().getPrimalBound()) || (!isMinimization && this->getIncumbentObjValue() < ProcessInfo::getInstance().getPrimalBound())))
    {
        auto primalSol = ProcessInfo::getInstance().primalSolution;

        IloNumArray tmpVals(this->getEnv());

        for (int i = 0; i < primalSol.size(); i++)
        {
            tmpVals.add(primalSol.at(i));
        }

        setSolution(cplexVars, tmpVals);

        tmpVals.end();
    }

    if (ProcessInfo::getInstance().getCurrentIteration()->iterationNumber > iterNumLastResetHyperplaneCounter)
    {
        iterNumLastResetHyperplaneCounter = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
        maxIntegerRelaxedHyperplanes = 0;
    }

    if (maxIntegerRelaxedHyperplanes < Settings::getInstance().getIntSetting("Relaxation.MaxLazyConstraints", "Dual"))
    {
        int waitingListSize = ProcessInfo::getInstance().hyperplaneWaitingList.size();

        std::vector<SolutionPoint> solutionPoints(1);

        IloNumArray tmpVals(getEnv());

        getValues(tmpVals, cplexVars);

        std::vector<double> solution(tmpVals.getSize());

        for (int i = 0; i < tmpVals.getSize(); i++)
        {
            solution.at(i) = tmpVals[i];
        }

        tmpVals.end();

        auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

        SolutionPoint tmpSolPt;

        tmpSolPt.point = solution;
        tmpSolPt.objectiveValue = getObjValue();
        tmpSolPt.iterFound = 0;
        tmpSolPt.maxDeviation = mostDevConstr;

        solutionPoints.at(0) = tmpSolPt;

        if (static_cast<ES_HyperplaneCutStrategy>(Settings::getInstance().getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
        {
            if (static_cast<ES_RootsearchConstraintStrategy>(Settings::getInstance().getIntSetting(
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

        maxIntegerRelaxedHyperplanes += (ProcessInfo::getInstance().hyperplaneWaitingList.size() - waitingListSize);
    }

    return;
}

InfoCallbackI::InfoCallbackI(IloEnv env, IloNumVarArray xx2) : IloCplex::MIPInfoCallbackI(env), cplexVars(xx2)
{
}

InfoCallbackI::~InfoCallbackI()
{
}

IloCplex::CallbackI *InfoCallbackI::duplicateCallback() const
{
    return (new (getEnv()) InfoCallbackI(*this));
}

IloCplex::Callback InfoCallback(IloEnv env, IloNumVarArray cplexVars)
{
    return (IloCplex::Callback(new (env) InfoCallbackI(env, cplexVars)));
}

void InfoCallbackI::main() // Called at each node...
{
    std::lock_guard<std::mutex> lock((static_cast<MIPSolverCplexLazyOriginalCallback *>(ProcessInfo::getInstance().MIPSolver))->callbackMutex2);

    bool isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

    auto absObjGap = ProcessInfo::getInstance().getAbsoluteObjectiveGap();
    auto relObjGap = ProcessInfo::getInstance().getRelativeObjectiveGap();

    if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
    {
        ProcessInfo::getInstance().outputAlways(
            "     Terminated by relative objective gap tolerance in info callback: " + UtilityFunctions::toString(relObjGap) + " < " + UtilityFunctions::toString(Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination")));

        this->abort();
        return;
    }
    else if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
    {
        ProcessInfo::getInstance().outputAlways(
            "     Terminated by absolute objective gap tolerance in info callback: " + UtilityFunctions::toString(absObjGap) + " < " + UtilityFunctions::toString(Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination")));

        this->abort();
        return;
    }
    else if (checkIterationLimit())
    {
        ProcessInfo::getInstance().outputAlways("     Terminated since iteration limit reached in info callback.");

        this->abort();
        return;
    }

    return;
}

CtCallbackI::CtCallbackI(IloEnv env, IloNumVarArray xx2, MIPSolverCplexLazyOriginalCallback *solver) : IloCplex::LazyConstraintCallbackI(env), cplexVars(xx2), cplexSolver(solver)
{
    std::lock_guard<std::mutex> lock((static_cast<MIPSolverCplexLazyOriginalCallback *>(ProcessInfo::getInstance().MIPSolver))->callbackMutex2);

    ProcessInfo::getInstance().lastLazyAddedIter = 0;
    isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();
    cbCalls = 0;

    if (static_cast<ES_HyperplaneCutStrategy>(Settings::getInstance().getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
    {
        tUpdateInteriorPoint = std::shared_ptr<TaskUpdateInteriorPoint>(new TaskUpdateInteriorPoint());

        if (static_cast<ES_RootsearchConstraintStrategy>(Settings::getInstance().getIntSetting("ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
        {
            taskSelectHPPts = std::shared_ptr<TaskSelectHyperplanePointsLinesearch>(new TaskSelectHyperplanePointsLinesearch());
        }
        else
        {
            taskSelectHPPts = std::shared_ptr<TaskSelectHyperplanePointsIndividualLinesearch>(new TaskSelectHyperplanePointsIndividualLinesearch());
        }
    }
    else
    {
        taskSelectHPPts = std::shared_ptr<TaskSelectHyperplanePointsSolution>(new TaskSelectHyperplanePointsSolution());
    }

    tSelectPrimNLP = std::shared_ptr<TaskSelectPrimalCandidatesFromNLP>(new TaskSelectPrimalCandidatesFromNLP());

    if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear() && Settings::getInstance().getBoolSetting("ObjectiveLinesearch.Use", "Dual"))
    {
        taskUpdateObjectiveByLinesearch = std::shared_ptr<TaskUpdateNonlinearObjectiveByLinesearch>(new TaskUpdateNonlinearObjectiveByLinesearch());
    }

    if (Settings::getInstance().getBoolSetting("Linesearch.Use", "Primal"))
    {
        taskSelectPrimalSolutionFromLinesearch = std::shared_ptr<TaskSelectPrimalCandidatesFromLinesearch>(new TaskSelectPrimalCandidatesFromLinesearch());
    }
}

CtCallbackI::~CtCallbackI()
{
}

IloCplex::CallbackI *CtCallbackI::duplicateCallback() const
{
    return (new (getEnv()) CtCallbackI(*this));
}

IloCplex::Callback CtCallback(IloEnv env, IloNumVarArray cplexVars, MIPSolverCplexLazyOriginalCallback *cplexSolver)
{
    return (IloCplex::Callback(new (env) CtCallbackI(env, cplexVars, cplexSolver)));
}

void CtCallbackI::main()
{
    std::lock_guard<std::mutex> lock((static_cast<MIPSolverCplexLazyOriginalCallback *>(ProcessInfo::getInstance().MIPSolver))->callbackMutex2);

    lastNumAddedHyperplanes = 0;
    this->cbCalls++;

    IloNumArray tmpVals(this->getEnv());

    this->getValues(tmpVals, cplexVars);

    std::vector<double> solution(tmpVals.getSize());

    for (int i = 0; i < tmpVals.getSize(); i++)
    {
        solution.at(i) = tmpVals[i];
    }

    tmpVals.end();

    auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(solution);

    double tmpDualObjBound = this->getBestObjValue();

    SolutionPoint tmpSolPt;

    tmpSolPt.point = solution;
    tmpSolPt.objectiveValue = getObjValue();
    tmpSolPt.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
    tmpSolPt.maxDeviation = mostDevConstr;

    // Check if better dual bound
    if ((isMinimization && tmpDualObjBound > ProcessInfo::getInstance().getDualBound()) || (!isMinimization && tmpDualObjBound < ProcessInfo::getInstance().getDualBound()))
    {
        DualSolution sol =
            {solution, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound, ProcessInfo::getInstance().getCurrentIteration()->iterationNumber};
        ProcessInfo::getInstance().addDualSolutionCandidate(sol);
    }

    // Check if better primal solution
    if (this->hasIncumbent())
    {
        double tmpPrimalObjBound = this->getIncumbentObjValue();

        if ((isMinimization && tmpPrimalObjBound < ProcessInfo::getInstance().getPrimalBound()) || (!isMinimization && tmpPrimalObjBound > ProcessInfo::getInstance().getPrimalBound()))
        {
            IloNumArray tmpPrimalVals(this->getEnv());

            this->getIncumbentValues(tmpPrimalVals, cplexVars);

            std::vector<double> primalSolution(tmpPrimalVals.getSize());

            for (int i = 0; i < tmpPrimalVals.getSize(); i++)
            {
                primalSolution.at(i) = tmpPrimalVals[i];
            }

            SolutionPoint tmpPt;
            tmpPt.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
            tmpPt.maxDeviation = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(primalSolution);
            tmpPt.objectiveValue = this->getIncumbentObjValue();
            tmpPt.point = primalSolution;

            ProcessInfo::getInstance().addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::LazyConstraintCallback);

            tmpPrimalVals.end();
            primalSolution.clear();
        }
    }

    if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet() || ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet() || checkIterationLimit())
    {
        solution.clear();
        abort();
        return;
    }

    ProcessInfo::getInstance().createIteration();
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    std::vector<SolutionPoint> candidatePoints(1);

    candidatePoints.at(0) = tmpSolPt;

    if (Settings::getInstance().getBoolSetting("Linesearch.Use", "Primal"))
    {
        taskSelectPrimalSolutionFromLinesearch->run(candidatePoints);
    }

    if (checkFixedNLPStrategy(tmpSolPt))
    {
        ProcessInfo::getInstance().addPrimalFixedNLPCandidate(solution, E_PrimalNLPSource::FirstSolution,
                                                              this->getObjValue(), ProcessInfo::getInstance().getCurrentIteration()->iterationNumber,
                                                              tmpSolPt.maxDeviation);

        tSelectPrimNLP.get()->run();

        ProcessInfo::getInstance()
            .checkPrimalSolutionCandidates();

        if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet() || ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
        {
            solution.clear();
            abort();
            return;
        }
    }

    if (static_cast<ES_HyperplaneCutStrategy>(Settings::getInstance().getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
    {
        tUpdateInteriorPoint->run();

        if (static_cast<ES_RootsearchConstraintStrategy>(Settings::getInstance().getIntSetting(
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

    if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear() && Settings::getInstance().getBoolSetting("ObjectiveLinesearch.Use", "Dual"))
    {
        taskUpdateObjectiveByLinesearch->updateObjectiveInPoint(candidatePoints.at(0));
    }

    for (auto hp : ProcessInfo::getInstance().hyperplaneWaitingList)
    {
        this->createHyperplane(hp);

        this->lastNumAddedHyperplanes++;
    }

    ProcessInfo::getInstance().hyperplaneWaitingList.clear();

    if (Settings::getInstance().getBoolSetting("HyperplaneCuts.UseIntegerCuts", "Dual"))
    {
        bool addedIntegerCut = false;

        for (auto ic : ProcessInfo::getInstance().integerCutWaitingList)
        {
            this->createIntegerCut(ic);
            addedIntegerCut = true;
        }

        if (addedIntegerCut)
        {
            ProcessInfo::getInstance().outputAlways(
                "     Added " + to_string(ProcessInfo::getInstance().integerCutWaitingList.size()) + " integer cut(s).                                        ");
        }

        ProcessInfo::getInstance().integerCutWaitingList.clear();
    }

    auto bestBound = UtilityFunctions::toStringFormat(this->getBestObjValue(), "%.3f", true);
    auto threadId = to_string(this->getMyThreadNum());
    auto openNodes = to_string(this->getNremainingNodes());

    printIterationReport(candidatePoints.at(0), threadId, bestBound, openNodes);

    candidatePoints.clear();
    solution.clear();
}

void CtCallbackI::createHyperplane(Hyperplane hyperplane)
{
    auto optional = ProcessInfo::getInstance().MIPSolver->createHyperplaneTerms(hyperplane);
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
            ProcessInfo::getInstance().outputWarning(
                "     Warning: hyperplane not generated, NaN found in linear terms!");
            hyperplaneIsOk = false;
            break;
        }
    }

    auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

    if (hyperplaneIsOk)
    {
        //GeneratedHyperplane genHyperplane;

        IloExpr expr(this->getEnv());

        for (int i = 0; i < tmpPair.first.size(); i++)
        {
            expr += tmpPair.first.at(i).value * cplexVars[tmpPair.first.at(i).idx];
        }

        IloRange tmpRange(this->getEnv(), -IloInfinity, expr, -tmpPair.second);

        tmpPair.first.clear();
        expr.end();
        add(tmpRange).end();

        // int constrIndex = 0;
        /*genHyperplane.generatedConstraintIndex = constrIndex;
		genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
		genHyperplane.generatedPoint = hyperplane.generatedPoint;
		genHyperplane.source = hyperplane.source;
		genHyperplane.generatedIter = currIter->iterationNumber;
		genHyperplane.isLazy = false;
		genHyperplane.isRemoved = false;*/

        //ProcessInfo::getInstance().MIPSolver->generatedHyperplanes.push_back(genHyperplane);

        currIter->numHyperplanesAdded++;
        currIter->totNumHyperplanes++;
    }

    optional.get().first.clear();
}

void CtCallbackI::createIntegerCut(std::vector<int> binaryIndexes)
{
    IloExpr expr(this->getEnv());

    for (int i = 0; i < binaryIndexes.size(); i++)
    {
        expr += 1.0 * cplexVars[binaryIndexes.at(i)];
    }

    IloRange tmpRange(this->getEnv(), -IloInfinity, expr, binaryIndexes.size() - 1.0);

    add(tmpRange);
    ProcessInfo::getInstance().numIntegerCutsAdded++;

    tmpRange.end();
    expr.end();
}

MIPSolverCplexLazyOriginalCallback::MIPSolverCplexLazyOriginalCallback()
{
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

        if (ProcessInfo::getInstance().originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
        {
            cplexInstance.setParam(IloCplex::Threads, 1);
        }

        cplexInstance.use(CtCallback(cplexEnv, cplexVars, this));
        cplexInstance.use(HCallback(cplexEnv, cplexVars));
        cplexInstance.use(InfoCallback(cplexEnv, cplexVars));
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Cplex error when initializing parameters for linear solver",
                                               e.getMessage());
    }
}

E_ProblemSolutionStatus MIPSolverCplexLazyOriginalCallback::solveProblem()
{
    MIPSolverCplex::startTimer();

    E_ProblemSolutionStatus MIPSolutionStatus;
    MIPSolverCplex::cachedSolutionHasChanged = true;

    try
    {
        if (modelUpdated)
        {
            //Extract the model if we have updated the constraints
            cplexInstance.extract(cplexModel);
        }

        double timeStart = ProcessInfo::getInstance().getElapsedTime("Total");

        cplexInstance.solve();
        double timeEnd = ProcessInfo::getInstance().getElapsedTime("Total");

        iterDurations.push_back(timeEnd - timeStart);
        MIPSolutionStatus = MIPSolverCplex::getSolutionStatus();
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    MIPSolverBase::stopTimer();

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
        ProcessInfo::getInstance().outputError("Error when increasing solution limit", e.getMessage());
    }

    return (sollim);
}

void MIPSolverCplexLazyOriginalCallback::setSolutionLimit(long limit)
{
    if (MIPSolverBase::originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
    {
        limit = Settings::getInstance().getIntSetting("MIP.SolutionLimit.Initial", "Dual");
    }

    try
    {
        cplexInstance.setParam(IloCplex::IntSolLim, limit);
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when setting solution limit", e.getMessage());
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

        ProcessInfo::getInstance().outputError("Error when obtaining solution limit", e.getMessage());
    }

    return (solLim);
}

void MIPSolverCplexLazyOriginalCallback::checkParameters()
{
}
