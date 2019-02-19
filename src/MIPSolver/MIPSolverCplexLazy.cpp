/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplexLazy.h"

namespace SHOT
{

CplexCallback::CplexCallback(EnvironmentPtr envPtr, const IloNumVarArray& vars, const IloCplex& inst)
{
    std::lock_guard<std::mutex> lock(callbackMutex);

    env = envPtr;

    cplexVars = vars;
    cplexInst = inst;

    isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;

    env->solutionStatistics.iterationLastLazyAdded = 0;

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

    lastUpdatedPrimal = env->results->getPrimalBound();
}

void CplexCallback::invoke(const IloCplex::Callback::Context& context)
{
    std::lock_guard<std::mutex> lock(callbackMutex);
    this->cbCalls++;

    try
    {
        // Check if better dual bound
        double tmpDualObjBound = context.getDoubleInfo(IloCplex::Callback::Context::Info::BestBound);

        if((isMinimization && tmpDualObjBound > env->results->getDualBound())
            || (!isMinimization && tmpDualObjBound < env->results->getDualBound()))
        {
            VectorDouble doubleSolution; // Empty since we have no point

            DualSolution sol = { doubleSolution, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound,
                env->results->getCurrentIteration()->iterationNumber };
            env->dualSolver->addDualSolutionCandidate(sol);
        }

        // Check for new primal solution
        double tmpPrimalObjBound = context.getIncumbentObjective();

        if((tmpPrimalObjBound < 1e74)
            && ((isMinimization && tmpPrimalObjBound < env->results->getPrimalBound())
                   || (!isMinimization && tmpPrimalObjBound > env->results->getPrimalBound())))
        {
            IloNumArray tmpPrimalVals(context.getEnv());

            context.getIncumbent(cplexVars, tmpPrimalVals);

            int numberOfVariables = env->problem->properties.numberOfVariables;

            VectorDouble primalSolution(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
            {
                primalSolution.at(i) = tmpPrimalVals[i];
            }

            SolutionPoint tmpPt;

            if(env->problem->properties.numberOfNonlinearConstraints > 0)
            {
                auto maxDev
                    = env->problem->getMaxNumericConstraintValue(primalSolution, env->problem->nonlinearConstraints);
                tmpPt.maxDeviation = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);
            }

            tmpPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
            tmpPt.objectiveValue = env->problem->objectiveFunction->calculateValue(primalSolution);
            tmpPt.point = primalSolution;

            env->primalSolver->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::LazyConstraintCallback);

            tmpPrimalVals.end();
        }

        if(env->results->isAbsoluteObjectiveGapToleranceMet() || env->results->isRelativeObjectiveGapToleranceMet()
            || checkIterationLimit() || checkUserTermination())
        {
            abort();
            return;
        }

        if(context.inRelaxation())
        {
            if(env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
                < env->settings->getIntSetting("Relaxation.MaxLazyConstraints", "Dual"))
            {
                int waitingListSize = env->dualSolver->MIPSolver->hyperplaneWaitingList.size();

                std::vector<SolutionPoint> solutionPoints(1);

                IloNumArray tmpVals(context.getEnv());

                context.getRelaxationPoint(cplexVars, tmpVals);

                int numberOfVariables = env->reformulatedProblem->properties.numberOfVariables;

                VectorDouble solution(numberOfVariables);

                for(int i = 0; i < numberOfVariables; i++)
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
                tmpSolPt.objectiveValue = context.getRelaxationObjective();
                tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;

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

                if(env->reformulatedProblem->objectiveFunction->properties.classification
                    > E_ObjectiveFunctionClassification::Quadratic)
                {
                    taskSelectHPPtsByObjectiveLinesearch->run(solutionPoints);
                }

                env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
                    += (env->dualSolver->MIPSolver->hyperplaneWaitingList.size() - waitingListSize);
            }
        }

        if(context.inCandidate())
        {
            auto currIter = env->results->getCurrentIteration();

            if(currIter->isSolved)
            {
                env->results->createIteration();
                currIter = env->results->getCurrentIteration();
            }

            IloNumArray tmpVals(context.getEnv());

            context.getCandidatePoint(cplexVars, tmpVals);

            int numberOfVariables = env->reformulatedProblem->properties.numberOfVariables;

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

            solutionCandidate.point = solution;
            solutionCandidate.objectiveValue = context.getCandidateObjective();
            solutionCandidate.iterFound = env->results->getCurrentIteration()->iterationNumber;

            std::vector<SolutionPoint> candidatePoints(1);
            candidatePoints.at(0) = solutionCandidate;

            addLazyConstraint(candidatePoints, context);

            currIter->maxDeviation = solutionCandidate.maxDeviation.value;
            currIter->maxDeviationConstraint = solutionCandidate.maxDeviation.index;
            currIter->solutionStatus = E_ProblemSolutionStatus::Feasible;
            currIter->objectiveValue = context.getCandidateObjective();

            env->results->getCurrentIteration()->numberOfOpenNodes = cplexInst.getNnodesLeft();
            env->solutionStatistics.numberOfExploredNodes
                = std::max(context.getIntInfo(IloCplex::Callback::Context::Info::NodeCount),
                    env->solutionStatistics.numberOfExploredNodes);

            auto bounds = std::make_pair(env->results->getDualBound(), env->results->getPrimalBound());
            currIter->currentObjectiveBounds = bounds;

            if(env->settings->getBoolSetting("Linesearch.Use", "Primal")
                && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
            {
                taskSelectPrimalSolutionFromLinesearch->run(candidatePoints);
            }

            if(checkFixedNLPStrategy(candidatePoints.at(0)))
            {
                env->primalSolver->addFixedNLPCandidate(candidatePoints.at(0).point, E_PrimalNLPSource::FirstSolution,
                    context.getCandidateObjective(), env->results->getCurrentIteration()->iterationNumber,
                    candidatePoints.at(0).maxDeviation);

                tSelectPrimNLP.get()->run();

                env->primalSolver->checkPrimalSolutionCandidates();
            }

            if(env->settings->getBoolSetting("HyperplaneCuts.UseIntegerCuts", "Dual"))
            {
                bool addedIntegerCut = false;

                for(auto& ic : env->dualSolver->MIPSolver->integerCutWaitingList)
                {
                    this->createIntegerCut(ic.first, ic.second, context);
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

            currIter->isSolved = true;

            auto threadId = std::to_string(context.getIntInfo(IloCplex::Callback::Context::Info::ThreadId));
            printIterationReport(candidatePoints.at(0), threadId);

            if(env->results->isAbsoluteObjectiveGapToleranceMet() || env->results->isRelativeObjectiveGapToleranceMet())
            {
                abort();
                return;
            }
        }

        // Add current primal bound as new incumbent candidate
        auto primalBound = env->results->getPrimalBound();

        if(((isMinimization && lastUpdatedPrimal < primalBound) || (!isMinimization && primalBound > primalBound)))
        {
            auto primalSol = env->results->primalSolution;

            IloNumArray tmpVals(context.getEnv());

            VectorDouble solution(primalSol.size());

            for(int i = 0; i < primalSol.size(); i++)
            {
                tmpVals.add(primalSol.at(i));
            }

            for(auto& V : env->reformulatedProblem->auxilliaryVariables)
            {
                tmpVals.add(V->calculateValue(primalSol));
            }

            context.postHeuristicSolution(
                cplexVars, tmpVals, primalBound, IloCplex::Callback::Context::SolutionStrategy::CheckFeasible);

            tmpVals.end();

            lastUpdatedPrimal = primalBound;
        }

        // Adds cutoff

        double cutOffTol = env->settings->getDoubleSetting("MIP.CutOffTolerance", "Dual");

        if(isMinimization)
        {
            (static_cast<MIPSolverCplexLazy*>(env->dualSolver->MIPSolver.get()))
                ->cplexInstance.setParam(IloCplex::CutUp, primalBound + cutOffTol);

            env->output->outputDebug(
                "     Setting cutoff value to " + std::to_string(primalBound + cutOffTol) + " for minimization.");
        }
        else
        {
            (static_cast<MIPSolverCplexLazy*>(env->dualSolver->MIPSolver.get()))
                ->cplexInstance.setParam(IloCplex::CutLo, primalBound - cutOffTol);

            env->output->outputDebug(
                "     Setting cutoff value to " + std::to_string(primalBound - cutOffTol) + " for maximization.");
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("Cplex error when invoking general callback", e.getMessage());
    }
}

/// Destructor
CplexCallback::~CplexCallback() {}

void CplexCallback::createHyperplane(Hyperplane hyperplane, const IloCplex::Callback::Context& context)
{
    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration
    auto optionalHyperplanes = env->dualSolver->MIPSolver->createHyperplaneTerms(hyperplane);

    if(!optionalHyperplanes)
    {
        return;
    }

    auto tmpPair = optionalHyperplanes.value();

    bool hyperplaneIsOk = true;

    for(auto &E : tmpPair.first)
    {
        if(E.value != E.value) // Check for NaN
        {
            env->output->outputError("     Warning: hyperplane not generated, NaN found in linear terms for variable "
                + env->problem->getVariable(E.index)->name);
            hyperplaneIsOk = false;
            break;
        }
    }

    if(hyperplaneIsOk)
    {
        GeneratedHyperplane genHyperplane;

        IloExpr expr(context.getEnv());

        for(int i = 0; i < tmpPair.first.size(); i++)
        {
            expr += tmpPair.first.at(i).value * cplexVars[tmpPair.first.at(i).index];
        }

        IloRange tmpRange(context.getEnv(), -IloInfinity, expr, -tmpPair.second);

        auto addedConstr = context.rejectCandidate(tmpRange);

        int constrIndex = 0;
        genHyperplane.generatedConstraintIndex = constrIndex;
        genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
        genHyperplane.generatedPoint = hyperplane.generatedPoint;
        genHyperplane.source = hyperplane.source;
        genHyperplane.generatedIter = currIter->iterationNumber;
        genHyperplane.isLazy = true;
        genHyperplane.isRemoved = false;

        currIter->numHyperplanesAdded++;
        currIter->totNumHyperplanes++;
        tmpRange.end();
        expr.end();
    }
}

void CplexCallback::createIntegerCut(
    VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes, const IloCplex::Callback::Context& context)
{
    IloExpr expr(context.getEnv());

    for(int i = 0; i < binaryIndexesOnes.size(); i++)
    {
        expr += 1.0 * cplexVars[binaryIndexesOnes.at(i)];
    }

    for(int i = 0; i < binaryIndexesZeroes.size(); i++)
    {
        expr += (1 - 1.0 * cplexVars[binaryIndexesZeroes.at(i)]);
    }

    IloRange tmpRange(
        context.getEnv(), -IloInfinity, expr, binaryIndexesOnes.size() + binaryIndexesZeroes.size() - 1.0);
    tmpRange.setName("IC");

    context.rejectCandidate(tmpRange);
    env->solutionStatistics.numberOfIntegerCuts++;

    tmpRange.end();
    expr.end();
}

void CplexCallback::addLazyConstraint(
    std::vector<SolutionPoint> candidatePoints, const IloCplex::Callback::Context& context)
{
    try
    {
        env->results->getCurrentIteration()->numHyperplanesAdded++;

        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            tUpdateInteriorPoint->run();

            static_cast<TaskSelectHyperplanePointsESH*>(taskSelectHPPts.get())->run(candidatePoints);

            if(env->reformulatedProblem->objectiveFunction->properties.classification
                > E_ObjectiveFunctionClassification::Quadratic)
            {
                taskSelectHPPtsByObjectiveLinesearch->run(candidatePoints);
            }
        }
        else
        {
            static_cast<TaskSelectHyperplanePointsECP*>(taskSelectHPPts.get())->run(candidatePoints);

            if(env->reformulatedProblem->objectiveFunction->properties.classification
                > E_ObjectiveFunctionClassification::Quadratic)
            {
                taskSelectHPPtsByObjectiveLinesearch->run(candidatePoints);
            }
        }

        for(auto &hp : env->dualSolver->MIPSolver->hyperplaneWaitingList)
        {
            this->createHyperplane(hp, context);
            this->lastNumAddedHyperplanes++;
        }

        env->dualSolver->MIPSolver->hyperplaneWaitingList.clear();
    }
    catch(IloException& e)
    {
        env->output->outputError("Cplex error when invoking general lazy callback", e.getMessage());
    }
}

MIPSolverCplexLazy::MIPSolverCplexLazy(EnvironmentPtr envPtr)
{
    env = envPtr;

    discreteVariablesActivated = true;

    cplexModel = IloModel(cplexEnv);

    cplexVars = IloNumVarArray(cplexEnv);
    cplexConstrs = IloRangeArray(cplexEnv);

    // cplexLazyConstrs = IloRangeArray(cplexEnv);

    // itersSinceNLPCall = 0;

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();
    modelUpdated = false;
}

MIPSolverCplexLazy::~MIPSolverCplexLazy() {}

void MIPSolverCplexLazy::initializeSolverSettings()
{
    try
    {
        MIPSolverCplex::initializeSolverSettings();

        cplexInstance.setParam(IloCplex::NumericalEmphasis, 1);
    }
    catch(IloException& e)
    {
        env->output->outputError("Cplex error when initializing parameters for linear solver", e.getMessage());
    }
}

E_ProblemSolutionStatus MIPSolverCplexLazy::solveProblem()
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

        activateDiscreteVariables(true); // Otherwise we will get an error from CPLEX

        CplexCallback cCallback(env, cplexVars, cplexInstance);
        CPXLONG contextMask = 0;

        contextMask |= IloCplex::Callback::Context::Id::Candidate;
        contextMask |= IloCplex::Callback::Context::Id::Relaxation;

        // If contextMask is not zero we add the callback.
        if(contextMask != 0)
            cplexInstance.use(&cCallback, contextMask);

        // This fixes a bug in CPLEX
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

int MIPSolverCplexLazy::increaseSolutionLimit(int increment)
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

void MIPSolverCplexLazy::setSolutionLimit(long limit)
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

int MIPSolverCplexLazy::getSolutionLimit()
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

void MIPSolverCplexLazy::checkParameters() {}
} // namespace SHOT