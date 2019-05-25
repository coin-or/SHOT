/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplexLazy.h"

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

CplexCallback::CplexCallback(EnvironmentPtr envPtr, const IloNumVarArray& vars, const IloCplex& inst)
{
    std::lock_guard<std::mutex> lock(callbackMutex);

    env = envPtr;

    cplexVars = vars;
    cplexInst = inst;

    isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;

    env->solutionStatistics.iterationLastLazyAdded = 0;

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

        if((isMinimization && tmpDualObjBound > env->results->getCurrentDualBound())
            || (!isMinimization && tmpDualObjBound < env->results->getCurrentDualBound()))
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
            return;
        }

        if(context.inRelaxation())
        {
            if(env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
                < env->settings->getSetting<int>("Relaxation.MaxLazyConstraints", "Dual"))
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

                if(env->reformulatedProblem->objectiveFunction->properties.classification
                    > E_ObjectiveFunctionClassification::Quadratic)
                {
                    taskSelectHPPtsByObjectiveRootsearch->run(solutionPoints);
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
                currIter->isDualProblemDiscrete = true;
                currIter->dualProblemClass = env->dualSolver->MIPSolver->getProblemClass();
            }

            IloNumArray tmpVals(context.getEnv());

            context.getCandidatePoint(cplexVars, tmpVals);

            int numberOfVariables = (env->dualSolver->MIPSolver->hasAuxiliaryObjectiveVariable())
                ? tmpVals.getSize() - 1
                : tmpVals.getSize();

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

            auto bounds = std::make_pair(env->results->getCurrentDualBound(), env->results->getPrimalBound());
            currIter->currentObjectiveBounds = bounds;

            if(env->settings->getSetting<bool>("Rootsearch.Use", "Primal")
                && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
            {
                taskSelectPrimalSolutionFromRootsearch->run(candidatePoints);
            }

            if(checkFixedNLPStrategy(candidatePoints.at(0)))
            {
                env->primalSolver->addFixedNLPCandidate(candidatePoints.at(0).point, E_PrimalNLPSource::FirstSolution,
                    context.getCandidateObjective(), env->results->getCurrentIteration()->iterationNumber,
                    candidatePoints.at(0).maxDeviation);

                tSelectPrimNLP.get()->run();

                env->primalSolver->checkPrimalSolutionCandidates();
            }

            if(env->settings->getSetting<bool>("HyperplaneCuts.UseIntegerCuts", "Dual"))
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
        }

        // Add current primal solution as new incumbent candidate
        auto primalBound = env->results->getPrimalBound();

        if(env->results->hasPrimalSolution()
            && ((isMinimization && lastUpdatedPrimal < primalBound) || (!isMinimization && primalBound > primalBound)))
        {
            auto primalSol = env->results->primalSolution;

            IloNumArray tmpVals(context.getEnv());

            for(double S : primalSol)
            {
                tmpVals.add(S);
            }

            for(auto& V : env->reformulatedProblem->auxiliaryVariables)
            {
                tmpVals.add(V->calculateAuxiliaryValue(primalSol));
            }

            if(env->reformulatedProblem->auxiliaryObjectiveVariable)
                tmpVals.add(env->reformulatedProblem->auxiliaryObjectiveVariable->calculateAuxiliaryValue(primalSol));

            lastUpdatedPrimal = primalBound;

            /*
            // Adds cutoff

                double cutOffTol
                = env->settings->getSetting<double>("MIP.CutOffTolerance", "Dual");

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
            */
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("Cplex error when invoking general callback", e.getMessage());
    }
}

/// Destructor
CplexCallback::~CplexCallback() = default;

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

    if(hyperplaneIsOk)
    {
        IloExpr expr(context.getEnv());

        for(auto& P : tmpPair.first)
        {
            expr += P.value * cplexVars[P.index];
        }

        IloRange tmpRange(context.getEnv(), -IloInfinity, expr, -tmpPair.second);

        context.rejectCandidate(tmpRange);

        std::string identifier = env->dualSolver->MIPSolver->getConstraintIdentifier(hyperplane.source);

        if(hyperplane.sourceConstraint != nullptr)
            identifier = identifier + "_" + hyperplane.sourceConstraint->name;

        env->dualSolver->addGeneratedHyperplane(hyperplane);

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

    for(int I : binaryIndexesOnes)
    {
        expr += 1.0 * cplexVars[I];
    }

    for(int I : binaryIndexesZeroes)
    {
        expr += (1 - 1.0 * cplexVars[I]);
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

        for(auto& hp : env->dualSolver->MIPSolver->hyperplaneWaitingList)
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

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();
    modelUpdated = false;
}

MIPSolverCplexLazy::~MIPSolverCplexLazy() = default;

void MIPSolverCplexLazy::initializeSolverSettings()
{
    try
    {
        MIPSolverCplex::initializeSolverSettings();

        // cplexInstance.setParam(IloCplex::NumericalEmphasis, 1);
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

        CPXLONG contextMask = 0;

        if(getDiscreteVariableStatus())
        {
            CplexCallback cCallback(env, cplexVars, cplexInstance);
            contextMask |= IloCplex::Callback::Context::Id::Candidate;
            contextMask |= IloCplex::Callback::Context::Id::Relaxation;

            if(contextMask != 0)
                cplexInstance.use(&cCallback, contextMask);
        }

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