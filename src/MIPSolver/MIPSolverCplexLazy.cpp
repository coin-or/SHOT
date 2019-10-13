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
    env = envPtr;
    lastUpdatedPrimal = env->results->getPrimalBound();

    cplexVars = vars;
    cplexInst = inst;

    isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;
}

void CplexCallback::invoke(const IloCplex::Callback::Context& context)
{
    try
    {
        if(context.inThreadUp())
        {
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

            if(env->reformulatedProblem->objectiveFunction->properties.classification
                > E_ObjectiveFunctionClassification::Quadratic)
            {
                taskSelectHPPtsByObjectiveRootsearch
                    = std::make_shared<TaskSelectHyperplanePointsByObjectiveRootsearch>(env);
            }

            tSelectPrimNLP = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env);

            if(env->settings->getSetting<bool>("Rootsearch.Use", "Primal")
                && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
            {
                taskSelectPrimalSolutionFromRootsearch
                    = std::make_shared<TaskSelectPrimalCandidatesFromRootsearch>(env);
            }

            lastUpdatedPrimal = env->results->getPrimalBound();

            return;
        }

        if(context.inThreadDown())
        {
            tUpdateInteriorPoint = nullptr;
            taskSelectHPPts = nullptr;
            taskSelectHPPtsByObjectiveRootsearch = nullptr;
            return;
        }

        // Check if better dual bound
        double tmpDualObjBound = context.getDoubleInfo(IloCplex::Callback::Context::Info::BestBound);

        if((isMinimization && tmpDualObjBound > env->results->getCurrentDualBound())
            || (!isMinimization && tmpDualObjBound < env->results->getCurrentDualBound()))
        {
            std::lock_guard<std::mutex> lock(callbackMutex);
            VectorDouble doubleSolution; // Empty since we have no point

            DualSolution sol = { doubleSolution, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound,
                env->results->getCurrentIteration()->iterationNumber, false };

            env->dualSolver->addDualSolutionCandidate(sol);
        }

        if(context.inCandidate())
        {
            // Check for new primal solution
            double tmpPrimalObjBound = context.getCandidateObjective();

            if((tmpPrimalObjBound < 1e74)
                && ((isMinimization && tmpPrimalObjBound < env->results->getPrimalBound())
                       || (!isMinimization && tmpPrimalObjBound > env->results->getPrimalBound())))
            {
                IloNumArray tmpPrimalVals(context.getEnv());

                context.getCandidatePoint(cplexVars, tmpPrimalVals);

                int numberOfVariables = env->problem->properties.numberOfVariables;

                VectorDouble primalSolution(numberOfVariables);

                for(int i = 0; i < numberOfVariables; i++)
                {
                    primalSolution.at(i) = tmpPrimalVals[i];
                }

                tmpPrimalVals.end();

                SolutionPoint tmpPt;

                if(env->problem->properties.numberOfNonlinearConstraints > 0)
                {
                    auto maxDev = env->problem->getMaxNumericConstraintValue(
                        primalSolution, env->problem->nonlinearConstraints);
                    tmpPt.maxDeviation = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);
                }
                else
                {
                    tmpPt.maxDeviation = PairIndexValue(-1, 0.0);
                }

                std::lock_guard<std::mutex> lock(callbackMutex);

                tmpPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
                tmpPt.objectiveValue = env->problem->objectiveFunction->calculateValue(primalSolution);
                tmpPt.point = primalSolution;

                env->primalSolver->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::LazyConstraintCallback);
            }
        }

        if(env->results->isAbsoluteObjectiveGapToleranceMet() || env->results->isRelativeObjectiveGapToleranceMet()
            || checkIterationLimit() || checkUserTermination())
        {
            context.abort();
            return;
        }

        if(context.inRelaxation())
        {
            int numberOfAddedHyperplanes;
            int iterationNumber;

            {
                std::lock_guard<std::mutex> lock(callbackMutex);
                numberOfAddedHyperplanes = env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded;
                iterationNumber = env->results->getCurrentIteration()->iterationNumber;
            }

            if(numberOfAddedHyperplanes < env->settings->getSetting<int>("Relaxation.MaxLazyConstraints", "Dual"))
            {
                int waitingListSize = env->dualSolver->hyperplaneWaitingList.size();

                IloNumArray tmpVals(context.getEnv());

                context.getRelaxationPoint(cplexVars, tmpVals);

                int numberOfVariables = (env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
                    ? tmpVals.getSize() - 1
                    : tmpVals.getSize();

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
                    solutionRelaxed.maxDeviation = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);
                }
                else
                {
                    solutionRelaxed.maxDeviation = PairIndexValue(-1, 0.0);
                }

                solutionRelaxed.point = solution;
                solutionRelaxed.objectiveValue = context.getRelaxationObjective();
                solutionRelaxed.iterFound = iterationNumber;
                solutionRelaxed.isRelaxedPoint = true;

                std::vector<SolutionPoint> solutionPoints = { solutionRelaxed };

                {
                    std::lock_guard<std::mutex> lock(callbackMutex);
                    if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
                        == ES_HyperplaneCutStrategy::ESH)
                    {
                        tUpdateInteriorPoint->run();
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
                        += (env->dualSolver->hyperplaneWaitingList.size() - waitingListSize);
                }
            }
        }

        if(context.inCandidate())
        {
            IterationPtr currIter;

            {
                std::lock_guard<std::mutex> lock(callbackMutex);
                currIter = env->results->getCurrentIteration();
            }

            std::vector<SolutionPoint> candidatePoints;

            std::lock_guard<std::mutex> lock(callbackMutex);

            if(currIter->isSolved)
            {
                env->results->createIteration();
                currIter = env->results->getCurrentIteration();
                currIter->isDualProblemDiscrete = true;
                currIter->dualProblemClass = env->dualSolver->MIPSolver->getProblemClass();
            }

            IloNumArray tmpVals(context.getEnv());

            context.getCandidatePoint(cplexVars, tmpVals);

            int numberOfVariables = (env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
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
            else
            {
                solutionCandidate.maxDeviation = PairIndexValue(-1, 0.0);
            }

            solutionCandidate.point = solution;
            solutionCandidate.objectiveValue = context.getCandidateObjective();
            solutionCandidate.iterFound = env->results->getCurrentIteration()->iterationNumber;

            candidatePoints.push_back(solutionCandidate);

            addLazyConstraint(candidatePoints, context);

            currIter->maxDeviation = solutionCandidate.maxDeviation.value;
            currIter->maxDeviationConstraint = solutionCandidate.maxDeviation.index;
            currIter->solutionStatus = E_ProblemSolutionStatus::Feasible;
            currIter->objectiveValue = context.getCandidateObjective();

            currIter->numberOfOpenNodes = cplexInst.getNnodesLeft();
            env->solutionStatistics.numberOfExploredNodes
                = std::max(context.getIntInfo(IloCplex::Callback::Context::Info::NodeCount),
                    env->solutionStatistics.numberOfExploredNodes);

            auto bounds = std::make_pair(env->results->getCurrentDualBound(), env->results->getPrimalBound());
            currIter->currentObjectiveBounds = bounds;

            if(env->settings->getSetting<bool>("Rootsearch.Use", "Primal")
                && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
            {
                taskSelectPrimalSolutionFromRootsearch->run(candidatePoints);
                env->primalSolver->checkPrimalSolutionCandidates();
            }

            currIter->isSolved = true;

            auto threadId = std::to_string(context.getIntInfo(IloCplex::Callback::Context::Info::ThreadId));
            printIterationReport(candidatePoints.at(0), threadId);

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
                int addedIntegerCuts = 0;

                for(auto& IC : env->dualSolver->integerCutWaitingList)
                {
                    if(this->createIntegerCut(IC.first, IC.second, context))
                        addedIntegerCuts++;
                }

                if(addedIntegerCuts > 0)
                    env->output->outputDebug(fmt::format("        Added {} integer cut(s)", addedIntegerCuts));

                env->dualSolver->integerCutWaitingList.clear();
            }
        }

        // Add current primal solution as new incumbent candidate
        auto primalBound = env->results->getPrimalBound();

        if((isMinimization && lastUpdatedPrimal < primalBound) || (!isMinimization && lastUpdatedPrimal > primalBound))
        {
            auto primalSol = env->results->primalSolution;

            IloNumArray tmpVals(context.getEnv());

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
                context.postHeuristicSolution(cplexVars, tmpVals, env->results->currentPrimalBound,
                    IloCplex::Callback::Context::SolutionStrategy::CheckFeasible);
            }
            catch(IloException& e)
            {
                env->output->outputError(
                    "Error when setting primal solution as starting point in heuristic callback:", e.getMessage());
            }

            tmpVals.end();

            lastUpdatedPrimal = env->results->getPrimalBound();
        }
        /*
        // Adds cutoff

            double cutOffTol
            = env->settings->getSetting<double>("MIP.CutOffTolerance", "Dual");

        if(isMinimization)
        {
            (static_cast<MIPSolverCplexLazy*>(env->dualSolver->MIPSolver.get()))
                ->cplexInstance.setParam(IloCplex::CutUp, primalBound + cutOffTol);

            env->output->outputDebug(
                "     Setting cutoff value to " + std::to_string(primalBound + cutOffTol) + " for
        minimization.");
        }
        else
        {
            (static_cast<MIPSolverCplexLazy*>(env->dualSolver->MIPSolver.get()))
                ->cplexInstance.setParam(IloCplex::CutLo, primalBound - cutOffTol);

            env->output->outputDebug(
                "     Setting cutoff value to " + std::to_string(primalBound - cutOffTol) + " for
        maximization.");
        }
        */
    }
    catch(IloException& e)
    {
        env->output->outputError("Cplex error when invoking general callback", e.getMessage());
    }
}

/// Destructor
CplexCallback::~CplexCallback() = default;

bool CplexCallback::createHyperplane(Hyperplane hyperplane, const IloCplex::Callback::Context& context)
{
    auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration
    auto optionalHyperplanes = env->dualSolver->MIPSolver->createHyperplaneTerms(hyperplane);

    if(!optionalHyperplanes)
    {
        return (false);
    }

    auto tmpPair = optionalHyperplanes.value();

    for(auto& E : tmpPair.first)
    {
        if(E.second != E.second) // Check for NaN
        {
            env->output->outputError("     Warning: hyperplane not generated, NaN found in linear terms for variable "
                + env->problem->getVariable(E.first)->name);
            return (false);
        }
    }

    try
    {
        IloExpr expr(context.getEnv());

        for(auto& P : tmpPair.first)
        {
            expr += P.second * cplexVars[P.first];
        }

        IloRange tmpRange(context.getEnv(), -IloInfinity, expr, -tmpPair.second);

        context.rejectCandidate(tmpRange);

        std::string identifier = env->dualSolver->MIPSolver->getConstraintIdentifier(hyperplane.source);

        if(hyperplane.sourceConstraint != nullptr)
            identifier = identifier + "_" + hyperplane.sourceConstraint->name;

        env->dualSolver->addGeneratedHyperplane(hyperplane);

        tmpRange.end();
        expr.end();
    }
    catch(IloException& e)
    {
        env->output->outputError("        Error when creating hyperplane in Cplex", e.getMessage());
        return (false);
    }

    return (true);
}

bool CplexCallback::createIntegerCut(
    VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes, const IloCplex::Callback::Context& context)
{
    try
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
            this->createHyperplane(hp, context);
            this->lastNumAddedHyperplanes++;
        }

        env->dualSolver->hyperplaneWaitingList.clear();
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
    modelUpdated = false;
    checkParameters();
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

        if(getDiscreteVariableStatus())
        {
            CPXLONG contextMask = 0;
            CplexCallback cCallback(env, cplexVars, cplexInstance);
            contextMask |= IloCplex::Callback::Context::Id::Candidate;
            contextMask |= IloCplex::Callback::Context::Id::Relaxation;
            // contextMask |= IloCplex::Callback::Context::Id::GlobalProgress;
            // contextMask |= IloCplex::Callback::Context::Id::LocalProgress;
            contextMask |= IloCplex::Callback::Context::Id::ThreadUp;
            contextMask |= IloCplex::Callback::Context::Id::ThreadDown;

            if(contextMask != 0)
                cplexInstance.use(&cCallback, contextMask);

            // This fixes a bug in CPLEX
            cplexEnv.setNormalizer(false);

            cplexInstance.solve();
        }
        else
        {
            // This fixes a bug in CPLEX
            cplexEnv.setNormalizer(false);

            cplexInstance.solve();
        }

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

int MIPSolverCplexLazy::increaseSolutionLimit(int increment)
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