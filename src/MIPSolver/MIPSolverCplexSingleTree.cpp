/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplexSingleTree.h"

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

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        taskSelectHPPtsByObjectiveRootsearch = std::make_shared<TaskSelectHyperplanesObjectiveFunction>(env);
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

    if(env->settings->getSetting<bool>("Rootsearch.Use", "Primal")
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        taskSelectPrimalSolutionFromRootsearch = std::make_shared<TaskSelectPrimalCandidatesFromRootsearch>(env);
    }

    lastUpdatedPrimal = env->results->getPrimalBound();

    isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;
}

void CplexCallback::invoke(const IloCplex::Callback::Context& context)
{
    try
    {
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

                env->primalSolver->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::MIPCallback);
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
                        static_cast<TaskSelectHyperplanesESH*>(taskSelectHPPts.get())->run(solutionPoints);
                    }
                    else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
                        == ES_HyperplaneCutStrategy::ECP)
                    {
                        static_cast<TaskSelectHyperplanesECP*>(taskSelectHPPts.get())->run(solutionPoints);
                    }

                    if(env->reformulatedProblem->objectiveFunction->properties.classification
                        > E_ObjectiveFunctionClassification::Quadratic)
                    {
                        taskSelectHPPtsByObjectiveRootsearch->run(solutionPoints);
                    }

                    taskSelectExternalHPs->run(solutionPoints);

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

                if(taskSelectPrimNLPOriginal)
                {
                    env->primalSolver->addFixedNLPCandidate(candidatePoints.at(0).point,
                        E_PrimalNLPSource::FirstSolution, context.getCandidateObjective(),
                        env->results->getCurrentIteration()->iterationNumber, candidatePoints.at(0).maxDeviation);

                    taskSelectPrimNLPOriginal->run();
                    env->primalSolver->fixedPrimalNLPCandidates.clear();
                }

                if(taskSelectPrimNLPReformulated)
                {
                    env->primalSolver->addFixedNLPCandidate(candidatePoints.at(0).point,
                        E_PrimalNLPSource::FirstSolution, context.getCandidateObjective(),
                        env->results->getCurrentIteration()->iterationNumber, candidatePoints.at(0).maxDeviation);

                    taskSelectPrimNLPReformulated->run();
                    env->primalSolver->fixedPrimalNLPCandidates.clear();
                }

                env->primalSolver->checkPrimalSolutionCandidates();
            }

            if(env->settings->getSetting<bool>("HyperplaneCuts.UseIntegerCuts", "Dual"))
            {
                int addedIntegerCuts = 0;

                for(auto& IC : env->dualSolver->integerCutWaitingList)
                {
                    if(this->createIntegerCut(IC, context))
                    {
                        env->dualSolver->addGeneratedIntegerCut(IC);
                        addedIntegerCuts++;
                    }
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

            if((int)primalSol.size() < env->reformulatedProblem->properties.numberOfVariables)
                env->reformulatedProblem->augmentAuxiliaryVariableValues(primalSol);

            if(env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
                primalSol.push_back(env->reformulatedProblem->objectiveFunction->calculateValue(primalSol));

            assert(cplexVars.getSize() == primalSol.size());

            for(double S : primalSol)
                tmpVals.add(S);

            try
            {
                context.postHeuristicSolution(cplexVars, tmpVals, env->results->currentPrimalBound,
                    IloCplex::Callback::Context::SolutionStrategy::CheckFeasible);
            }
            catch(IloException& e)
            {
                env->output->outputError(
                    "        Error when setting primal solution as starting point in heuristic callback:",
                    e.getMessage());
            }

            tmpVals.end();

            lastUpdatedPrimal = env->results->getPrimalBound();
        }
    }
    catch(IloException& e)
    {
        env->output->outputError("        Cplex error when invoking general callback", e.getMessage());
    }
}

/// Destructor
CplexCallback::~CplexCallback() = default;

bool CplexCallback::createHyperplane(HyperplanePtr hyperplane, const IloCplex::Callback::Context& context)
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
        IloExpr expr(context.getEnv());

        for(auto& P : tmpPair.first)
        {
            expr += P.second * cplexVars[P.first];
        }

        IloRange tmpRange(context.getEnv(), -IloInfinity, expr, -tmpPair.second);

        context.rejectCandidate(tmpRange);

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

bool CplexCallback::createIntegerCut(IntegerCut& integerCut, const IloCplex::Callback::Context& context)
{
    if(!integerCut.areAllVariablesBinary)
    {
        env->output->outputDebug("        Integer cut for nonbinary variables not supported in single-tree strategy.");
        return (false);
    }

    try
    {
        IloExpr expr(context.getEnv());
        size_t index = 0;

        for(auto& VAR : env->reformulatedProblem->allVariables)
        {
            if(!(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
                   || VAR->properties.type == E_VariableType::Semiinteger))
                continue;

            int variableValue = integerCut.variableValues[index];
            auto variable = cplexVars[VAR->index];

            if(variableValue == VAR->upperBound)
            {
                expr += (variableValue - variable);
            }
            else if(variableValue == VAR->lowerBound)
            {
                expr += variable;
            }

            index++;
        }

        IloRange tmpRange(context.getEnv(), 1, expr, IloInfinity,
            fmt::format("IC{}", env->solutionStatistics.numberOfIntegerCuts).c_str());

        context.rejectCandidate(tmpRange);

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
            if(this->createHyperplane(hp, context))
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
            CPXLONG contextMask = 0;
            contextMask |= IloCplex::Callback::Context::Id::Candidate;
            contextMask |= IloCplex::Callback::Context::Id::Relaxation;
            // contextMask |= IloCplex::Callback::Context::Id::GlobalProgress;
            // contextMask |= IloCplex::Callback::Context::Id::LocalProgress;
            // contextMask |= IloCplex::Callback::Context::Id::ThreadUp;
            // contextMask |= IloCplex::Callback::Context::Id::ThreadDown;

            CplexCallback cCallback(env, cplexVars, cplexInstance);

            if(contextMask != 0)
                cplexInstance.use(&cCallback, contextMask);

            // Fixes a deadlock bug in Cplex 12.7 and 12.8
            cplexEnv.setNormalizer(false);

            cplexInstance.solve();
            MIPSolutionStatus = MIPSolverCplex::getSolutionStatus();
        }

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

        // If the previous repair failed, we can try this
        if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded)
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
} // namespace SHOT