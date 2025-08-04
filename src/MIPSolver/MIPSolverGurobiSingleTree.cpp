/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverGurobiSingleTree.h"

#include "../DualSolver.h"
#include "../EventHandler.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../PrimalSolver.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"

namespace SHOT
{

MIPSolverGurobiSingleTree::MIPSolverGurobiSingleTree(EnvironmentPtr envPtr)
{
    env = envPtr;
    discreteVariablesActivated = true;

    try
    {
        GRBEnv env = GRBEnv();
        gurobiModel = std::make_shared<GRBModel>(env);
    }
    catch(GRBException& e)
    {
        env->output->outputError(" Error when initializing Gurobi: ", e.getMessage());
        return;
    }

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;
}

MIPSolverGurobiSingleTree::~MIPSolverGurobiSingleTree() = default;

void MIPSolverGurobiSingleTree::initializeSolverSettings()
{
    MIPSolverGurobi::initializeSolverSettings();

    try
    {
        gurobiModel->set(GRB_IntParam_LazyConstraints, 1);
    }
    catch(GRBException& e)
    {
        env->output->outputError(" Error when initializing Gurobi parameters: ", e.getMessage());
    }
}

int MIPSolverGurobiSingleTree::increaseSolutionLimit(int increment)
{
    gurobiModel->set(GRB_IntParam_SolutionLimit, gurobiModel->get(GRB_IntParam_SolutionLimit) + increment);

    return (gurobiModel->get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobiSingleTree::setSolutionLimit(long limit)
{
    if(limit > GRB_MAXINT)
        gurobiModel->set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
    else
        gurobiModel->set(GRB_IntParam_SolutionLimit, limit);
}

int MIPSolverGurobiSingleTree::getSolutionLimit() { return (gurobiModel->get(GRB_IntParam_SolutionLimit)); }

void MIPSolverGurobiSingleTree::checkParameters() { }

E_ProblemSolutionStatus MIPSolverGurobiSingleTree::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    cachedSolutionHasChanged = true;

    try
    {
        if(!isCallbackInitialized)
        {
            gurobiCallback = std::make_unique<GurobiCallbackSingleTree>(gurobiModel->getVars(), env);
            isCallbackInitialized = true;
        }

        gurobiModel->set(GRB_IntParam_LazyConstraints, 1);
        gurobiModel->setCallback(gurobiCallback.get());

        gurobiModel->optimize();

        MIPSolutionStatus = getSolutionStatus();
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    // To find a feasible point for an unbounded dual problem
    if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded)
    {
        bool variableBoundsUpdated = false;

        if((env->reformulatedProblem->objectiveFunction->properties.classification
                   == E_ObjectiveFunctionClassification::Linear
               && std::dynamic_pointer_cast<LinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                      ->isDualUnbounded())
            || (env->reformulatedProblem->objectiveFunction->properties.classification
                    == E_ObjectiveFunctionClassification::Quadratic
                && std::dynamic_pointer_cast<QuadraticObjectiveFunction>(env->reformulatedProblem->objectiveFunction)
                       ->isDualUnbounded()))
        {
            for(auto& V : env->reformulatedProblem->allVariables)
            {
                if(V->isDualUnbounded())
                {
                    updateVariableBound(
                        V->index, -getUnboundedVariableBoundValue() / 1.1, getUnboundedVariableBoundValue() / 1.1);
                    variableBoundsUpdated = true;
                }
            }
        }
        else if(env->reformulatedProblem->objectiveFunction->properties.classification
                >= E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear
            && hasDualAuxiliaryObjectiveVariable())
        {
            // The auxiliary variable in the dual problem is unbounded
            updateVariableBound(getDualAuxiliaryObjectiveVariableIndex(), -getUnboundedVariableBoundValue() / 1.1,
                getUnboundedVariableBoundValue() / 1.1);
            variableBoundsUpdated = true;
        }

        if(variableBoundsUpdated)
        {
            gurobiModel->update();

            if(!isCallbackInitialized)
            {
                gurobiCallback = std::make_unique<GurobiCallbackSingleTree>(gurobiModel->getVars(), env);
                isCallbackInitialized = true;
            }

            gurobiModel->set(GRB_IntParam_LazyConstraints, 1);
            gurobiModel->setCallback(gurobiCallback.get());

            gurobiModel->optimize();

            MIPSolutionStatus = getSolutionStatus();

            for(auto& V : env->reformulatedProblem->allVariables)
            {
                if(V->isDualUnbounded())
                    updateVariableBound(V->index, V->lowerBound, V->upperBound);
            }

            env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed = true;
        }
    }

    return (MIPSolutionStatus);
}

void GurobiCallbackSingleTree::callback()
{
    if(where == GRB_CB_POLLING || where == GRB_CB_PRESOLVE || where == GRB_CB_SIMPLEX || where == GRB_CB_BARRIER)
        return;

    if(where == GRB_CB_MESSAGE && showOutput) // Show output on console and log
    {
        auto message = getStringInfo(GRB_CB_MSG_STRING);
        message.erase(std::remove(message.begin(), message.end(), '\n'), message.end());
        env->output->outputInfo(fmt::format("      | {} ", message));
    }

    try
    {

        // Add current primal bound as new incumbent candidate
        auto primalBound = env->results->getPrimalBound();

        if(((isMinimization && lastUpdatedPrimal < primalBound)
               || (!isMinimization && lastUpdatedPrimal > primalBound)))
        {
            auto primalSol = env->results->primalSolution;

            if((int)primalSol.size() < env->reformulatedProblem->properties.numberOfVariables)
                env->reformulatedProblem->augmentAuxiliaryVariableValues(primalSol);

            assert(env->reformulatedProblem->properties.numberOfVariables == primalSol.size());

            if(env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
                primalSol.push_back(env->reformulatedProblem->objectiveFunction->calculateValue(primalSol));

            for(size_t i = 0; i < primalSol.size(); i++)
            {
                setSolution(vars[i], primalSol.at(i));
            }

            lastUpdatedPrimal = env->results->getPrimalBound();
        }

        // Check if better dual bound
        double tmpDualObjBound;

        if(where == GRB_CB_MIP || where == GRB_CB_MIPSOL || where == GRB_CB_MIPNODE)
        {
            switch(where)
            {
            case GRB_CB_MIP:
                tmpDualObjBound = getDoubleInfo(GRB_CB_MIP_OBJBND);
                break;
            case GRB_CB_MIPSOL:
                tmpDualObjBound = getDoubleInfo(GRB_CB_MIPSOL_OBJBND);
                break;
            case GRB_CB_MIPNODE:
                tmpDualObjBound = getDoubleInfo(GRB_CB_MIPNODE_OBJBND);
                break;
            default:
                tmpDualObjBound = 0.0; // just to suppress a compiler warning
                break;
            }

            if((isMinimization && tmpDualObjBound > env->results->getCurrentDualBound())
                || (!isMinimization && tmpDualObjBound < env->results->getCurrentDualBound()))
            {
                VectorDouble doubleSolution; // Empty since we have no point

                DualSolution sol = { doubleSolution, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound,
                    env->results->getCurrentIteration()->iterationNumber, false };
                env->dualSolver->addDualSolutionCandidate(sol);
            }
        }

        if(where == GRB_CB_MIPSOL)
        {
            // Check for new primal solution
            double tmpPrimalObjBound = getDoubleInfo(GRB_CB_MIPSOL_OBJ);

            if((tmpPrimalObjBound < 1e100)
                && ((isMinimization && tmpPrimalObjBound < env->results->getPrimalBound())
                    || (!isMinimization && tmpPrimalObjBound > env->results->getPrimalBound())))
            {
                int numberOfVariables = env->problem->properties.numberOfVariables;
                VectorDouble primalSolution(numberOfVariables);

                for(int i = 0; i < numberOfVariables; i++)
                {
                    primalSolution.at(i) = getSolution(vars[i]);
                }

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

                tmpPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
                tmpPt.objectiveValue = env->problem->objectiveFunction->calculateValue(primalSolution);
                tmpPt.point = primalSolution;

                env->primalSolver->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::MIPCallback);
            }
        }

        if(env->results->isAbsoluteObjectiveGapToleranceMet() || env->results->isRelativeObjectiveGapToleranceMet()
            || checkIterationLimit() || checkUserTermination())
        {
            abort();
            return;
        }

        if(where == GRB_CB_MIPNODE && getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL)
        {
            if(env->results->getCurrentIteration()->relaxedLazyHyperplanesAdded
                < env->settings->getSetting<int>("Relaxation.MaxLazyConstraints", "Dual"))
            {
                int waitingListSize = env->dualSolver->hyperplaneWaitingList.size();

                int numModelVars = static_cast<MIPSolverGurobiSingleTree*>(env->dualSolver->MIPSolver.get())
                                       ->gurobiModel->get(GRB_IntAttr_NumVars);

                int numberOfVariables = (env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
                    ? numModelVars - 1
                    : numModelVars;

                VectorDouble solution(numberOfVariables);

                for(int i = 0; i < numberOfVariables; i++)
                {
                    solution.at(i) = getNodeRel(vars[i]);
                }

                SolutionPoint solutionRelaxed;

                if(env->problem->properties.numberOfNonlinearConstraints > 0)
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
                solutionRelaxed.objectiveValue = env->reformulatedProblem->objectiveFunction->calculateValue(solution);
                solutionRelaxed.iterFound = env->results->getCurrentIteration()->iterationNumber;
                solutionRelaxed.isRelaxedPoint = true;

                std::vector<SolutionPoint> solutionPoints = { solutionRelaxed };

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

        if(where == GRB_CB_MIPSOL)
        {
            auto currIter = env->results->getCurrentIteration();

            if(currIter->isSolved)
            {
                env->results->createIteration();
                currIter = env->results->getCurrentIteration();
                currIter->isDualProblemDiscrete = true;
                currIter->dualProblemClass = env->dualSolver->MIPSolver->getProblemClass();
            }

            int numModelVars = static_cast<MIPSolverGurobiSingleTree*>(env->dualSolver->MIPSolver.get())
                                   ->gurobiModel->get(GRB_IntAttr_NumVars);

            int numberOfVariables
                = (env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable()) ? numModelVars - 1 : numModelVars;

            VectorDouble solution(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
            {
                solution.at(i) = getSolution(vars[i]);
            }

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
            solutionCandidate.objectiveValue = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
            solutionCandidate.iterFound = env->results->getCurrentIteration()->iterationNumber;

            std::vector<SolutionPoint> candidatePoints { solutionCandidate };

            addLazyConstraint(candidatePoints);

            currIter->maxDeviation = solutionCandidate.maxDeviation.value;
            currIter->maxDeviationConstraint = solutionCandidate.maxDeviation.index;
            currIter->solutionStatus = E_ProblemSolutionStatus::Feasible;
            currIter->objectiveValue = getDoubleInfo(GRB_CB_MIPSOL_OBJ);

            currIter->numberOfExploredNodes = lastExploredNodes - env->solutionStatistics.numberOfExploredNodes;
            env->solutionStatistics.numberOfExploredNodes = lastExploredNodes;
            currIter->numberOfOpenNodes = lastOpenNodes;

            auto bounds = std::make_pair(env->results->getCurrentDualBound(), env->results->getPrimalBound());
            currIter->currentObjectiveBounds = bounds;

            if(env->settings->getSetting<bool>("Rootsearch.Use", "Primal")
                && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
            {
                taskSelectPrimalSolutionFromRootsearch.get()->run(candidatePoints);
                env->primalSolver->checkPrimalSolutionCandidates();
            }

            if(checkFixedNLPStrategy(candidatePoints.at(0)))
            {
                if(taskSelectPrimNLPOriginal)
                {
                    env->primalSolver->addFixedNLPCandidate(candidatePoints.at(0).point,
                        E_PrimalNLPSource::FirstSolution, getDoubleInfo(GRB_CB_MIPSOL_OBJ),
                        env->results->getCurrentIteration()->iterationNumber, candidatePoints.at(0).maxDeviation);

                    taskSelectPrimNLPOriginal->run();
                    env->primalSolver->fixedPrimalNLPCandidates.clear();
                }

                if(taskSelectPrimNLPReformulated)
                {
                    env->primalSolver->addFixedNLPCandidate(candidatePoints.at(0).point,
                        E_PrimalNLPSource::FirstSolution, getDoubleInfo(GRB_CB_MIPSOL_OBJ),
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

            currIter->isSolved = true;

            auto threadId = "";
            printIterationReport(candidatePoints.at(0), threadId);
        }

        if(where == GRB_CB_MIP)
        {
            lastExploredNodes = (int)getDoubleInfo(GRB_CB_MIP_NODCNT);
            lastOpenNodes = (int)getDoubleInfo(GRB_CB_MIP_NODLFT);
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi error when running main callback method", e.getMessage());
    }
}

bool GurobiCallbackSingleTree::createHyperplane(HyperplanePtr hyperplane)
{
    try
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

        GRBLinExpr expr = 0;

        for(auto& P : tmpPair.first)
        {
            expr += P.second * vars[P.first];
        }

        addLazy(expr <= -tmpPair.second);

        env->dualSolver->addGeneratedHyperplane(hyperplane);
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi error when creating lazy hyperplane", e.getMessage());
        return (false);
    }

    return (true);
}

GurobiCallbackSingleTree::GurobiCallbackSingleTree(GRBVar* xvars, EnvironmentPtr envPtr)
{
    env = envPtr;
    vars = xvars;

    showOutput = env->settings->getSetting<bool>("Console.DualSolver.Show", "Output");

    lastUpdatedPrimal = env->results->getPrimalBound();
    isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;

    env->solutionStatistics.iterationLastLazyAdded = 0;

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
        taskSelectPrimalSolutionFromRootsearch = std::make_shared<TaskSelectPrimalCandidatesFromRootsearch>(env);
    }

    lastUpdatedPrimal = env->results->getPrimalBound();
}

GurobiCallbackSingleTree::~GurobiCallbackSingleTree() { delete[] vars; }

bool GurobiCallbackSingleTree::createIntegerCut(IntegerCut& integerCut)
{
    if(!integerCut.areAllVariablesBinary)
    {
        env->output->outputDebug("        Integer cut for nonbinary variables not supported in single-tree strategy.");
        return (false);
    }

    try
    {
        GRBLinExpr expr = 0;
        size_t index = 0;

        for(auto& VAR : env->reformulatedProblem->allVariables)
        {
            if(!(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
                   || VAR->properties.type == E_VariableType::Semiinteger))
                continue;

            int variableValue = integerCut.variableValues[index];
            auto variable = vars[VAR->index];

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

        addLazy(expr >= 1);
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi error when adding lazy integer cut", e.getMessage());
        return (false);
    }

    return (true);
}

void GurobiCallbackSingleTree::addLazyConstraint(std::vector<SolutionPoint> candidatePoints)
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
            if(this->createHyperplane(hp))
                this->lastNumAddedHyperplanes++;
        }

        env->dualSolver->hyperplaneWaitingList.clear();
    }
    catch(GRBException& e)
    {
        env->output->outputError("        Gurobi error when invoking adding lazy constraint", e.getMessage());
    }
}
} // namespace SHOT
