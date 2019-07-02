/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverGurobiLazy.h"

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

MIPSolverGurobiLazy::MIPSolverGurobiLazy(EnvironmentPtr envPtr)
{
    env = envPtr;
    discreteVariablesActivated = true;

    try
    {
        gurobiEnv = std::make_shared<GRBEnv>();
        gurobiModel = std::make_shared<GRBModel>(*gurobiEnv.get());
    }
    catch(GRBException& e)
    {
        env->output->outputError("Error when initializing Gurobi:", e.getMessage());
        return;
    }

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();
}

MIPSolverGurobiLazy::~MIPSolverGurobiLazy() = default;

void MIPSolverGurobiLazy::initializeSolverSettings()
{
    MIPSolverGurobi::initializeSolverSettings();

    try
    {
        gurobiModel->set(GRB_IntParam_LazyConstraints, 1);
    }
    catch(GRBException& e)
    {
        env->output->outputError("Error when initializing parameters for linear solver", e.getMessage());
    }
}

int MIPSolverGurobiLazy::increaseSolutionLimit(int increment)
{
    gurobiModel->getEnv().set(
        GRB_IntParam_SolutionLimit, gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit) + increment);

    return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobiLazy::setSolutionLimit(long limit)
{
    if(limit > GRB_MAXINT)
        gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
    else
        gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, limit);
}

int MIPSolverGurobiLazy::getSolutionLimit() { return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit)); }

void MIPSolverGurobiLazy::checkParameters() {}

E_ProblemSolutionStatus MIPSolverGurobiLazy::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    cachedSolutionHasChanged = true;

    try
    {
        if(getDiscreteVariableStatus())
        {
            GurobiCallback gurobiCallback = GurobiCallback(gurobiModel->getVars(), env);
            gurobiModel->setCallback(&gurobiCallback);
        }

        gurobiModel->optimize();

        MIPSolutionStatus = getSolutionStatus();
    }
    catch(GRBException& e)
    {
        env->output->outputError("Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

void GurobiCallback::callback()
{
    if(where == GRB_CB_POLLING || where == GRB_CB_PRESOLVE || where == GRB_CB_SIMPLEX || where == GRB_CB_MESSAGE
        || where == GRB_CB_BARRIER)
        return;

    try
    {
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

                tmpPt.iterFound = env->results->getCurrentIteration()->iterationNumber;
                tmpPt.objectiveValue = env->problem->objectiveFunction->calculateValue(primalSolution);
                tmpPt.point = primalSolution;

                env->primalSolver->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::LazyConstraintCallback);
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
                int waitingListSize = env->dualSolver->MIPSolver->hyperplaneWaitingList.size();
                std::vector<SolutionPoint> solutionPoints(1);

                int numModelVars = static_cast<MIPSolverGurobiLazy*>(env->dualSolver->MIPSolver.get())
                                       ->gurobiModel->get(GRB_IntAttr_NumVars);

                int numberOfVariables
                    = (env->dualSolver->MIPSolver->hasAuxiliaryObjectiveVariable()) ? numModelVars - 1 : numModelVars;

                VectorDouble solution(numberOfVariables);

                for(int i = 0; i < numberOfVariables; i++)
                {
                    solution.at(i) = getNodeRel(vars[i]);
                }

                SolutionPoint tmpSolPt;

                if(env->problem->properties.numberOfNonlinearConstraints > 0)
                {
                    auto maxDev = env->reformulatedProblem->getMaxNumericConstraintValue(
                        solution, env->reformulatedProblem->nonlinearConstraints);
                    tmpSolPt.maxDeviation = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);
                }

                tmpSolPt.point = solution;
                tmpSolPt.objectiveValue = env->reformulatedProblem->objectiveFunction->calculateValue(solution);
                tmpSolPt.iterFound = env->results->getCurrentIteration()->iterationNumber;

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
                    += (env->dualSolver->MIPSolver->hyperplaneWaitingList.size() - waitingListSize);
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

            int numModelVars = static_cast<MIPSolverGurobiLazy*>(env->dualSolver->MIPSolver.get())
                                   ->gurobiModel->get(GRB_IntAttr_NumVars);

            int numberOfVariables
                = (env->dualSolver->MIPSolver->hasAuxiliaryObjectiveVariable()) ? numModelVars - 1 : numModelVars;

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

                // Remove??
                if(maxDev.normalizedValue <= env->settings->getSetting<double>("ConstraintTolerance", "Termination"))
                {
                    // return;
                }

                solutionCandidate.maxDeviation = PairIndexValue(maxDev.constraint->index, maxDev.normalizedValue);
            }

            solutionCandidate.point = solution;
            solutionCandidate.objectiveValue = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
            solutionCandidate.iterFound = env->results->getCurrentIteration()->iterationNumber;

            std::vector<SolutionPoint> candidatePoints(1);
            candidatePoints.at(0) = solutionCandidate;

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
            }

            if(checkFixedNLPStrategy(candidatePoints.at(0)))
            {
                env->primalSolver->addFixedNLPCandidate(candidatePoints.at(0).point, E_PrimalNLPSource::FirstSolution,
                    getDoubleInfo(GRB_CB_MIPSOL_OBJ), env->results->getCurrentIteration()->iterationNumber,
                    candidatePoints.at(0).maxDeviation);

                tSelectPrimNLP.get()->run();

                env->primalSolver->checkPrimalSolutionCandidates();
            }

            if(env->settings->getSetting<bool>("HyperplaneCuts.UseIntegerCuts", "Dual"))
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

            currIter->isSolved = true;

            auto threadId = "";
            printIterationReport(candidatePoints.at(0), threadId);

            if(env->results->isAbsoluteObjectiveGapToleranceMet() || env->results->isRelativeObjectiveGapToleranceMet())
            {
                abort();
                return;
            }
        }

        if(where == GRB_CB_MIP)
        {
            lastExploredNodes = (int)getDoubleInfo(GRB_CB_MIP_NODCNT);
            lastOpenNodes = (int)getDoubleInfo(GRB_CB_MIP_NODLFT);
        }

        if(where == GRB_CB_MIPSOL)
        {
            // Add current primal bound as new incumbent candidate
            auto primalBound = env->results->getPrimalBound();

            if(((isMinimization && lastUpdatedPrimal < primalBound) || (!isMinimization && primalBound > primalBound)))
            {
                auto primalSol = env->results->primalSolution;

                for(size_t i = 0; i < primalSol.size(); i++)
                {
                    setSolution(vars[i], primalSol.at(i));
                }

                for(size_t i = 0; i < env->reformulatedProblem->auxiliaryVariables.size(); i++)
                {
                    setSolution(vars[i + primalSol.size()],
                        env->reformulatedProblem->auxiliaryVariables.at(i)->calculateAuxiliaryValue(primalSol));
                }

                if(env->reformulatedProblem->auxiliaryObjectiveVariable)
                    setSolution(vars[env->reformulatedProblem->auxiliaryVariables.size() + primalSol.size()],
                        env->reformulatedProblem->auxiliaryObjectiveVariable->calculateAuxiliaryValue(primalSol));

                lastUpdatedPrimal = primalBound;
            }

            // Adds cutoff

            /*double cutOffTol = env->settings->getSetting<double>("MIP.CutOffTolerance", "Dual");

            if(isMinimization)
            {
                static_cast<MIPSolverGurobiLazy*>(env->dualSolver->MIPSolver.get())
                    ->gurobiModel->set(GRB_DoubleParam_Cutoff, primalBound + cutOffTol);

                env->output->outputDebug("     Setting cutoff value to "
                    + Utilities::toString(primalBound + cutOffTol) + " for minimization.");
            }
            else
            {
                static_cast<MIPSolverGurobiLazy*>(env->dualSolver->MIPSolver.get())
                    ->gurobiModel->set(GRB_DoubleParam_Cutoff, -primalBound - cutOffTol);

                env->output->outputDebug("     Setting cutoff value to "
                    + Utilities::toString(-primalBound - cutOffTol) + " for minimization.");
            }*/
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("Gurobi error when running main callback method", e.getMessage());
    }
}

void GurobiCallback::createHyperplane(Hyperplane hyperplane)
{
    try
    {
        auto currIter = env->results->getCurrentIteration(); // The unsolved new iteration
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
                env->output->outputError("     Warning: hyperplane for constraint "
                    + std::to_string(hyperplane.sourceConstraint->index)
                    + " not generated, NaN found in linear terms for variable "
                    + env->problem->getVariable(E.index)->name);
                hyperplaneIsOk = false;
                break;
            }
        }

        if(hyperplaneIsOk)
        {
            GRBLinExpr expr = 0;

            for(auto& P : tmpPair.first)
            {
                expr += +(P.value) * (vars[P.index]);
            }

            addLazy(expr <= -tmpPair.second);

            // std::string identifier = env->dualSolver->MIPSolver->getConstraintIdentifier(hyperplane.source);

            // if(hyperplane.sourceConstraint != nullptr)
            //    identifier = identifier + "_" + hyperplane.sourceConstraint->name;

            env->dualSolver->addGeneratedHyperplane(hyperplane);

            currIter->numHyperplanesAdded++;
            currIter->totNumHyperplanes++;
        }
    }
    catch(GRBException& e)
    {
        env->output->outputError("Gurobi error when creating lazy hyperplane", e.getMessage());
    }
}

GurobiCallback::GurobiCallback(GRBVar* xvars, EnvironmentPtr envPtr)
{
    env = envPtr;
    vars = xvars;

    isMinimization = env->reformulatedProblem->objectiveFunction->properties.isMinimize;

    env->solutionStatistics.iterationLastLazyAdded = 0;

    cbCalls = 0;

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("CutStrategy", "Dual"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            tUpdateInteriorPoint = std::make_shared<TaskUpdateInteriorPoint>(env);
            taskSelectHPPts
                = std::shared_ptr<TaskSelectHyperplanePointsESH>(std::make_shared<TaskSelectHyperplanePointsESH>(env));
        }
        else
        {
            taskSelectHPPts
                = std::shared_ptr<TaskSelectHyperplanePointsECP>(std::make_shared<TaskSelectHyperplanePointsECP>(env));
        }
    }

    tSelectPrimNLP
        = std::shared_ptr<TaskSelectPrimalCandidatesFromNLP>(std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env));

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        taskSelectHPPtsByObjectiveRootsearch = std::shared_ptr<TaskSelectHyperplanePointsByObjectiveRootsearch>(
            std::make_shared<TaskSelectHyperplanePointsByObjectiveRootsearch>(env));
    }

    if(env->settings->getSetting<bool>("Rootsearch.Use", "Primal")
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        taskSelectPrimalSolutionFromRootsearch = std::shared_ptr<TaskSelectPrimalCandidatesFromRootsearch>(
            std::make_shared<TaskSelectPrimalCandidatesFromRootsearch>(env));
    }

    lastUpdatedPrimal = env->results->getPrimalBound();
}

void GurobiCallback::createIntegerCut(VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes)
{
    try
    {
        GRBLinExpr expr = 0;

        for(int I : binaryIndexesOnes)
        {
            expr += 1.0 * vars[I];
        }

        for(int I : binaryIndexesZeroes)
        {
            expr += (1 - 1.0 * vars[I]);
        }

        addLazy(expr <= binaryIndexesOnes.size() + binaryIndexesZeroes.size() - 1.0);

        env->solutionStatistics.numberOfIntegerCuts++;
    }
    catch(GRBException& e)
    {
        env->output->outputError("Gurobi error when adding lazy integer cut", e.getMessage());
    }
}

void GurobiCallback::addLazyConstraint(std::vector<SolutionPoint> candidatePoints)
{
    try
    {
        this->cbCalls++;

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
            this->createHyperplane(hp);
            this->lastNumAddedHyperplanes++;
        }

        env->dualSolver->MIPSolver->hyperplaneWaitingList.clear();
    }
    catch(GRBException& e)
    {
        env->output->outputError("Gurobi error when invoking adding lazy constraint", e.getMessage());
    }
}
} // namespace SHOT