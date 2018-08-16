/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverGurobiLazy.h"

MIPSolverGurobiLazy::MIPSolverGurobiLazy(EnvironmentPtr envPtr)
{
    env = envPtr;
    discreteVariablesActivated = true;

    try
    {
        gurobiEnv = new GRBEnv();
        gurobiModel = new GRBModel(*gurobiEnv);
    }
    catch (GRBException &e)
    {
        {
            env->output->outputError("Error when initializing Gurobi:", e.getMessage());
        }

        return;
    }

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();
}

MIPSolverGurobiLazy::~MIPSolverGurobiLazy()
{
}

void MIPSolverGurobiLazy::initializeSolverSettings()
{
    MIPSolverGurobi::initializeSolverSettings();

    try
    {
        gurobiModel->set(GRB_IntParam_LazyConstraints, 1);
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when initializing parameters for linear solver", e.getMessage());
    }
}

int MIPSolverGurobiLazy::increaseSolutionLimit(int increment)
{
    gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit,
                              gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit) + increment);

    return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobiLazy::setSolutionLimit(long limit)
{
    if (limit > GRB_MAXINT)
        gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
    else
        gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, limit);
}

int MIPSolverGurobiLazy::getSolutionLimit()
{
    return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobiLazy::checkParameters()
{
}

E_ProblemSolutionStatus MIPSolverGurobiLazy::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    cachedSolutionHasChanged = true;

    try
    {
        GurobiCallback gurobiCallback = GurobiCallback(gurobiModel->getVars());
        gurobiModel->setCallback(&gurobiCallback);
        gurobiModel->optimize();

        MIPSolutionStatus = getSolutionStatus();
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

void GurobiCallback::callback()
{
    if (where == GRB_CB_POLLING || where == GRB_CB_PRESOLVE || where == GRB_CB_SIMPLEX || where == GRB_CB_MESSAGE || where == GRB_CB_BARRIER)
        return;

    try
    {
        // Check if better dual bound
        double tmpDualObjBound;

        if (where == GRB_CB_MIP || where == GRB_CB_MIPSOL || where == GRB_CB_MIPNODE)
        {
            switch (where)
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

            if ((isMinimization && tmpDualObjBound > env->process->getDualBound()) || (!isMinimization && tmpDualObjBound < env->process->getDualBound()))
            {
                DoubleVector doubleSolution; // Empty since we have no point

                DualSolution sol =
                    {doubleSolution, E_DualSolutionSource::MIPSolverBound, tmpDualObjBound, env->process->getCurrentIteration()->iterationNumber};
                env->process->addDualSolutionCandidate(sol);
            }
        }

        if (where == GRB_CB_MIPSOL)
        {
            // Check for new primal solution
            double tmpPrimalObjBound = getDoubleInfo(GRB_CB_MIPSOL_OBJ);

            if ((tmpPrimalObjBound < 1e100) && ((isMinimization && tmpPrimalObjBound < env->process->getPrimalBound()) || (!isMinimization && tmpPrimalObjBound > env->process->getPrimalBound())))
            {
                DoubleVector primalSolution(numVar);

                for (int i = 0; i < numVar; i++)
                {
                    primalSolution.at(i) = getSolution(vars[i]);
                }

                SolutionPoint tmpPt;
                tmpPt.iterFound = env->process->getCurrentIteration()->iterationNumber;
                tmpPt.maxDeviation = env->model->originalProblem->getMostDeviatingConstraint(
                    primalSolution);
                tmpPt.objectiveValue = env->model->originalProblem->calculateOriginalObjectiveValue(
                    primalSolution);
                tmpPt.point = primalSolution;

                env->process->addPrimalSolutionCandidate(tmpPt, E_PrimalSolutionSource::LazyConstraintCallback);
            }
        }

        if (env->process->isAbsoluteObjectiveGapToleranceMet() || env->process->isRelativeObjectiveGapToleranceMet() || checkIterationLimit())
        {
            abort();
            return;
        }

        if (where == GRB_CB_MIPNODE && getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL)
        {
            if (env->process->getCurrentIteration()->relaxedLazyHyperplanesAdded < env->settings->getIntSetting("Relaxation.MaxLazyConstraints", "Dual"))
            {
                int waitingListSize = env->process->hyperplaneWaitingList.size();
                std::vector<SolutionPoint> solutionPoints(1);

                DoubleVector solution(numVar);

                for (int i = 0; i < numVar; i++)
                {
                    solution.at(i) = getNodeRel(vars[i]);
                }

                auto mostDevConstr = env->model->originalProblem->getMostDeviatingConstraint(solution);

                SolutionPoint tmpSolPt;

                tmpSolPt.point = solution;
                tmpSolPt.objectiveValue = env->model->originalProblem->calculateOriginalObjectiveValue(
                    solution);
                tmpSolPt.iterFound = env->process->getCurrentIteration()->iterationNumber;
                tmpSolPt.maxDeviation = mostDevConstr;

                solutionPoints.at(0) = tmpSolPt;

                if (static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting(
                        "CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
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
        }

        if (where == GRB_CB_MIPSOL)
        {
            auto currIter = env->process->getCurrentIteration();

            if (currIter->isSolved)
            {
                env->process->createIteration();
                currIter = env->process->getCurrentIteration();
            }

            DoubleVector solution(numVar);

            for (int i = 0; i < numVar; i++)
            {
                solution.at(i) = getSolution(vars[i]);
            }

            auto mostDevConstr = env->model->originalProblem->getMostDeviatingConstraint(solution);

            SolutionPoint solutionCandidate;

            solutionCandidate.point = solution;
            solutionCandidate.objectiveValue = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
            solutionCandidate.iterFound = env->process->getCurrentIteration()->iterationNumber;
            solutionCandidate.maxDeviation = mostDevConstr;

            std::vector<SolutionPoint> candidatePoints(1);
            candidatePoints.at(0) = solutionCandidate;

            addLazyConstraint(candidatePoints);

            currIter->maxDeviation = mostDevConstr.value;
            currIter->maxDeviationConstraint = mostDevConstr.idx;
            currIter->solutionStatus = E_ProblemSolutionStatus::Feasible;
            currIter->objectiveValue = getDoubleInfo(GRB_CB_MIPSOL_OBJ);

            currIter->numberOfExploredNodes = lastExploredNodes - env->solutionStatistics.numberOfExploredNodes;
            env->solutionStatistics.numberOfExploredNodes = lastExploredNodes;
            currIter->numberOfOpenNodes = lastOpenNodes;

            auto bounds = std::make_pair(env->process->getDualBound(), env->process->getPrimalBound());
            currIter->currentObjectiveBounds = bounds;

            if (env->settings->getBoolSetting("Linesearch.Use", "Primal"))
            {
                taskSelectPrimalSolutionFromLinesearch.get()->run(candidatePoints);
            }

            if (checkFixedNLPStrategy(candidatePoints.at(0)))
            {
                env->process->addPrimalFixedNLPCandidate(candidatePoints.at(0).point,
                                                                      E_PrimalNLPSource::FirstSolution, getDoubleInfo(GRB_CB_MIPSOL_OBJ), env->process->getCurrentIteration()->iterationNumber,
                                                                      candidatePoints.at(0).maxDeviation);

                tSelectPrimNLP.get()->run();

                env->process->checkPrimalSolutionCandidates();
            }

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

            currIter->isSolved = true;

            auto threadId = "";
            printIterationReport(candidatePoints.at(0), threadId);

            if (env->process->isAbsoluteObjectiveGapToleranceMet() || env->process->isRelativeObjectiveGapToleranceMet())
            {
                abort();
                return;
            }
        }

        if (where == GRB_CB_MIP)
        {
            lastExploredNodes = (int)getDoubleInfo(GRB_CB_MIP_NODCNT);
            lastOpenNodes = (int)getDoubleInfo(GRB_CB_MIP_NODLFT);
        }

        if (where == GRB_CB_MIPSOL)
        {
            // Add current primal bound as new incumbent candidate
            auto primalBound = env->process->getPrimalBound();

            if (((isMinimization && lastUpdatedPrimal < primalBound) || (!isMinimization && primalBound > primalBound)))
            {
                auto primalSol = env->process->primalSolution;

                DoubleVector primalSolution(numVar);

                for (int i = 0; i < numVar; i++)
                {
                    setSolution(vars[i], primalSol.at(i));
                }

                lastUpdatedPrimal = primalBound;
            }

            // Adds cutoff

            double cutOffTol = env->settings->getDoubleSetting("MIP.CutOffTolerance", "Dual");

            if (isMinimization)
            {
                static_cast<MIPSolverGurobiLazy *>(env->dualSolver.get())->gurobiModel->set(GRB_DoubleParam_Cutoff, primalBound + cutOffTol);

                env->output->outputInfo(
                    "     Setting cutoff value to " + UtilityFunctions::toString(primalBound + cutOffTol) + " for minimization.");
            }
            else
            {
                static_cast<MIPSolverGurobiLazy *>(env->dualSolver.get())->gurobiModel->set(GRB_DoubleParam_Cutoff, -primalBound - cutOffTol);

                env->output->outputInfo(
                    "     Setting cutoff value to " + UtilityFunctions::toString(-primalBound - cutOffTol) + " for minimization.");
            }
        }
    }
    catch (GRBException &e)
    {
        env->output->outputError("Gurobi error when running main callback method", e.getMessage());
    }
}

void GurobiCallback::createHyperplane(Hyperplane hyperplane)
{
    try
    {
        auto currIter = env->process->getCurrentIteration(); // The unsolved new iteration
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

        if (hyperplaneIsOk)
        {
            GeneratedHyperplane genHyperplane;

            GRBLinExpr expr = 0;

            for (int i = 0; i < tmpPair.first.size(); i++)
            {
                expr += +(tmpPair.first.at(i).value) * (vars[tmpPair.first.at(i).idx]);
            }

            addLazy(expr <= -tmpPair.second);

            int constrIndex = 0;
            genHyperplane.generatedConstraintIndex = constrIndex;
            genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
            genHyperplane.generatedPoint = hyperplane.generatedPoint;
            genHyperplane.source = hyperplane.source;
            genHyperplane.generatedIter = currIter->iterationNumber;
            genHyperplane.isLazy = false;
            genHyperplane.isRemoved = false;

            //env->dualSolver->generatedHyperplanes.push_back(genHyperplane);

            currIter->numHyperplanesAdded++;
            currIter->totNumHyperplanes++;
        }
    }
    catch (GRBException &e)
    {
        env->output->outputError("Gurobi error when creating lazy hyperplane", e.getMessage());
    }
}

GurobiCallback::GurobiCallback(GRBVar *xvars)
{
    vars = xvars;

    isMinimization = env->model->originalProblem->isTypeOfObjectiveMinimize();

    env->solutionStatistics.iterationLastLazyAdded = 0;

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

    lastUpdatedPrimal = env->process->getPrimalBound();

    numVar = (static_cast<MIPSolverGurobiLazy *>(env->dualSolver.get()))->gurobiModel->get(GRB_IntAttr_NumVars);
}

void GurobiCallback::createIntegerCut(std::vector<int> binaryIndexes)
{
    try
    {
        GRBLinExpr expr = 0;

        for (int i = 0; i < binaryIndexes.size(); i++)
        {
            expr += vars[binaryIndexes.at(i)];
        }

        addLazy(expr <= binaryIndexes.size() - 1.0);

        env->solutionStatistics.numberOfIntegerCuts++;
    }
    catch (GRBException &e)
    {
        env->output->outputError("Gurobi error when adding lazy integer cut", e.getMessage());
    }
}

void GurobiCallback::addLazyConstraint(std::vector<SolutionPoint> candidatePoints)
{
    try
    {
        this->cbCalls++;

        if (static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
        {
            tUpdateInteriorPoint->run();

            if (static_cast<ES_RootsearchConstraintStrategy>(env->settings->getIntSetting("ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
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

        for (auto hp : env->process->hyperplaneWaitingList)
        {
            this->createHyperplane(hp);
            this->lastNumAddedHyperplanes++;
        }

        env->process->hyperplaneWaitingList.clear();
    }
    catch (GRBException &e)
    {
        env->output->outputError("Gurobi error when invoking adding lazy constraint", e.getMessage());
    }
}
