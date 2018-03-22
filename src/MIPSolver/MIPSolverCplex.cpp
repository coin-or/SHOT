/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCplex.h"

MIPSolverCplex::MIPSolverCplex()
{
    discreteVariablesActivated = true;

    cplexModel = IloModel(cplexEnv);

    cplexVars = IloNumVarArray(cplexEnv);
    cplexConstrs = IloRangeArray(cplexEnv);

    cachedSolutionHasChanged = true;

    isVariablesFixed = false;

    checkParameters();
    modelUpdated = false;
}

MIPSolverCplex::~MIPSolverCplex()
{
    iterDurations.clear();
    cplexVarConvers.clear();
    cplexModel.end();
    cplexVars.end();
    cplexConstrs.end();
    cplexInstance.end();
    cplexEnv.end();
}

bool MIPSolverCplex::createLinearProblem(OptProblem *origProblem)
{
    originalProblem = origProblem;

    auto numVar = origProblem->getNumberOfVariables();
    auto tmpLBs = origProblem->getVariableLowerBounds();
    auto tmpUBs = origProblem->getVariableUpperBounds();
    auto tmpNames = origProblem->getVariableNames();
    auto tmpTypes = origProblem->getVariableTypes();

    int numCon = origProblem->getNumberOfConstraints();
    if (origProblem->isObjectiveFunctionNonlinear())
        numCon--; // Only want the number of original constraints and not the objective function

    // Now creating the variables
    for (int i = 0; i < numVar; i++)
    {
        if (tmpTypes.at(i) == 'C')
        {
            cplexVars.add(IloNumVar(cplexEnv, tmpLBs.at(i), tmpUBs.at(i), ILOFLOAT, tmpNames.at(i).c_str()));
        }
        else if (tmpTypes.at(i) == 'I')
        {
            cplexVars.add(IloNumVar(cplexEnv, tmpLBs.at(i), tmpUBs.at(i), ILOINT, tmpNames.at(i).c_str()));
        }
        else if (tmpTypes.at(i) == 'B')
        {
            cplexVars.add(IloNumVar(cplexEnv, tmpLBs.at(i), tmpUBs.at(i), ILOBOOL, tmpNames.at(i).c_str()));
        }
        else if (tmpTypes.at(i) == 'D')
        {
            cplexVars.add(IloSemiContVar(cplexEnv, tmpLBs.at(i), tmpUBs.at(i), ILOFLOAT, tmpNames.at(i).c_str()));
        }
        else
        {
            ProcessInfo::getInstance().outputWarning(
                "Error variable type " + to_string(tmpTypes.at(i)) + " for " + tmpNames.at(i));
        }
    }

    cplexModel.add(cplexVars);

    // Now creating the objective function

    IloExpr objExpr(cplexEnv);

    auto tmpObjPairs = origProblem->getObjectiveFunctionVarCoeffPairs();

    for (int i = 0; i < tmpObjPairs.size(); i++)
    {
        objExpr += tmpObjPairs.at(i).second * cplexVars[tmpObjPairs.at(i).first];
    }

    // Add quadratic terms in the objective if they exist (and the strategy is to solve QPs)
    if (origProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
    {
        auto quadTerms = origProblem->getQuadraticTermsInConstraint(-1);

        for (auto T : quadTerms)
        {
            objExpr += T->coef * cplexVars[T->idxOne] * cplexVars[T->idxTwo];
        }
    }

    double objConstant = origProblem->getObjectiveConstant();
    if (objConstant != 0.0)
        objExpr += objConstant;

    if (origProblem->isTypeOfObjectiveMinimize())
    {
        cplexModel.add(IloMinimize(cplexEnv, objExpr));
    }
    else
    {
        cplexModel.add(IloMaximize(cplexEnv, objExpr));
    }

    objExpr.end();

    // Now creating the constraints

    int row_nonz = 0;
    int obj_nonz = 0;
    int varIdx = 0;

    SparseMatrix *m_linearConstraintCoefficientsInRowMajor =
        origProblem->getProblemInstance()->getLinearConstraintCoefficientsInRowMajor();

    auto constrTypes = origProblem->getProblemInstance()->getConstraintTypes();
    auto constrNames = origProblem->getProblemInstance()->getConstraintNames();
    auto constrLBs = origProblem->getProblemInstance()->getConstraintLowerBounds();
    auto constrUBs = origProblem->getProblemInstance()->getConstraintUpperBounds();

    for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
    {
        // Only use constraints that don't contain a nonlinear part (may include a quadratic part)
        if (!origProblem->isConstraintNonlinear(rowIdx))
        {
            IloExpr expr(cplexEnv);

            if (origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients != NULL && origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients->numberOfValues > 0)
            {
                row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1] - m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];

                for (int j = 0; j < row_nonz; j++)
                {
                    double val =
                        m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
                    varIdx =
                        m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];

                    expr += val * cplexVars[varIdx];
                }
            }

            // Add quadratic terms if they exist and have been defined as quadratic and not nonlinear
            auto quadTerms = origProblem->getQuadraticTermsInConstraint(rowIdx);

            for (auto T : quadTerms)
            {
                expr += T->coef * cplexVars[T->idxOne] * cplexVars[T->idxTwo];
            }

            expr += origProblem->getProblemInstance()->instanceData->constraints->con[rowIdx]->constant;

            // Add the constraint
            if (constrTypes[rowIdx] == 'L')
            {
                IloRange tmpRange = IloRange(cplexEnv, -IloInfinity, expr, constrUBs[rowIdx],
                                             constrNames[rowIdx].c_str());
                cplexConstrs.add(tmpRange);
            }
            else if (constrTypes[rowIdx] == 'G')
            {
                IloRange tmpRange = IloRange(cplexEnv, constrLBs[rowIdx], expr, IloInfinity,
                                             constrNames[rowIdx].c_str());
                cplexConstrs.add(tmpRange);
            }
            else if (constrTypes[rowIdx] == 'E')
            {
                IloRange tmpRange = IloRange(cplexEnv, constrLBs[rowIdx], expr, constrUBs[rowIdx],
                                             constrNames[rowIdx].c_str());
                cplexConstrs.add(tmpRange);
            }
            else
            {
            }

            expr.end();
        }
    }

    cplexModel.add(cplexConstrs);

    try
    {
        cplexInstance = IloCplex(cplexModel);
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Cplex exception caught when creating model", e.getMessage());
        return (false);
    }

    setSolutionLimit(9223372036800000000);

    return (true);
}

void MIPSolverCplex::initializeSolverSettings()
{
    try
    {
        // Disable Cplex output
        if (!Settings::getInstance().getBoolSetting("Console.DualSolver.Show", "Output"))
        {
            cplexInstance.setOut(cplexEnv.getNullStream());

            if (Settings::getInstance().getIntSetting("Console.LogLevel", "Output") <= static_cast<int>(ENUM_OUTPUT_LEVEL_summary))
            {
                cplexInstance.setWarning(cplexEnv.getNullStream());
            }
        }

        cplexInstance.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, Settings::getInstance().getDoubleSetting("ObjectiveGap.Relative", "Termination") / 1.0);
        cplexInstance.setParam(IloCplex::Param::MIP::Tolerances::AbsMIPGap, Settings::getInstance().getDoubleSetting("ObjectiveGap.Absolute", "Termination") / 1.0);

        cplexInstance.setParam(IloCplex::IntSolLim, 2100000000);

        cplexInstance.setParam(IloCplex::SolnPoolIntensity, Settings::getInstance().getIntSetting("Cplex.SolnPoolIntensity", "Subsolver")); // Don't use 3 with heuristics
        cplexInstance.setParam(IloCplex::SolnPoolReplace, Settings::getInstance().getIntSetting("Cplex.SolnPoolReplace", "Subsolver"));
        cplexInstance.setParam(IloCplex::SolnPoolGap, Settings::getInstance().getDoubleSetting("Cplex.SolnPoolGap", "Subsolver"));
        cplexInstance.setParam(IloCplex::SolnPoolCapacity, Settings::getInstance().getIntSetting("MIP.SolutionPool.Capacity", "Dual"));

        cplexInstance.setParam(IloCplex::Probe, Settings::getInstance().getIntSetting("Cplex.Probe", "Subsolver"));
        cplexInstance.setParam(IloCplex::MIPEmphasis, Settings::getInstance().getIntSetting("Cplex.MIPEmphasis", "Subsolver"));

        cplexInstance.setParam(IloCplex::ParallelMode, Settings::getInstance().getIntSetting("Cplex.ParallelMode", "Subsolver"));
        cplexInstance.setParam(IloCplex::Threads, Settings::getInstance().getIntSetting("MIP.NumberOfThreads", "Dual"));

        cplexInstance.setParam(IloCplex::NumericalEmphasis, Settings::getInstance().getIntSetting("Cplex.NumericalEmphasis", "Subsolver"));
        cplexInstance.setParam(IloCplex::MemoryEmphasis, Settings::getInstance().getIntSetting("Cplex.MemoryEmphasis", "Subsolver"));

        // Options for using swap file
        cplexInstance.setParam(IloCplex::WorkDir, Settings::getInstance().getStringSetting("Cplex.WorkDir", "Subsolver").c_str());
        cplexInstance.setParam(IloCplex::WorkMem, Settings::getInstance().getDoubleSetting("Cplex.WorkMem", "Subsolver"));
        cplexInstance.setParam(IloCplex::NodeFileInd, Settings::getInstance().getIntSetting("Cplex.NodeFileInd", "Subsolver"));

        //cplexInstance.setParam(IloCplex::Param::Tune::Measure, CPX_TUNE_AVERAGE);
        //cplexInstance.setParam(IloCplex::Param::Tune::TimeLimit, 10);

        //cplexInstance.setParam(IloCplex::EpRHS, 10 ^ (-5));
        //cplexInstance.setParam(IloCplex::EpInt, 10 ^ (-6));
        //cplexInstance.setParam(IloCplex::EpOpt, 1 ^ (-9));
        //cplexInstance.setParam(IloCplex::EpAGap, 10 ^ (-14));

        //	cplexInstance.setParam(IloCplex::PopulateLim, 10);

        //cplexInstance.setParam(IloCplex::SolnPoolGap, 0);

        //cplexInstance.setParam(IloCplex::Param::MIP::Pool::RelGap, 0.1);

        /*plexInstance.setParam(IloCplex::PreInd, 0);

		 cplexInstance.setParam(IloCplex::RelaxPreInd, 0);
		 cplexInstance.setParam(IloCplex::PreslvNd, -1);
		 */
        //cplexInstance.setParam(IloCplex::EpMrk, 0.9);
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Cplex error when initializing parameters for linear solver",
                                               e.getMessage());
    }
}

int MIPSolverCplex::addLinearConstraint(std::vector<IndexValuePair> elements, double constant, bool isGreaterThan)
{
    try
    {
        IloExpr expr(cplexEnv);

        for (int i = 0; i < elements.size(); i++)
        {
            expr += elements.at(i).value * cplexVars[elements.at(i).idx];
        }

        if (isGreaterThan)
        {
            IloRange tmpRange(cplexEnv, -constant, expr);
            cplexConstrs.add(tmpRange);
            cplexModel.add(tmpRange);
        }
        else
        {
            IloRange tmpRange(cplexEnv, -IloInfinity, expr, -constant);
            cplexConstrs.add(tmpRange);
            cplexModel.add(tmpRange);
        }

        modelUpdated = true;

        expr.end();
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when adding linear constraint", e.getMessage());

        return (-1);
    }

    return (cplexInstance.getNrows() - 1);
}

void MIPSolverCplex::activateDiscreteVariables(bool activate)
{
    auto variableTypes = originalProblem->getVariableTypes();

    try
    {
        for (int i = 0; i < cplexVarConvers.size(); i++)
        {
            cplexVarConvers.at(i).end();
        }

        cplexVarConvers.clear();

        if (activate)
        {
            for (int i = 0; i < originalProblem->getNumberOfVariables(); i++)
            {
                if (variableTypes.at(i) == 'I')
                {
                    auto tmpVar = cplexVars[i];
                    auto tmpConv = IloConversion(cplexEnv, tmpVar, ILOINT);
                    cplexModel.add(tmpConv);
                    cplexVarConvers.push_back(tmpConv);
                }
                else if (variableTypes.at(i) == 'B')
                {
                    auto tmpVar = cplexVars[i];
                    auto tmpConv = IloConversion(cplexEnv, tmpVar, ILOBOOL);
                    cplexModel.add(tmpConv);
                    cplexVarConvers.push_back(tmpConv);
                }
            }

            discreteVariablesActivated = true;
        }
        else
        {
            for (int i = 0; i < originalProblem->getNumberOfVariables(); i++)
            {
                if (variableTypes.at(i) == 'I' || variableTypes.at(i) == 'B')
                {
                    auto tmpVar = cplexVars[i];
                    auto tmpConv = IloConversion(cplexEnv, tmpVar, ILOFLOAT);
                    cplexModel.add(tmpConv);
                    cplexVarConvers.push_back(tmpConv);
                }
            }

            discreteVariablesActivated = false;
        }
    }
    catch (IloException &e)
    {
        if (activate)
            ProcessInfo::getInstance().outputError("Error when activating discrete variables",
                                                   e.getMessage());
    }
}

E_ProblemSolutionStatus MIPSolverCplex::getSolutionStatus()
{
    E_ProblemSolutionStatus MIPSolutionStatus;

    try
    {
        auto status = cplexInstance.getCplexStatus();

        if (status == IloCplex::CplexStatus::Optimal || status == IloCplex::CplexStatus::OptimalTol)
        {
            auto statusInstance = cplexInstance.getStatus();

            if (statusInstance == IloAlgorithm::Status::Optimal)
            {
                MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
            }
            else
            {
                MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
            }
        }
        else if (status == IloCplex::CplexStatus::Infeasible)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
        }
        else if (status == IloCplex::CplexStatus::InfOrUnbd)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
        }
        else if (status == IloCplex::CplexStatus::Unbounded)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
        }
        else if (status == IloCplex::CplexStatus::NodeLimFeas)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::NodeLimit;
        }
        else if (status == IloCplex::CplexStatus::AbortTimeLim || status == IloCplex::CplexStatus::AbortDetTimeLim)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
        }
        else if (status == IloCplex::CplexStatus::SolLim)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
        }
        else if (status == IloCplex::CplexStatus::AbortUser)
        {
            MIPSolutionStatus = E_ProblemSolutionStatus::Abort;
        }
        else
        {
            ProcessInfo::getInstance().outputError("MIP solver return status " + to_string(status));
            MIPSolutionStatus = E_ProblemSolutionStatus::Error;
        }
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when obtaining solution status", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

E_ProblemSolutionStatus MIPSolverCplex::solveProblem()
{
    startTimer();

    E_ProblemSolutionStatus MIPSolutionStatus;
    cachedSolutionHasChanged = true;

    try
    {
        if (modelUpdated) // EDIT: march 2016, should this be used??
        {
            cplexInstance.extract(cplexModel);

            modelUpdated = false;
        }

        double timeStart = ProcessInfo::getInstance().getElapsedTime("Total");

        cplexInstance.solve();
        double timeEnd = ProcessInfo::getInstance().getElapsedTime("Total");

        iterDurations.push_back(timeEnd - timeStart);
        MIPSolutionStatus = getSolutionStatus();
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    stopTimer();

    return (MIPSolutionStatus);
}

int MIPSolverCplex::increaseSolutionLimit(int increment)
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

void MIPSolverCplex::setSolutionLimit(long limit)
{

    try
    {
        cplexInstance.setParam(IloCplex::IntSolLim, limit);
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when setting solution limit", e.getMessage());
    }
}

int MIPSolverCplex::getSolutionLimit()
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

std::vector<double> MIPSolverCplex::getVariableSolution(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus() && ProcessInfo::getInstance().originalProblem->isProblemDiscrete();
    int numVar = cplexVars.getSize();
    std::vector<double> solution(numVar);

    IloNumArray tmpSolsCplex(cplexEnv);

    try
    {
        if (isMIP)
        {
            cplexInstance.getValues(cplexVars, tmpSolsCplex, solIdx);
        }
        else
        {
            cplexInstance.getValues(cplexVars, tmpSolsCplex);
        }

        for (int i = 0; i < numVar; i++)
        {
            solution.at(i) = tmpSolsCplex[i];
        }
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when reading solution with index " + to_string(solIdx),
                                               e.getMessage());
    }

    tmpSolsCplex.end();
    return (solution);
}

int MIPSolverCplex::getNumberOfSolutions()
{
    int numSols = 0;
    bool isMIP = getDiscreteVariableStatus();

    try
    {
        if (ProcessInfo::getInstance().originalProblem->isProblemDiscrete() && isMIP)
        {
            numSols = cplexInstance.getSolnPoolNsolns();
        }
        else
        {
            numSols = 1;
        }
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when obtaining number of solutions", e.getMessage());
    }

    return (numSols);
}

double MIPSolverCplex::getObjectiveValue(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus() && ProcessInfo::getInstance().originalProblem->isProblemDiscrete();

    double objVal = NAN;

    if (!isMIP && solIdx > 0) // LP problems only have one solution!
    {
        ProcessInfo::getInstance().outputError(
            "Cannot obtain solution with index " + to_string(solIdx) + " since the problem is LP/QP!");
        return (objVal);
    }

    try
    {
        if (isMIP)
        {
            objVal = cplexInstance.getObjValue(solIdx);
        }
        else
        {
            objVal = cplexInstance.getObjValue();
        }
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError(
            "Error when obtaining objective value for solution index " + to_string(solIdx), e.getMessage());
    }

    return (objVal);
}

void MIPSolverCplex::populateSolutionPool()
{
    ProcessInfo::getInstance().startTimer("PopulateSolutionPool");
    double initialPopulateTimeLimit = 0.5;
    double timeLimitIncreaseFactor = 2.0;

    try
    {
        int poolSizeBefore = cplexInstance.getSolnPoolNsolns();

        if (ProcessInfo::getInstance().getCurrentIteration()->iterationNumber == 0)
        {
            setTimeLimit(initialPopulateTimeLimit);
        }
        else
        {
            // Note that the vector elements are rearranged each iteration
            std::nth_element(iterDurations.begin(), iterDurations.begin() + iterDurations.size() / 2,
                             iterDurations.end());

            double newTimeLimit = timeLimitIncreaseFactor * iterDurations[iterDurations.size() / 2];
            setTimeLimit(newTimeLimit);
        }

        double newSolnPoolGap = min(1.0e+75, ProcessInfo::getInstance().getAbsoluteObjectiveGap());
        cplexInstance.setParam(IloCplex::SolnPoolGap, newSolnPoolGap);

        cplexInstance.populate();

        int poolSizeAfter = cplexInstance.getSolnPoolNsolns();

        if (poolSizeAfter > poolSizeBefore)
        {
            ProcessInfo::getInstance().outputInfo(
                "     Solution pool populated from: " + to_string(poolSizeBefore) + " to " + to_string(poolSizeAfter));
        }
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when populating solution pool", e.getMessage());
        ProcessInfo::getInstance().stopTimer("PopulateSolutionPool");
    }

    ProcessInfo::getInstance().stopTimer("PopulateSolutionPool");
}

void MIPSolverCplex::setTimeLimit(double seconds)
{
    try
    {
        if (seconds > 0)
        {
            cplexInstance.setParam(IloCplex::TiLim, seconds);
        }
        else
        {
            cplexInstance.setParam(IloCplex::TiLim, 0.01);
        }
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when setting time limit", e.getMessage());
    }
}

void MIPSolverCplex::setCutOff(double cutOff)
{
    double cutOffTol = Settings::getInstance().getDoubleSetting("MIP.CutOffTolerance", "Dual");

    try
    {
        if (originalProblem->isTypeOfObjectiveMinimize())
        {
            cplexInstance.setParam(IloCplex::CutUp, cutOff + cutOffTol);

            ProcessInfo::getInstance().outputInfo(
                "     Setting cutoff value to " + to_string(cutOff + cutOffTol) + " for minimization.");
        }
        else
        {
            cplexInstance.setParam(IloCplex::CutLo, cutOff - cutOffTol);
            ProcessInfo::getInstance().outputInfo(
                "     Setting cutoff value to " + to_string(cutOff - cutOffTol) + " for maximization.");
        }
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when setting cut off value", e.getMessage());
    }
}

void MIPSolverCplex::addMIPStart(std::vector<double> point)
{
    IloNumArray startVal(cplexEnv);

    int numVar = cplexVars.getSize();

    for (int i = 0; i < numVar; i++)
    {
        startVal.add(point.at(i));
    }

    try
    {
        cplexInstance.addMIPStart(cplexVars, startVal);
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when adding MIP starting point", e.getMessage());
    }

    startVal.end();

    ProcessInfo::getInstance().outputInfo("     Added MIP starting point.");
}

void MIPSolverCplex::deleteMIPStarts()
{
    int numStarts = cplexInstance.getNMIPStarts();

    if (numStarts > 0)
    {
        try
        {
            cplexInstance.deleteMIPStarts(0, numStarts);
        }
        catch (IloException &e)
        {
            ProcessInfo::getInstance().outputError("Error when deleting MIP starting points", e.getMessage());
        }

        ProcessInfo::getInstance().outputDebug("    Deleted " + to_string(numStarts) + " MIP starting points.");
    }
}

void MIPSolverCplex::writeProblemToFile(std::string filename)
{
    try
    {
        if (modelUpdated)
        {
            //Extract the model if we have updated the constraints
            cplexInstance.extract(cplexModel);
            modelUpdated = false;
        }

        cplexInstance.exportModel(filename.c_str());
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when saving model to file", e.getMessage());
    }
}

void MIPSolverCplex::fixVariable(int varIndex, double value)
{
    updateVariableBound(varIndex, value, value);
}

void MIPSolverCplex::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
    try
    {
        cplexVars[varIndex].setBounds(lowerBound, upperBound);
        modelUpdated = true;
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError(
            "Error when updating variable bounds for variable index" + to_string(varIndex), e.getMessage());
    }
}

pair<double, double> MIPSolverCplex::getCurrentVariableBounds(int varIndex)
{
    pair<double, double> tmpBounds;

    try
    {
        tmpBounds.first = cplexVars[varIndex].getLB();
        tmpBounds.second = cplexVars[varIndex].getUB();
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError(
            "Error when obtaining variable bounds for variable index" + to_string(varIndex), e.getMessage());
    }
    return (tmpBounds);
}

bool MIPSolverCplex::supportsQuadraticObjective()
{
    return (true);
}
bool MIPSolverCplex::supportsQuadraticConstraints()
{
    return (true);
}

double MIPSolverCplex::getDualObjectiveValue()
{
    double objVal = NAN;

    bool isMIP = getDiscreteVariableStatus() && ProcessInfo::getInstance().originalProblem->isProblemDiscrete();

    try
    {
        if (isMIP)
        {
            objVal = cplexInstance.getBestObjValue();
        }
        else
        {
            objVal = cplexInstance.getObjValue();
        }
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when obtaining dual objective value", e.getMessage());
    }

    return (objVal);
}

std::pair<std::vector<double>, std::vector<double>> MIPSolverCplex::presolveAndGetNewBounds()
{
    auto numVar = originalProblem->getNumberOfVariables();

    IloNumArray redubs(cplexEnv, numVar);
    IloNumArray redlbs(cplexEnv, numVar);

    IloBoolArray redund(cplexEnv);

    try
    {
        bool isUpdated = false;

        cplexInstance.basicPresolve(cplexVars, redlbs, redubs, cplexConstrs, redund);

        std::vector<double> newLBs;
        std::vector<double> newUBs;

        newLBs.reserve(numVar);
        newUBs.reserve(numVar);

        for (int i = 0; i < numVar; i++)
        {
            newLBs.push_back(redlbs[i]);
            newUBs.push_back(redubs[i]);
        }

        if (Settings::getInstance().getBoolSetting("MIP.Presolve.RemoveRedundantConstraints", "Dual"))
        {
            int numconstr = 0;

            for (int j = 0; j < cplexConstrs.getSize(); j++)
            {
                if (redund[j] == true)
                {
                    cplexModel.remove(cplexConstrs[j]);
                    cplexConstrs[j].asConstraint().removeFromAll();

                    numconstr++;
                    isUpdated = true;
                }
            }

            if (isUpdated)
            {
                cplexInstance.extract(cplexModel);
                ProcessInfo::getInstance().outputInfo(
                    "     Removed " + to_string(numconstr) + " redundant constraints from MIP model.");
                ProcessInfo::getInstance().numConstraintsRemovedInPresolve = numconstr;
            }
        }

        redlbs.end();
        redubs.end();
        redund.end();

        return (std::make_pair(newLBs, newUBs));
    }
    catch (IloException &e)
    {
        redlbs.end();
        redubs.end();
        redund.end();

        ProcessInfo::getInstance().outputError("Error during presolve", e.getMessage());

        return (std::make_pair(originalProblem->getVariableLowerBounds(), originalProblem->getVariableLowerBounds()));
    }
}

void MIPSolverCplex::writePresolvedToFile(std::string filename)
{
    try
    {
        //Not implemented
    }
    catch (IloException &e)
    {
        ProcessInfo::getInstance().outputError("Error when saving presolved model to file", e.getMessage());
    }
}

void MIPSolverCplex::checkParameters()
{
}

void MIPSolverCplex::createHyperplane(Hyperplane hyperplane,
                                      std::function<IloConstraint(IloRange)> addConstraintFunction)
{
    auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

    auto optional = createHyperplaneTerms(hyperplane);

    if (!optional)
    {
        return;
    }

    auto tmpPair = optional.get();

    //auto tmpPair = createHyperplaneTerms(hyperplane);
    bool hyperplaneIsOk = true;

    for (auto E : tmpPair.first)
    {
        if (E.value != E.value) //Check for NaN
        {
            ProcessInfo::getInstance().outputWarning("     Warning: hyperplane not generated, NaN found in linear terms!");
            hyperplaneIsOk = false;
            break;
        }
    }

    if (hyperplaneIsOk)
    {
        GeneratedHyperplane genHyperplane;

        IloExpr expr(cplexEnv);

        for (int i = 0; i < tmpPair.first.size(); i++)
        {
            expr += tmpPair.first.at(i).value * cplexVars[tmpPair.first.at(i).idx];
        }

        IloRange tmpRange(cplexEnv, -IloInfinity, expr, -tmpPair.second);

        auto addedConstr = addConstraintFunction(tmpRange);

        int constrIndex = 0;

        genHyperplane.generatedConstraintIndex = constrIndex;
        genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
        genHyperplane.generatedPoint = hyperplane.generatedPoint;
        genHyperplane.source = hyperplane.source;
        genHyperplane.generatedIter = currIter->iterationNumber;
        genHyperplane.isLazy = false;
        genHyperplane.isRemoved = false;

        generatedHyperplanes.push_back(genHyperplane);

        currIter->numHyperplanesAdded++;
        currIter->totNumHyperplanes++;
        expr.end();
    }
}

void MIPSolverCplex::createIntegerCut(std::vector<int> binaryIndexes,
                                      std::function<IloConstraint(IloRange)> addConstraintFunction)
{
    IloExpr expr(cplexEnv);

    for (int i = 0; i < binaryIndexes.size(); i++)
    {
        expr += 1.0 * cplexVars[binaryIndexes.at(i)];
    }

    IloRange tmpRange(cplexEnv, -IloInfinity, expr, binaryIndexes.size() - 1.0);

    auto addedConstr = addConstraintFunction(tmpRange);
    ProcessInfo::getInstance().numIntegerCutsAdded++;

    expr.end();
}
