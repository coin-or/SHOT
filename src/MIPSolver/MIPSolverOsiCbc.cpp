/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverOsiCbc.h"

MIPSolverOsiCbc::MIPSolverOsiCbc(EnvironmentPtr envPtr)
{
    env = envPtr;

    discreteVariablesActivated = true;

    osiInterface = new OsiClpSolverInterface();

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();
}

MIPSolverOsiCbc::~MIPSolverOsiCbc()
{
}

bool MIPSolverOsiCbc::createLinearProblem(OptProblem *origProblem)
{
    originalProblem = origProblem;

    if (originalProblem->isTypeOfObjectiveMinimize())
    {
        this->cutOff = DBL_MAX;
    }
    else
    {
        this->cutOff = -DBL_MAX;
    }

    CoinModel *coinModel;
    coinModel = new CoinModel();

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
        coinModel->setColumnBounds(i, tmpLBs.at(i), tmpUBs.at(i));
        coinModel->setColName(i, tmpNames.at(i).c_str());

        if (tmpTypes.at(i) == 'C')
        {
        }
        else if (tmpTypes.at(i) == 'I' || tmpTypes.at(i) == 'B')
        {
            coinModel->setInteger(i);
        }
        else if (tmpTypes.at(i) == 'D')
        {
            coinModel->setInteger(i);
        }
        else
        {
            env->output->outputWarning(
                "Error variable type " + std::to_string(tmpTypes.at(i)) + " for " + tmpNames.at(i));
        }
    }

    // Now creating the objective function

    auto tmpObjPairs = origProblem->getObjectiveFunctionVarCoeffPairs();

    for (int i = 0; i < tmpObjPairs.size(); i++)
    {
        coinModel->setColObjective(tmpObjPairs.at(i).first, tmpObjPairs.at(i).second);
    }

    // Add quadratic terms in the objective if they exist (and the strategy is to solve QPs)
    // Since this is not used for the Cbc solver here, it should never happen and quadratic objective functions
    // are regarded as general nonlinear
    if (origProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
    {
        auto quadTerms = origProblem->getQuadraticTermsInConstraint(-1);

        for (auto T : quadTerms)
        {
            coinModel->setQuadraticElement(T->idxOne, T->idxTwo, T->coef);
        }
    }

    double objConstant = origProblem->getObjectiveConstant();
    coinModel->setObjectiveOffset(objConstant);

    if (origProblem->isTypeOfObjectiveMinimize())
    {
        coinModel->setOptimizationDirection(1.0);
    }
    else
    {
        coinModel->setOptimizationDirection(-1.0);
    }

    // Now creating the constraints

    int row_nonz = 0;
    int obj_nonz = 0;
    int varIdx = 0;

    SparseMatrix *m_linearConstraintCoefficientsInRowMajor = origProblem->getProblemInstance()->getLinearConstraintCoefficientsInRowMajor();

    auto constrTypes = origProblem->getProblemInstance()->getConstraintTypes();
    auto constrNames = origProblem->getProblemInstance()->getConstraintNames();
    auto constrLBs = origProblem->getProblemInstance()->getConstraintLowerBounds();
    auto constrUBs = origProblem->getProblemInstance()->getConstraintUpperBounds();

    int corr = 0;

    for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
    {
        // Only use constraints that don't contain a nonlinear part (may include a quadratic part)
        if (!origProblem->isConstraintNonlinear(rowIdx))
        {
            if (origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients != NULL && origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients->numberOfValues > 0)
            {
                row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1] - m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];

                for (int j = 0; j < row_nonz; j++)
                {
                    double val =
                        m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
                    varIdx =
                        m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];

                    coinModel->setElement(rowIdx - corr, varIdx, val);
                }
            }

            double rowConstant = origProblem->getProblemInstance()->instanceData->constraints->con[rowIdx]->constant;

            coinModel->setRowName(rowIdx - corr, constrNames[rowIdx].c_str());

            // Add the constraint
            if (constrTypes[rowIdx] == 'L')
            {
                coinModel->setRowUpper(rowIdx - corr, constrUBs[rowIdx] - rowConstant);
            }
            else if (constrTypes[rowIdx] == 'G')
            {
                coinModel->setRowLower(rowIdx - corr, constrLBs[rowIdx] - rowConstant);
            }
            else if (constrTypes[rowIdx] == 'E')
            {
                coinModel->setRowBounds(rowIdx - corr, constrLBs[rowIdx] - rowConstant, constrUBs[rowIdx] - rowConstant);
            }
            else
            {
            }
        }
        else
        {
            corr++;
        }
    }

    osiInterface->loadFromCoinModel(*coinModel);
    cbcModel = new CbcModel(*osiInterface);
    CbcMain0(*cbcModel);

    if (!env->settings->getBoolSetting("Console.DualSolver.Show", "Output"))
    {
        cbcModel->setLogLevel(0);
        osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
    }

    setSolutionLimit(1);

    return (true);
}

void MIPSolverOsiCbc::initializeSolverSettings()
{
    if (cbcModel->haveMultiThreadSupport())
    {
        cbcModel->setNumberThreads(env->settings->getIntSetting("MIP.NumberOfThreads", "Dual"));
    }

    cbcModel->setAllowableGap(env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination") / 2.0);
    cbcModel->setAllowableFractionGap(env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination") / 2.0);
    cbcModel->setMaximumSolutions(solLimit);
    cbcModel->setMaximumSavedSolutions(env->settings->getIntSetting("MIP.SolutionPool.Capacity", "Dual"));

    // Cbc has problems with too large cutoff values
    if (originalProblem->isTypeOfObjectiveMinimize() && abs(this->cutOff) < 10e20)
    {
        cbcModel->setCutoff(this->cutOff);

        env->output->outputInfo(
            "     Setting cutoff value to " + std::to_string(cutOff) + " for minimization.");
    }
    else if (!originalProblem->isTypeOfObjectiveMinimize() && abs(this->cutOff) < 10e20)
    {
        cbcModel->setCutoff(this->cutOff);
        env->output->outputInfo(
            "     Setting cutoff value to " + std::to_string(cutOff) + " for maximization.");
    }
}

int MIPSolverOsiCbc::addLinearConstraint(std::vector<IndexValuePair> elements, double constant, bool isGreaterThan)
{
    CoinPackedVector cut;

    for (int i = 0; i < elements.size(); i++)
    {
        cut.insert(elements.at(i).idx, elements.at(i).value);
    }

    // Adds the cutting plane
    if (isGreaterThan)
        osiInterface->addRow(cut, -constant, osiInterface->getInfinity());
    else
        osiInterface->addRow(cut, -osiInterface->getInfinity(), -constant);

    return (osiInterface->getNumRows() - 1);
}

void MIPSolverOsiCbc::activateDiscreteVariables(bool activate)
{
    auto variableTypes = env->model->originalProblem->getVariableTypes();
    int numVar = env->model->originalProblem->getNumberOfVariables();

    if (activate)
    {
        env->output->outputInfo("Activating MIP strategy");

        for (int i = 0; i < numVar; i++)
        {
            if (variableTypes.at(i) == 'I' || variableTypes.at(i) == 'B')
            {
                osiInterface->setInteger(i);
            }
        }

        discreteVariablesActivated = true;
    }
    else
    {
        env->output->outputInfo("Activating LP strategy");
        for (int i = 0; i < numVar; i++)
        {
            if (variableTypes.at(i) == 'I' || variableTypes.at(i) == 'B')
            {
                osiInterface->setContinuous(i);
            }
        }

        discreteVariablesActivated = false;
    }
}

E_ProblemSolutionStatus MIPSolverOsiCbc::getSolutionStatus()
{
    E_ProblemSolutionStatus MIPSolutionStatus;

    if (cbcModel->isProvenOptimal())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
    }
    else if (cbcModel->isProvenInfeasible())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
    }
    else if (cbcModel->isSolutionLimitReached())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
    }
    else if (cbcModel->isSecondsLimitReached())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
    }
    else if (cbcModel->isNodeLimitReached())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::NodeLimit;
    }
    else
    {
        env->output->outputError("MIP solver return status unknown.");
    }

    return (MIPSolutionStatus);
}

E_ProblemSolutionStatus MIPSolverOsiCbc::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    cachedSolutionHasChanged = true;

    try
    {
        cbcModel = new CbcModel(*osiInterface);

        initializeSolverSettings();

        // Adding the MIP starts provided
        for (auto it = MIPStarts.begin(); it != MIPStarts.end(); ++it)
        {
            cbcModel->setMIPStart(*it);
        }

        MIPStarts.clear();

        CbcMain0(*cbcModel);

        if (!env->settings->getBoolSetting("Console.DualSolver.Show", "Output"))
        {
            cbcModel->setLogLevel(0);
            osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
        }

        cbcModel->branchAndBound();

        MIPSolutionStatus = getSolutionStatus();
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when solving subproblem with Cbc", e.what());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

int MIPSolverOsiCbc::increaseSolutionLimit(int increment)
{
    this->solLimit += increment;

    this->setSolutionLimit(this->solLimit);

    return (this->solLimit);
}

void MIPSolverOsiCbc::setSolutionLimit(long int limit)
{
    this->solLimit = limit;
}

int MIPSolverOsiCbc::getSolutionLimit()
{
    return (this->solLimit);
}

void MIPSolverOsiCbc::setTimeLimit(double seconds)
{
    try
    {
        cbcModel->setMaximumSeconds(seconds);
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when setting time limit in Cbc", e.what());
    }
}

void MIPSolverOsiCbc::setCutOff(double cutOff)
{
    double cutOffTol = env->settings->getDoubleSetting("MIP.CutOffTolerance", "Dual");

    try
    {

        if (originalProblem->isTypeOfObjectiveMinimize())
        {
            this->cutOff = cutOff + cutOffTol;

            env->output->outputInfo(
                "     Setting cutoff value to " + std::to_string(this->cutOff) + " for minimization.");
        }
        else
        {
            this->cutOff = cutOff - cutOffTol;

            env->output->outputInfo(
                "     Setting cutoff value to " + std::to_string(this->cutOff) + " for maximization.");
        }
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when setting cut off value", e.what());
    }
}

void MIPSolverOsiCbc::addMIPStart(std::vector<double> point)
{
    auto numVar = env->model->originalProblem->getNumberOfVariables();
    auto varNames = env->model->originalProblem->getVariableNames();

    std::vector<std::pair<std::string, double>> variableValues;

    for (int i = 0; i < numVar; i++)
    {
        std::pair<std::string, double> tmpPair;

        tmpPair.first = varNames.at(i);
        tmpPair.second = point.at(i);

        variableValues.push_back(tmpPair);
    }
    try
    {
        MIPStarts.push_back(variableValues);
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when adding MIP start to Cbc", e.what());
    }
}

void MIPSolverOsiCbc::writeProblemToFile(std::string filename)
{
    try
    {
        osiInterface->writeLp(filename.c_str(), "");
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when saving model to file in Cbc", e.what());
    }
}

double MIPSolverOsiCbc::getObjectiveValue(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();

    double objVal = NAN;

    if (!isMIP && solIdx > 0) // LP problems only have one solution!
    {
        env->output->outputError(
            "Cannot obtain solution with index " + std::to_string(solIdx) + " in Cbc since the problem is LP/QP!");

        return (objVal);
    }

    try
    {
        // Fixes some strange behavior with the objective value when solving MIP vs LP problems
        if (isMIP && env->model->originalProblem->isTypeOfObjectiveMinimize())
        {
            objVal = 1.0;
        }
        else if (isMIP && !originalProblem->isTypeOfObjectiveMinimize())
        {
            objVal = -1.0;
        }
        else
        {
            objVal = 1.0;
        }

        if (isMIP)
        {
            objVal *= cbcModel->savedSolutionObjective(solIdx);
        }
        else
        {
            objVal *= cbcModel->getObjValue();
        }
    }
    catch (std::exception &e)
    {
        env->output->outputError(
            "Error when obtaining objective value for solution index " + std::to_string(solIdx) + " in Cbc", e.what());
    }

    return (objVal);
}

void MIPSolverOsiCbc::deleteMIPStarts()
{
    // Not implemented
}

std::vector<double> MIPSolverOsiCbc::getVariableSolution(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();
    int numVar = cbcModel->getNumCols();
    std::vector<double> solution(numVar);

    try
    {
        if (isMIP)
        {
            auto tmpSol = cbcModel->savedSolution(solIdx);
            for (int i = 0; i < numVar; i++)
            {
                solution.at(i) = tmpSol[i];
            }
        }
        else
        {
            auto tmpSol = cbcModel->bestSolution();

            for (int i = 0; i < numVar; i++)
            {
                solution.at(i) = tmpSol[i];
            }
        }
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when reading solution with index " + std::to_string(solIdx) + " in Cbc", e.what());
    }
    return (solution);
}

int MIPSolverOsiCbc::getNumberOfSolutions()
{
    int numSols = 0;
    bool isMIP = getDiscreteVariableStatus();

    try
    {
        if (isMIP)
            numSols = cbcModel->getSolutionCount();
        else
            numSols = 1;
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when obtaining number of solutions in Cbc", e.what());
    }

    return (numSols);
}

void MIPSolverOsiCbc::fixVariable(int varIndex, double value)
{
    updateVariableBound(varIndex, value, value);
}

void MIPSolverOsiCbc::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
    try
    {
        osiInterface->setColBounds(varIndex, lowerBound, upperBound);
    }
    catch (std::exception &e)
    {
        env->output->outputError(
            "Error when updating variable bounds for variable index" + std::to_string(varIndex) + " in Cbc", e.what());
    }
}

DoublePair MIPSolverOsiCbc::getCurrentVariableBounds(int varIndex)
{
    DoublePair tmpBounds;

    try
    {
        tmpBounds.first = osiInterface->getColLower()[varIndex];
        tmpBounds.second = osiInterface->getColUpper()[varIndex];
    }
    catch (std::exception &e)
    {
        env->output->outputError(
            "Error when obtaining variable bounds for variable index" + std::to_string(varIndex) + " in Cbc", e.what());
    }

    return (tmpBounds);
}

bool MIPSolverOsiCbc::supportsQuadraticObjective()
{
    return (false);
}

bool MIPSolverOsiCbc::supportsQuadraticConstraints()
{
    return (false);
}

double MIPSolverOsiCbc::getDualObjectiveValue()
{
    double objVal = NAN;

    try
    {
        objVal = cbcModel->getBestPossibleObjValue();
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when obtaining dual objective value in Cbc", e.what());
    }

    return (objVal);
}

std::pair<std::vector<double>, std::vector<double>> MIPSolverOsiCbc::presolveAndGetNewBounds()
{
    return (std::make_pair(originalProblem->getVariableLowerBounds(), env->model->originalProblem->getVariableUpperBounds()));
}

void MIPSolverOsiCbc::writePresolvedToFile(std::string filename)
{
    // Not implemented
}

void MIPSolverOsiCbc::checkParameters()
{
    env->settings->updateSetting("MIP.NumberOfThreads", "Dual", 0);

    // Some features are not available in Cbc
    env->settings->updateSetting("TreeStrategy", "Dual", static_cast<int>(ES_TreeStrategy::MultiTree));
    env->settings->updateSetting("QuadraticStrategy", "Dual", static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear));

    // For stability
    env->settings->updateSetting("Tolerance.TrustLinearConstraintValues", "Primal", false);
}

int MIPSolverOsiCbc::getNumberOfExploredNodes()
{
    try
    {
        return (cbcModel->getNodeCount());
    }
    catch (std::exception &e)
    {
        env->output->outputError("Error when getting number of explored nodes", e.what());
        return 0;
    }
}