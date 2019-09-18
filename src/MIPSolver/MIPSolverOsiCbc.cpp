/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverOsiCbc.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../PrimalSolver.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"

#include "CoinBuild.hpp"
#include "CoinModel.hpp"
#include "CoinPragma.hpp"
#include "CbcModel.hpp"
#include "OsiClpSolverInterface.hpp"

namespace SHOT
{

MIPSolverOsiCbc::MIPSolverOsiCbc(EnvironmentPtr envPtr)
{
    env = envPtr;

    initializeProblem();
    checkParameters();
}

MIPSolverOsiCbc::~MIPSolverOsiCbc() = default;

bool MIPSolverOsiCbc::initializeProblem()
{
    discreteVariablesActivated = true;

    if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
    {
        this->cutOff = SHOT_DBL_MAX;
    }
    else
    {
        this->cutOff = SHOT_DBL_MIN;
    }

    osiInterface = std::make_unique<OsiClpSolverInterface>();
    coinModel = std::make_unique<CoinModel>();

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();
    return (true);
}

bool MIPSolverOsiCbc::addVariable(std::string name, E_VariableType type, double lowerBound, double upperBound)
{
    int index = numberOfVariables;

    if(lowerBound < -getUnboundedVariableBoundValue())
        lowerBound = -getUnboundedVariableBoundValue();

    if(upperBound > getUnboundedVariableBoundValue())
        upperBound = getUnboundedVariableBoundValue();

    try
    {
        coinModel->setColumnBounds(index, lowerBound, upperBound);
        coinModel->setColName(index, name.c_str());

        switch(type)
        {
        case E_VariableType::Real:
            break;

        case E_VariableType::Integer:
            isProblemDiscrete = true;
            coinModel->setInteger(index);
            break;

        case E_VariableType::Binary:
            isProblemDiscrete = true;
            coinModel->setInteger(index);
            break;

        case E_VariableType::Semicontinuous:
            isProblemDiscrete = true;
            coinModel->setInteger(index);
            break;

        default:
            break;
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("Cbc exception caught when adding variable to model: ", e.what());
        return (false);
    }

    variableTypes.push_back(type);
    variableNames.push_back(name);
    variableLowerBounds.push_back(lowerBound);
    variableUpperBounds.push_back(upperBound);
    numberOfVariables++;
    return (true);
}

bool MIPSolverOsiCbc::initializeObjective() { return (true); }

bool MIPSolverOsiCbc::addLinearTermToObjective(double coefficient, int variableIndex)
{
    try
    {
        coinModel->setColObjective(variableIndex, coefficient);
    }
    catch(std::exception& e)
    {
        env->output->outputError("Cbc exception caught when adding linear term to objective: ", e.what());
        return (false);
    }

    return (true);
}

bool MIPSolverOsiCbc::addQuadraticTermToObjective([[maybe_unused]] double coefficient,
    [[maybe_unused]] int firstVariableIndex, [[maybe_unused]] int secondVariableIndex)
{
    // Not implemented
    return (false);
}

bool MIPSolverOsiCbc::finalizeObjective(bool isMinimize, double constant)
{
    try
    {
        if(constant != 0.0)
            coinModel->setObjectiveOffset(constant);

        if(isMinimize)
        {
            coinModel->setOptimizationDirection(1.0);
            isMinimizationProblem = true;
        }
        else
        {
            coinModel->setOptimizationDirection(-1.0);
            isMinimizationProblem = false;
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("Cbc exception caught when adding objective function to model: ", e.what());
        return (false);
    }

    return (true);
}

bool MIPSolverOsiCbc::initializeConstraint() { return (true); }

bool MIPSolverOsiCbc::addLinearTermToConstraint(double coefficient, int variableIndex)
{
    try
    {
        coinModel->setElement(numberOfConstraints, variableIndex, coefficient);
    }
    catch(std::exception& e)
    {
        env->output->outputError("Cbc exception caught when adding linear term to constraint: ", e.what());
        return (false);
    }

    return (true);
}

bool MIPSolverOsiCbc::addQuadraticTermToConstraint([[maybe_unused]] double coefficient,
    [[maybe_unused]] int firstVariableIndex, [[maybe_unused]] int secondVariableIndex)
{
    // Not implemented
    return (false);
}

bool MIPSolverOsiCbc::finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant)
{
    int index = numberOfConstraints;
    try
    {
        if(valueLHS <= valueRHS)
        {
            coinModel->setRowBounds(index, valueLHS - constant, valueRHS - constant);
        }
        else
        {
            coinModel->setRowBounds(index, valueRHS - constant, valueLHS - constant);
        }

        coinModel->setRowName(index, name.c_str());
    }
    catch(std::exception& e)
    {
        env->output->outputError("Cbc exception caught when adding constraint to model: ", e.what());
        return (false);
    }

    numberOfConstraints++;
    return (true);
}

bool MIPSolverOsiCbc::finalizeProblem()
{
    try
    {
        osiInterface->loadFromCoinModel(*coinModel);
        cbcModel = std::make_unique<CbcModel>(*osiInterface);
        CbcMain0(*cbcModel);

        if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
        {
            cbcModel->setLogLevel(0);
            osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
        }

        setSolutionLimit(1);
    }
    catch(std::exception& e)
    {
        env->output->outputError("Cbc exception caught when finalizing model", e.what());
        return (false);
    }

    return (true);
}

void MIPSolverOsiCbc::initializeSolverSettings()
{
    if(cbcModel->haveMultiThreadSupport())
        cbcModel->setNumberThreads(env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual"));

    cbcModel->setAllowableGap(env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") / 1.0);
    cbcModel->setAllowableFractionGap(env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") / 1.0);
    cbcModel->setMaximumSolutions(solLimit);
    cbcModel->setMaximumSavedSolutions(env->settings->getSetting<int>("MIP.SolutionPool.Capacity", "Dual"));

    // Cbc has problems with too large cutoff values
    if(isMinimizationProblem && std::abs(this->cutOff) < 10e20)
    {
        cbcModel->setCutoff(this->cutOff);
        env->output->outputDebug("     Setting cutoff value to " + std::to_string(cutOff) + " for minimization.");
    }
    else if(!isMinimizationProblem && std::abs(this->cutOff) < 10e20)
    {
        cbcModel->setCutoff(this->cutOff);
        env->output->outputDebug("     Setting cutoff value to " + std::to_string(cutOff) + " for maximization.");
    }
}

int MIPSolverOsiCbc::addLinearConstraint(
    const std::vector<PairIndexValue>& elements, double constant, std::string name, bool isGreaterThan)
{
    CoinPackedVector cut;

    for(auto E : elements)
    {
        cut.insert(E.index, E.value);
    }

    // Adds the cutting plane
    if(isGreaterThan)
        osiInterface->addRow(cut, -constant, osiInterface->getInfinity(), name);
    else
        osiInterface->addRow(cut, -osiInterface->getInfinity(), -constant, name);

    return (osiInterface->getNumRows() - 1);
}

void MIPSolverOsiCbc::activateDiscreteVariables(bool activate)
{
    if(activate)
    {
        env->output->outputDebug("Activating MIP strategy");

        for(int i = 0; i < numberOfVariables; i++)
        {
            if(variableTypes.at(i) == E_VariableType::Integer || variableTypes.at(i) == E_VariableType::Binary)
            {
                osiInterface->setInteger(i);
            }
        }

        discreteVariablesActivated = true;
    }
    else
    {
        env->output->outputDebug("Activating LP strategy");
        for(int i = 0; i < numberOfVariables; i++)
        {
            if(variableTypes.at(i) == E_VariableType::Integer || variableTypes.at(i) == E_VariableType::Binary)
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

    if(cbcModel->isProvenOptimal())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
    }
    else if(cbcModel->isProvenInfeasible())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
    }
    else if(cbcModel->isSolutionLimitReached())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
    }
    else if(cbcModel->isSecondsLimitReached())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
    }
    else if(cbcModel->isNodeLimitReached())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::NodeLimit;
    }
    else if(cbcModel->isAbandoned())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }
    else if(cbcModel->isContinuousUnbounded())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
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
        cbcModel = std::make_unique<CbcModel>(*osiInterface);

        initializeSolverSettings();

        // Adding the MIP starts provided
        try
        {
            for(auto& P : MIPStarts)
            {
                cbcModel->setMIPStart(P);
            }

            MIPStarts.clear();
        }
        catch(std::exception& e)
        {
            env->output->outputError("Error when adding MIP start to Cbc", e.what());
        }

        CbcMain0(*cbcModel);

        if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
        {
            cbcModel->setLogLevel(0);
            osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
        }

        const char* argv[] = { "", "-solve", "-quit" };
        CbcMain1(3, argv, *cbcModel);

        MIPSolutionStatus = getSolutionStatus();
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when solving subproblem with Cbc", e.what());
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
                        V->index, -getUnboundedVariableBoundValue() / 10e30, getUnboundedVariableBoundValue() / 10e30);
                    variableBoundsUpdated = true;
                }
            }
        }

        if(variableBoundsUpdated)
        {
            cbcModel = std::make_unique<CbcModel>(*osiInterface);

            initializeSolverSettings();

            CbcMain0(*cbcModel);

            if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
            {
                cbcModel->setLogLevel(0);
                osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
            }

            cbcModel->branchAndBound();

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

bool MIPSolverOsiCbc::repairInfeasibility() { return false; }

int MIPSolverOsiCbc::increaseSolutionLimit(int increment)
{
    this->solLimit += increment;

    this->setSolutionLimit(this->solLimit);

    return (this->solLimit);
}

void MIPSolverOsiCbc::setSolutionLimit(long int limit) { this->solLimit = limit; }

int MIPSolverOsiCbc::getSolutionLimit() { return (this->solLimit); }

void MIPSolverOsiCbc::setTimeLimit(double seconds)
{
    try
    {
        cbcModel->setMaximumSeconds(seconds);
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when setting time limit in Cbc", e.what());
    }
}

void MIPSolverOsiCbc::setCutOff(double cutOff)
{
    double cutOffTol = env->settings->getSetting<double>("MIP.CutOffTolerance", "Dual");

    try
    {
        if(isMinimizationProblem)
        {
            this->cutOff = cutOff + cutOffTol;

            env->output->outputDebug(
                "     Setting cutoff value to " + std::to_string(this->cutOff) + " for minimization.");
        }
        else
        {
            this->cutOff = cutOff - cutOffTol;

            env->output->outputDebug(
                "     Setting cutoff value to " + std::to_string(this->cutOff) + " for maximization.");
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when setting cut off value", e.what());
    }
}

void MIPSolverOsiCbc::setCutOffAsConstraint([[maybe_unused]] double cutOff)
{
    // TODO
}

void MIPSolverOsiCbc::addMIPStart(VectorDouble point)
{
    std::vector<std::pair<std::string, double>> variableValues;

    for(int i = 0; i < env->problem->properties.numberOfVariables; i++)
    {
        std::pair<std::string, double> tmpPair;

        tmpPair.first = variableNames.at(i);
        tmpPair.second = point.at(i);

        variableValues.push_back(tmpPair);
    }

    for(auto& V : env->reformulatedProblem->auxiliaryVariables)
    {
        std::pair<std::string, double> tmpPair;

        tmpPair.first = V->name;
        tmpPair.second = V->calculateAuxiliaryValue(point);

        variableValues.push_back(tmpPair);
    }

    if(env->reformulatedProblem->auxiliaryObjectiveVariable)
    {
        std::pair<std::string, double> tmpPair;

        tmpPair.first = env->reformulatedProblem->auxiliaryObjectiveVariable->name;
        tmpPair.second = env->reformulatedProblem->auxiliaryObjectiveVariable->calculateAuxiliaryValue(point);

        variableValues.push_back(tmpPair);
    }

    MIPStarts.push_back(variableValues);
}

void MIPSolverOsiCbc::writeProblemToFile(std::string filename)
{
    try
    {
        osiInterface->writeLp(filename.c_str(), "");
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when saving model to file in Cbc", e.what());
    }
}

double MIPSolverOsiCbc::getObjectiveValue(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();

    double objVal = NAN;

    if(!isMIP && solIdx > 0) // LP problems only have one solution!
    {
        env->output->outputError(
            "Cannot obtain solution with index " + std::to_string(solIdx) + " in Cbc since the problem is LP/QP!");

        return (objVal);
    }

    try
    {
        // Fixes some strange behavior with the objective value when solving MIP vs LP problems
        if(isMIP && isMinimizationProblem)
        {
            objVal = 1.0;
        }
        else if(isMIP && !isMinimizationProblem)
        {
            objVal = -1.0;
        }
        else
        {
            objVal = 1.0;
        }

        if(isMIP)
        {
            objVal *= cbcModel->savedSolutionObjective(solIdx);
        }
        else
        {
            objVal *= cbcModel->getObjValue();
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "Error when obtaining objective value for solution index " + std::to_string(solIdx) + " in Cbc", e.what());
    }

    return (objVal);
}

void MIPSolverOsiCbc::deleteMIPStarts() { MIPStarts.clear(); }

VectorDouble MIPSolverOsiCbc::getVariableSolution(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();
    int numVar = cbcModel->getNumCols();
    VectorDouble solution(numVar);

    try
    {
        if(isMIP)
        {
            auto tmpSol = cbcModel->savedSolution(solIdx);
            for(int i = 0; i < numVar; i++)
            {
                solution.at(i) = tmpSol[i];
            }
        }
        else
        {
            auto tmpSol = cbcModel->bestSolution();

            for(int i = 0; i < numVar; i++)
            {
                solution.at(i) = tmpSol[i];
            }
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "Error when reading solution with index " + std::to_string(solIdx) + " in Cbc", e.what());
    }
    return (solution);
}

int MIPSolverOsiCbc::getNumberOfSolutions()
{
    int numSols = 0;
    bool isMIP = getDiscreteVariableStatus();

    try
    {
        if(isMIP)
            numSols = cbcModel->getSolutionCount();
        else
        {
            numSols = cbcModel->numberSavedSolutions();
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when obtaining number of solutions in Cbc", e.what());
    }

    return (numSols);
}

void MIPSolverOsiCbc::fixVariable(int varIndex, double value) { updateVariableBound(varIndex, value, value); }

void MIPSolverOsiCbc::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.first == lowerBound && currentVariableBounds.second == upperBound)
        return;

    try
    {
        osiInterface->setColBounds(varIndex, lowerBound, upperBound);
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "Error when updating variable bounds for variable index" + std::to_string(varIndex) + " in Cbc", e.what());
    }
}

void MIPSolverOsiCbc::updateVariableLowerBound(int varIndex, double lowerBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.first == lowerBound)
        return;

    try
    {
        osiInterface->setColLower(varIndex, lowerBound);
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "Error when updating variable bounds for variable index" + std::to_string(varIndex) + " in Cbc", e.what());
    }
}

void MIPSolverOsiCbc::updateVariableUpperBound(int varIndex, double upperBound)
{
    auto currentVariableBounds = getCurrentVariableBounds(varIndex);

    if(currentVariableBounds.second == upperBound)
        return;

    try
    {
        osiInterface->setColUpper(varIndex, upperBound);
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "Error when updating variable bounds for variable index" + std::to_string(varIndex) + " in Cbc", e.what());
    }
}

PairDouble MIPSolverOsiCbc::getCurrentVariableBounds(int varIndex)
{
    PairDouble tmpBounds;

    try
    {
        tmpBounds.first = osiInterface->getColLower()[varIndex];
        tmpBounds.second = osiInterface->getColUpper()[varIndex];
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "Error when obtaining variable bounds for variable index" + std::to_string(varIndex) + " in Cbc", e.what());
    }

    return (tmpBounds);
}

bool MIPSolverOsiCbc::supportsQuadraticObjective() { return (false); }

bool MIPSolverOsiCbc::supportsQuadraticConstraints() { return (false); }

double MIPSolverOsiCbc::getUnboundedVariableBoundValue() { return (1e+50); }

double MIPSolverOsiCbc::getDualObjectiveValue()
{
    double objVal = NAN;

    try
    {
        objVal = cbcModel->getBestPossibleObjValue();
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when obtaining dual objective value in Cbc", e.what());
    }

    return (objVal);
}

std::pair<VectorDouble, VectorDouble> MIPSolverOsiCbc::presolveAndGetNewBounds()
{
    return (std::make_pair(variableLowerBounds, variableUpperBounds));
}

void MIPSolverOsiCbc::writePresolvedToFile([[maybe_unused]] std::string filename)
{
    // Not implemented
}

void MIPSolverOsiCbc::checkParameters()
{
    // Check if Cbc has been compiled with support for multiple threads
    if(!cbcModel->haveMultiThreadSupport())
        env->settings->updateSetting("MIP.NumberOfThreads", "Dual", 1);

    // Some features are not available in Cbc
    env->settings->updateSetting("TreeStrategy", "Dual", static_cast<int>(ES_TreeStrategy::MultiTree));
    env->settings->updateSetting(
        "Reformulation.Quadratics.Strategy", "Model", static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear));

    // For stability
    env->settings->updateSetting("Tolerance.TrustLinearConstraintValues", "Primal", false);
}

int MIPSolverOsiCbc::getNumberOfExploredNodes()
{
    try
    {
        return (cbcModel->getNodeCount());
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when getting number of explored nodes", e.what());
        return 0;
    }
}
} // namespace SHOT