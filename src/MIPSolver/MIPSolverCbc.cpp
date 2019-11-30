/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverCbc.h"

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

MIPSolverCbc::MIPSolverCbc(EnvironmentPtr envPtr)
{
    env = envPtr;

    initializeProblem();
    checkParameters();
}

MIPSolverCbc::~MIPSolverCbc() = default;

bool MIPSolverCbc::initializeProblem()
{
    discreteVariablesActivated = true;

    this->cutOff = SHOT_DBL_MAX;

    osiInterface = std::make_unique<OsiClpSolverInterface>();
    coinModel = std::make_unique<CoinModel>();

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;
    return (true);
}

bool MIPSolverCbc::addVariable(std::string name, E_VariableType type, double lowerBound, double upperBound)
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

bool MIPSolverCbc::initializeObjective() { return (true); }

bool MIPSolverCbc::addLinearTermToObjective(double coefficient, int variableIndex)
{
    try
    {
        coinModel->setColObjective(variableIndex, coefficient);
        objectiveLinearExpression.insert(variableIndex, coefficient);
    }
    catch(std::exception& e)
    {
        env->output->outputError("Cbc exception caught when adding linear term to objective: ", e.what());
        return (false);
    }

    return (true);
}

bool MIPSolverCbc::addQuadraticTermToObjective([[maybe_unused]] double coefficient,
    [[maybe_unused]] int firstVariableIndex, [[maybe_unused]] int secondVariableIndex)
{
    // Not implemented
    return (false);
}

bool MIPSolverCbc::finalizeObjective(bool isMinimize, double constant)
{
    try
    {
        if(!isMinimize)
        {
            isMinimizationProblem = false;

            for(int i = 0; i < objectiveLinearExpression.getNumElements(); i++)
            {
                objectiveLinearExpression.getElements()[i] *= -1;
                coinModel->setColObjective(
                    objectiveLinearExpression.getIndices()[i], objectiveLinearExpression.getElements()[i]);
            }

            coinModel->setObjectiveOffset(-constant);
        }
        else
        {
            isMinimizationProblem = true;
            coinModel->setObjectiveOffset(constant);
        }

        coinModel->setOptimizationDirection(1.0);
    }
    catch(std::exception& e)
    {
        env->output->outputError("Cbc exception caught when adding objective function to model: ", e.what());
        return (false);
    }

    return (true);
}

bool MIPSolverCbc::initializeConstraint() { return (true); }

bool MIPSolverCbc::addLinearTermToConstraint(double coefficient, int variableIndex)
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

bool MIPSolverCbc::addQuadraticTermToConstraint([[maybe_unused]] double coefficient,
    [[maybe_unused]] int firstVariableIndex, [[maybe_unused]] int secondVariableIndex)
{
    // Not implemented
    return (false);
}

bool MIPSolverCbc::finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant)
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

bool MIPSolverCbc::finalizeProblem()
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

void MIPSolverCbc::initializeSolverSettings()
{
    if(cbcModel->haveMultiThreadSupport())
        cbcModel->setNumberThreads(env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual"));

    cbcModel->setAllowableGap(env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") / 1.0);
    cbcModel->setAllowableFractionGap(env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination") / 1.0);
    cbcModel->setMaximumSolutions(solLimit);
    cbcModel->setMaximumSavedSolutions(env->settings->getSetting<int>("MIP.SolutionPool.Capacity", "Dual"));

    // Adds a user-provided node limit
    if(env->settings->getSetting<double>("MIP.NodeLimit", "Dual") > 0)
    {
        auto nodeLimit = env->settings->getSetting<double>("MIP.NodeLimit", "Dual");

        if(nodeLimit > SHOT_INT_MAX)
            nodeLimit = SHOT_INT_MAX;

        cbcModel->setMaximumNodes(nodeLimit);
    }
}

int MIPSolverCbc::addLinearConstraint(
    const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan)
{
    try
    {
        CoinPackedVector cut;

        for(auto E : elements)
        {
            cut.insert(E.first, E.second);
        }

        // Adds the cutting plane
        if(isGreaterThan)
            osiInterface->addRow(cut, -constant, osiInterface->getInfinity(), name);
        else
            osiInterface->addRow(cut, -osiInterface->getInfinity(), -constant, name);
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when adding term to linear constraint in Cbc: ", e.what());
    }
    catch(CoinError& e)
    {
        env->output->outputError("Error when adding term to linear constraint in Cbc: ", e.message());
    }

    return (osiInterface->getNumRows() - 1);
}

void MIPSolverCbc::activateDiscreteVariables(bool activate)
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

E_ProblemSolutionStatus MIPSolverCbc::getSolutionStatus()
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
    else if(cbcModel->isProvenDualInfeasible())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
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
        MIPSolutionStatus = E_ProblemSolutionStatus::Abort;
    }
    else if(cbcModel->isContinuousUnbounded())
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if(cbcModel->status() == 5)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Abort;
    }
    else
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
        env->output->outputError("MIP solver return status unknown.");
    }

    return (MIPSolutionStatus);
}

E_ProblemSolutionStatus MIPSolverCbc::solveProblem()
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

        // Cbc has problems with too large cutoff values
        if(std::abs(this->cutOff) < 10e20)
            cbcModel->setCutoff(this->cutOff);

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

    if(MIPSolutionStatus == E_ProblemSolutionStatus::Infeasible)
    {
        if((env->reformulatedProblem->objectiveFunction->properties.classification
               == E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear))
        {
            osiInterface->setColBounds(getDualAuxiliaryObjectiveVariableIndex(), -1000000000.0, 1000000000.0);

            cbcModel = std::make_unique<CbcModel>(*osiInterface);

            initializeSolverSettings();

            CbcMain0(*cbcModel);

            if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
            {
                cbcModel->setLogLevel(0);
                osiInterface->setHintParam(OsiDoReducePrint, false, OsiHintTry);
            }

            const char* argv[] = { "", "-solve", "-quit" };
            CbcMain1(3, argv, *cbcModel);

            MIPSolutionStatus = getSolutionStatus();

            osiInterface->setColBounds(getDualAuxiliaryObjectiveVariableIndex(), -getUnboundedVariableBoundValue(),
                getUnboundedVariableBoundValue());
        }
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
        else if((env->reformulatedProblem->objectiveFunction->properties.classification
                    >= E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear))
        {
            // The auxiliary variable in the dual problem is unbounded
            updateVariableBound(getDualAuxiliaryObjectiveVariableIndex(), -getUnboundedVariableBoundValue() / 10e30,
                getUnboundedVariableBoundValue() / 10e30);
            variableBoundsUpdated = true;
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

            const char* argv[] = { "", "-solve", "-quit" };
            CbcMain1(3, argv, *cbcModel);

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

bool MIPSolverCbc::repairInfeasibility() { return false; }

int MIPSolverCbc::increaseSolutionLimit(int increment)
{
    this->solLimit += increment;

    this->setSolutionLimit(this->solLimit);

    return (this->solLimit);
}

void MIPSolverCbc::setSolutionLimit(long int limit) { this->solLimit = limit; }

int MIPSolverCbc::getSolutionLimit() { return (this->solLimit); }

void MIPSolverCbc::setTimeLimit(double seconds)
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

void MIPSolverCbc::setCutOff(double cutOff)
{
    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
        return;

    double cutOffTol = env->settings->getSetting<double>("MIP.CutOff.Tolerance", "Dual");

    if(isMinimizationProblem)
    {
        this->cutOff = cutOff + cutOffTol;

        env->output->outputDebug("     Setting cutoff value to " + std::to_string(this->cutOff) + " for minimization.");
    }
    else
    {
        this->cutOff = -1 * (cutOff + cutOffTol);

        env->output->outputDebug(
            "     Setting cutoff value to " + std::to_string(cutOff + cutOffTol) + " for maximization.");
    }
}

void MIPSolverCbc::setCutOffAsConstraint([[maybe_unused]] double cutOff)
{
    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
        return;

    try
    {
        if(!cutOffConstraintDefined)
        {
            if(isMinimizationProblem)
                osiInterface->addRow(objectiveLinearExpression, -osiInterface->getInfinity(), cutOff, "CUTOFF_C");
            else
                osiInterface->addRow(
                    objectiveLinearExpression, -osiInterface->getInfinity(), -1.0 * cutOff, "CUTOFF_C");

            cutOffConstraintDefined = true;
            cutOffConstraintIndex = osiInterface->getNumRows() - 1;

            modelUpdated = true;
        }
        else
        {
            if(isMinimizationProblem)
            {
                osiInterface->setRowUpper(cutOffConstraintIndex, cutOff);

                env->output->outputDebug(
                    "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for minimization.");
            }
            else
            {
                osiInterface->setRowUpper(cutOffConstraintIndex, -cutOff);

                env->output->outputDebug(
                    "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for maximization.");
            }

            modelUpdated = true;
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when setting cut off constraint value", e.what());
    }
}

void MIPSolverCbc::addMIPStart(VectorDouble point)
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
        tmpPair.second = V->calculate(point);

        variableValues.push_back(tmpPair);
    }

    if(env->reformulatedProblem->auxiliaryObjectiveVariable)
    {
        std::pair<std::string, double> tmpPair;

        tmpPair.first = env->reformulatedProblem->auxiliaryObjectiveVariable->name;

        if(isMinimizationProblem)
            tmpPair.second = env->reformulatedProblem->auxiliaryObjectiveVariable->calculate(point);
        else
            tmpPair.second = -1.0 * env->reformulatedProblem->auxiliaryObjectiveVariable->calculate(point);

        variableValues.push_back(tmpPair);
    }

    MIPStarts.push_back(variableValues);
}

void MIPSolverCbc::writeProblemToFile(std::string filename)
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

double MIPSolverCbc::getObjectiveValue(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();

    if(!isMIP && solIdx > 0) // LP problems only have one solution!
    {
        env->output->outputError(
            "Cannot obtain solution with index " + std::to_string(solIdx) + " in Cbc since the problem is LP/QP!");

        return (NAN);
    }

    double objectiveValue = NAN;

    // Cannot trust Cbc to give the correct sign of the objective back se we recalculate it
    try
    {
        auto variableSolution = getVariableSolution(solIdx);
        objectiveValue = (isMinimizationProblem) ? coinModel->objectiveOffset() : -coinModel->objectiveOffset();

        for(int i = 0; i < objectiveLinearExpression.getNumElements(); i++)
        {
            objectiveValue += objectiveLinearExpression.getElements()[i]
                * variableSolution[objectiveLinearExpression.getIndices()[i]];
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "Error when obtaining objective value for solution index " + std::to_string(solIdx) + " in Cbc", e.what());
    }

    return (objectiveValue);
}

void MIPSolverCbc::deleteMIPStarts() { MIPStarts.clear(); }

bool MIPSolverCbc::createIntegerCut(VectorInteger& binaryIndexesOnes, VectorInteger& binaryIndexesZeroes)
{
    try
    {
        CoinPackedVector cut;

        for(int I : binaryIndexesOnes)
        {
            cut.insert(I, 1.0);
        }

        for(int I : binaryIndexesZeroes)
        {
            cut.insert(I, -1.0);
        }

        osiInterface->addRow(cut, -osiInterface->getInfinity(), binaryIndexesOnes.size() - 1.0,
            "IC_" + std::to_string(env->solutionStatistics.numberOfIntegerCuts));

        modelUpdated = true;

        env->solutionStatistics.numberOfIntegerCuts++;
    }
    catch(CoinError& e)
    {
        env->output->outputError("Error when adding term to integer cut in Cbc: ", e.message());
        return (false);
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when adding term to integer cut in Cbc: ", e.what());
        return (false);
    }

    return (true);
}

VectorDouble MIPSolverCbc::getVariableSolution(int solIdx)
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

int MIPSolverCbc::getNumberOfSolutions()
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

void MIPSolverCbc::fixVariable(int varIndex, double value) { updateVariableBound(varIndex, value, value); }

void MIPSolverCbc::updateVariableBound(int varIndex, double lowerBound, double upperBound)
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

void MIPSolverCbc::updateVariableLowerBound(int varIndex, double lowerBound)
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

void MIPSolverCbc::updateVariableUpperBound(int varIndex, double upperBound)
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

PairDouble MIPSolverCbc::getCurrentVariableBounds(int varIndex)
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

bool MIPSolverCbc::supportsQuadraticObjective() { return (false); }

bool MIPSolverCbc::supportsQuadraticConstraints() { return (false); }

double MIPSolverCbc::getUnboundedVariableBoundValue() { return (1e+50); }

double MIPSolverCbc::getDualObjectiveValue()
{
    double objVal = NAN;

    try
    {
        objVal = cbcModel->getBestPossibleObjValue();

        if(!isMinimizationProblem)
            objVal *= -1.0;
    }
    catch(std::exception& e)
    {
        env->output->outputError("Error when obtaining dual objective value in Cbc", e.what());
    }

    return (objVal);
}

std::pair<VectorDouble, VectorDouble> MIPSolverCbc::presolveAndGetNewBounds()
{
    return (std::make_pair(variableLowerBounds, variableUpperBounds));
}

void MIPSolverCbc::writePresolvedToFile([[maybe_unused]] std::string filename)
{
    // Not implemented
}

void MIPSolverCbc::checkParameters()
{
    // Check if Cbc has been compiled with support for multiple threads
    if(!cbcModel->haveMultiThreadSupport())
    {
        env->settings->updateSetting("MIP.NumberOfThreads", "Dual", 1);
    }

    // For stability
    env->settings->updateSetting("Tolerance.TrustLinearConstraintValues", "Primal", false);
}

int MIPSolverCbc::getNumberOfExploredNodes()
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