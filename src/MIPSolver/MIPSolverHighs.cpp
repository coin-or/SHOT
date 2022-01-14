/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverHighs.h"
#include "MIPSolverCallbackBase.h"

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
MIPSolverHighs::MIPSolverHighs(EnvironmentPtr envPtr) { env = envPtr; }

MIPSolverHighs::~MIPSolverHighs() = default;

bool MIPSolverHighs::initializeProblem()
{
    discreteVariablesActivated = true;

    this->cutOff = 1e100;

    aMatrixStart.push_back(0);

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();

    return (true);
}

bool MIPSolverHighs::addVariable(
    std::string name, E_VariableType type, double lowerBound, double upperBound, double semiBound)
{
    int index = numberOfVariables;

    if(lowerBound < -getUnboundedVariableBoundValue())
        lowerBound = -getUnboundedVariableBoundValue();

    if(upperBound > getUnboundedVariableBoundValue())
        upperBound = getUnboundedVariableBoundValue();

    variableNames.push_back(name);
    variableCosts.push_back(0.0);

    variableTypes.push_back(type);
    variableLowerBounds.push_back(lowerBound);
    variableUpperBounds.push_back(upperBound);

    switch(type)
    {
    case E_VariableType::Real:
        variableTypesHighs.push_back(HighsVarType::kContinuous);
        break;

    case E_VariableType::Integer:
        isProblemDiscrete = true;
        variableTypesHighs.push_back(HighsVarType::kInteger);
        break;

    case E_VariableType::Binary:
        isProblemDiscrete = true;
        variableTypesHighs.push_back(HighsVarType::kInteger);
        break;

    case E_VariableType::Semiinteger:
        variableTypesHighs.push_back(HighsVarType::kSemiInteger);
    case E_VariableType::Semicontinuous:
    {
        // TODO
        isProblemDiscrete = true;
        break;
    }

    default:
        break;
    }

    numberOfVariables++;
    return (true);
}

bool MIPSolverHighs::initializeObjective() { return (true); }

bool MIPSolverHighs::addLinearTermToObjective(double coefficient, int variableIndex)
{
    variableCosts.at(variableIndex) += coefficient;
    return (true);
}

bool MIPSolverHighs::addQuadraticTermToObjective([[maybe_unused]] double coefficient,
    [[maybe_unused]] int firstVariableIndex, [[maybe_unused]] int secondVariableIndex)
{
    // Not implemented
    return (false);
}

bool MIPSolverHighs::finalizeObjective(bool isMinimize, double constant)
{
    if(!isMinimize)
    {
        objectiveSense = ObjSense::kMaximize;
        isMinimizationProblem = false;
    }
    else
    {
        objectiveSense = ObjSense::kMinimize;
        isMinimizationProblem = true;
    }

    this->objectiveConstant = constant;

    return (true);
}

bool MIPSolverHighs::initializeConstraint()
{
    currentConstraintTerms.clear();
    return (true);
}

bool MIPSolverHighs::addLinearTermToConstraint(double coefficient, int variableIndex)
{
    if(currentConstraintTerms.find(variableIndex) == currentConstraintTerms.end())
    {
        currentConstraintTerms.insert(std::make_pair(variableIndex, coefficient));
    }
    else
    {
        currentConstraintTerms[variableIndex] += coefficient;
    }

    return (true);
}

bool MIPSolverHighs::addQuadraticTermToConstraint([[maybe_unused]] double coefficient,
    [[maybe_unused]] int firstVariableIndex, [[maybe_unused]] int secondVariableIndex)
{
    // Not implemented
    return (false);
}

bool MIPSolverHighs::finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant)
{
    if(valueLHS <= valueRHS)
    {
        constraintLBs.push_back(valueLHS - constant);
        constraintUBs.push_back(valueRHS - constant);
    }
    else
    {
        constraintLBs.push_back(valueRHS - constant);
        constraintUBs.push_back(valueLHS - constant);
    }

    constraintNames.push_back(name);

    aMatrixStart.push_back(aMatrixStart.back() + currentConstraintTerms.size());

    for(auto const& [variableIndex, coefficient] : currentConstraintTerms)
    {
        aMatrixIndex.push_back(variableIndex);
        aMatrixValue.push_back(coefficient);
    }

    allowRepairOfConstraint.push_back(false);
    numberOfConstraints++;

    return (true);
}

bool MIPSolverHighs::finalizeProblem()
{
    highsModel.lp_.a_matrix_.start_ = aMatrixStart;
    highsModel.lp_.a_matrix_.index_ = aMatrixIndex;
    highsModel.lp_.a_matrix_.value_ = aMatrixValue;
    highsModel.lp_.a_matrix_.format_ = MatrixFormat::kRowwise;
    highsModel.lp_.a_matrix_.num_col_ = numberOfVariables;
    highsModel.lp_.a_matrix_.num_row_ = numberOfConstraints;

    highsModel.lp_.num_col_ = numberOfVariables;
    highsModel.lp_.col_lower_ = variableLowerBounds;
    highsModel.lp_.col_upper_ = variableUpperBounds;
    highsModel.lp_.col_names_ = variableNames;
    highsModel.lp_.integrality_ = variableTypesHighs;

    highsModel.lp_.num_row_ = numberOfConstraints;
    highsModel.lp_.row_lower_ = constraintLBs;
    highsModel.lp_.row_upper_ = constraintUBs;
    highsModel.lp_.row_names_ = constraintNames;

    highsModel.lp_.offset_ = objectiveConstant;
    highsModel.lp_.col_cost_ = variableCosts;
    highsModel.lp_.sense_ = objectiveSense;

    if(highsInstance.passModel(highsModel) != HighsStatus::kOk)
        return (false);

    setSolutionLimit(1);

    return (true);
}

void MIPSolverHighs::initializeSolverSettings()
{
    highsInstance.setOptionValue("highs_debug_level", 0);
    highsInstance.setOptionValue("output_flag", true);
}

int MIPSolverHighs::addLinearConstraint(
    const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan, bool allowRepair)
{
    VectorInteger variableIndexes;
    VectorDouble coefficients;

    int numConstraintsBefore = highsInstance.getNumRow();

    for(auto E : elements)
    {
        variableIndexes.push_back(E.first);
        coefficients.push_back(E.second);
    }

    // Adds the cutting plane
    if(isGreaterThan)
    {
        highsInstance.addRow(
            -constant, highsInstance.getInfinity(), variableIndexes.size(), &variableIndexes[0], &coefficients[0]);
    }
    else
    {
        highsInstance.addRow(
            -highsInstance.getInfinity(), -constant, variableIndexes.size(), &variableIndexes[0], &coefficients[0]);
    }

    if(highsInstance.getNumRow() > numConstraintsBefore)
    {
        allowRepairOfConstraint.push_back(allowRepair);
    }
    else
    {
        env->output->outputWarning("        Linear constraint  not added by Highs");
        return (-1);
    }

    return (highsInstance.getNumRow() - 1);
}

bool MIPSolverHighs::addSpecialOrderedSet(E_SOSType type, VectorInteger variableIndexes, VectorDouble variableWeights)
{
    // Not implemented
    return (true);
}

void MIPSolverHighs::activateDiscreteVariables(bool activate)
{
    if(env->reformulatedProblem->properties.numberOfSemiintegerVariables > 0
        || env->reformulatedProblem->properties.numberOfSemicontinuousVariables > 0)
        return;

    assert(variableTypes.size() == variableTypesHighs.size());

    if(activate)
    {
        env->output->outputDebug("        Activating MIP strategy");

        for(int i = 0; i < numberOfVariables; i++)
        {
            assert(variableTypes.at(i) != E_VariableType::Semicontinuous
                && variableTypes.at(i) != E_VariableType::Semiinteger);

            if(variableTypes.at(i) == E_VariableType::Integer || variableTypes.at(i) == E_VariableType::Binary)
            {
                this->variableTypesHighs.at(i) = HighsVarType::kInteger;
                highsInstance.changeColIntegrality(i, HighsVarType::kInteger);
            }
        }

        discreteVariablesActivated = true;
    }
    else
    {
        env->output->outputDebug("        Activating LP strategy");
        for(int i = 0; i < numberOfVariables; i++)
        {
            assert(variableTypes.at(i) != E_VariableType::Semicontinuous
                && variableTypes.at(i) != E_VariableType::Semiinteger);

            if(variableTypes.at(i) == E_VariableType::Integer || variableTypes.at(i) == E_VariableType::Binary)
            {
                this->variableTypesHighs.at(i) = HighsVarType::kContinuous;
                highsInstance.changeColIntegrality(i, HighsVarType::kContinuous);
            }
        }

        discreteVariablesActivated = false;
    }
}

E_ProblemSolutionStatus MIPSolverHighs::getSolutionStatus()
{

    E_ProblemSolutionStatus MIPSolutionStatus;

    auto modelStatus = highsInstance.getModelStatus();

    if(modelStatus == HighsModelStatus::kOptimal)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
    }
    else if(modelStatus == HighsModelStatus::kInfeasible)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
    }
    else if(modelStatus == HighsModelStatus::kUnbounded || modelStatus == HighsModelStatus::kUnboundedOrInfeasible)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if(modelStatus == HighsModelStatus::kTimeLimit)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
    }
    else
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
        env->output->outputError(
            fmt::format("        MIP solver return status unknown (HiGHS returned status {}).", modelStatus));
    }

    return (MIPSolutionStatus);
}

E_ProblemSolutionStatus MIPSolverHighs::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    cachedSolutionHasChanged = true;

    initializeSolverSettings();

    // TODO: Add MIP start

    highsReturnStatus = highsInstance.run();
    assert(highsReturnStatus == HighsStatus::kOk);
    MIPSolutionStatus = getSolutionStatus();

    // TODO:: repair infeasible or unbounded problem (if needed)

    return (MIPSolutionStatus);
}

bool MIPSolverHighs::repairInfeasibility() { return false; }

int MIPSolverHighs::increaseSolutionLimit(int increment)
{
    this->solLimit += increment;

    this->setSolutionLimit(this->solLimit);

    return (this->solLimit);
}

void MIPSolverHighs::setSolutionLimit(long int limit) { this->solLimit = limit; }

int MIPSolverHighs::getSolutionLimit() { return (this->solLimit); }

void MIPSolverHighs::setTimeLimit(double seconds) { highsInstance.setOptionValue("time_limit", seconds); }

void MIPSolverHighs::setCutOff(double cutOff)
{
    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
        return;

    double cutOffTol = env->settings->getSetting<double>("MIP.CutOff.Tolerance", "Dual");

    if(isMinimizationProblem)
    {
        this->cutOff = cutOff + cutOffTol;

        env->output->outputDebug(fmt::format("        Setting cutoff value to {} for minimization.", this->cutOff));
    }
    else
    {
        this->cutOff = -1 * (cutOff + cutOffTol);

        env->output->outputDebug(fmt::format("        Setting cutoff value to {} for maximization.", this->cutOff));
    }
}

void MIPSolverHighs::setCutOffAsConstraint([[maybe_unused]] double cutOff)
{
    // TODO
    return;
}

void MIPSolverHighs::addMIPStart(VectorDouble point)
{
    // TODO
}

void MIPSolverHighs::writeProblemToFile(std::string filename)
{
    try
    {
        highsInstance.writeModel(filename.c_str());
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when saving model to file in HiGHS", e.what());
    }
}

double MIPSolverHighs::getObjectiveValue(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();

    if(!isMIP && solIdx > 0) // LP problems only have one solution!
    {
        env->output->outputError("        Cannot obtain solution with index " + std::to_string(solIdx)
            + " in HiGHS since the problem is LP/QP!");

        return (NAN);
    }

    double objectiveValue = highsInstance.getInfo().objective_function_value;

    return (objectiveValue);
}

void MIPSolverHighs::deleteMIPStarts() { MIPStart.clear(); }

bool MIPSolverHighs::createIntegerCut(IntegerCut& integerCut)
{
    // TODO
    return (false);
}

VectorDouble MIPSolverHighs::getVariableSolution(int solIdx)
{
    auto solution = highsInstance.getSolution().col_value;
    return (solution);
}

int MIPSolverHighs::getNumberOfSolutions()
{
    int numSols = 1;

    return (numSols);
}

void MIPSolverHighs::fixVariable(int varIndex, double value) { updateVariableBound(varIndex, value, value); }

void MIPSolverHighs::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
    variableLowerBounds[varIndex] = lowerBound;
    variableUpperBounds[varIndex] = upperBound;
}

void MIPSolverHighs::updateVariableLowerBound(int varIndex, double lowerBound)
{
    variableLowerBounds[varIndex] = lowerBound;
}

void MIPSolverHighs::updateVariableUpperBound(int varIndex, double upperBound)
{
    variableUpperBounds[varIndex] = upperBound;
}

PairDouble MIPSolverHighs::getCurrentVariableBounds(int varIndex)
{
    PairDouble tmpBounds;

    try
    {
        tmpBounds.first = variableLowerBounds[varIndex];
        tmpBounds.second = variableUpperBounds[varIndex];
    }
    catch(std::exception& e)
    {
        env->output->outputError(
            "        Error when obtaining variable bounds for variable index" + std::to_string(varIndex) + " in HiGHS",
            e.what());
    }

    return (tmpBounds);
}

bool MIPSolverHighs::supportsQuadraticObjective() { return (false); }

bool MIPSolverHighs::supportsQuadraticConstraints() { return (false); }

double MIPSolverHighs::getUnboundedVariableBoundValue() { return (highsInstance.getInfinity()); }

double MIPSolverHighs::getDualObjectiveValue()
{
    bool isMIP = getDiscreteVariableStatus();
    double objVal = (isMinimizationProblem ? SHOT_DBL_MIN : SHOT_DBL_MAX);

    try
    {
        if(isMIP)
        {
            objVal = highsInstance.getInfo().mip_dual_bound;
        }
        else if(getSolutionStatus() == E_ProblemSolutionStatus::Optimal)
        {
            objVal = highsInstance.getInfo().objective_function_value;
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when obtaining dual objective value in HiGHS", e.what());
    }

    return (objVal);
}

std::pair<VectorDouble, VectorDouble> MIPSolverHighs::presolveAndGetNewBounds()
{
    return (std::make_pair(variableLowerBounds, variableUpperBounds));
}

void MIPSolverHighs::writePresolvedToFile([[maybe_unused]] std::string filename)
{
    // Not implemented
}

void MIPSolverHighs::checkParameters() { }

int MIPSolverHighs::getNumberOfExploredNodes() { return (highsInstance.getInfo().mip_node_count); }

std::string MIPSolverHighs::getSolverVersion()
{
    return (fmt::format("{}.{}.{}", HIGHS_VERSION_MAJOR, HIGHS_VERSION_MINOR, HIGHS_VERSION_PATCH));
}
} // namespace SHOT
