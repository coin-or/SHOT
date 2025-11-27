/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section∏LICENSE
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

// Callback that correctly indents Highs log messages and prints them using SHOT's logging functionality
HighsCallbackFunctionType highsCallback
    = [](int callback_type, const std::string& message, const HighsCallbackOutput* data_out,
          HighsCallbackInput* data_in, void* user_callback_data) {
          HighsMipData callback_data = *(static_cast<HighsMipData*>(user_callback_data));
          auto env = callback_data.env;

          if(callback_type == kCallbackLogging)
          {
              if(!env->settings->getSetting<bool>("Console.DualSolver.Show", "Output"))
                  return;

              auto lines = Utilities::splitStringByCharacter(std::string(message), '\n');

              for(auto const& line : lines)
                  env->output->outputInfo(fmt::format("      | {} ", line));

              // data_in->user_interrupt = false;
              return;
          }

          auto MIPSolver = std::dynamic_pointer_cast<MIPSolverHighs>(env->dualSolver->MIPSolver);

          if(callback_type == kCallbackMipInterrupt)
          {
              if(MIPSolver->currentSolutions.size() >= MIPSolver->getSolutionLimit())
              {
                  env->output->outputDebug(fmt::format("      | solution limit reached "));
                  data_in->user_interrupt = true;
              }
              else
              {
                  data_in->user_interrupt = false;
              }

              return;
          }

          if(callback_type == kCallbackMipSolution)
          {
              std::vector<double> solution = data_out->mip_solution;

              double hashValue = Utilities::calculateHash(solution);

              for(int i = 0; i < MIPSolver->currentSolutions.size(); i++)
              {
                  if(MIPSolver->currentSolutions[i].hashValue == hashValue)
                  {
                      return;
                  }
              }

              SolutionPoint currentSolution;
              currentSolution.objectiveValue = env->reformulatedProblem->objectiveFunction->calculateValue(solution);
              currentSolution.point = solution;
              currentSolution.hashValue = hashValue;
              MIPSolver->currentSolutions.push_back(currentSolution);

              env->output->outputDebug(fmt::format("      | #sols: {} \t obj.val: {:.4f} \t gap: {:.4f} ",
                  MIPSolver->currentSolutions.size(), data_out->objective_function_value, data_out->mip_gap));

              // Sorts the solutions so that the best one is at the first position
              if(env->reformulatedProblem->objectiveFunction->properties.isMinimize)
              {
                  std::sort(MIPSolver->currentSolutions.begin(), MIPSolver->currentSolutions.end(),
                      [](const SolutionPoint& firstSolution, const SolutionPoint& secondSolution) {
                          return (firstSolution.objectiveValue < secondSolution.objectiveValue);
                      });
              }
              else
              {
                  std::sort(MIPSolver->currentSolutions.begin(), MIPSolver->currentSolutions.end(),
                      [](const SolutionPoint& firstSolution, const SolutionPoint& secondSolution) {
                          return (firstSolution.objectiveValue > secondSolution.objectiveValue);
                      });
              }

              /*for(int i = 0; i < MIPSolver->currentSolutions.size(); i++)
               {
                   std::cout << fmt::format("{:.8f} \t {:.8f}  ", MIPSolver->currentSolutions[i].objectiveValue,
                       MIPSolver->currentSolutions[i].hashValue)
                             << std::endl;
               }*/

              // Strange that we need to set this manually
              data_in->user_interrupt = false;

              return;
          }
      };

MIPSolverHighs::MIPSolverHighs(EnvironmentPtr envPtr) { env = envPtr; }

MIPSolverHighs::~MIPSolverHighs() = default;

bool MIPSolverHighs::initializeProblem()
{
    discreteVariablesActivated = true;

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
        isProblemDiscrete = true;
        variableTypesHighs.push_back(HighsVarType::kSemiInteger);
        break;
    case E_VariableType::Semicontinuous:
    {
        isProblemDiscrete = true;
        variableTypesHighs.push_back(HighsVarType::kSemiContinuous);
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
    // TODO: Not implemented
    throw new OperationNotImplementedException("Quadratic objective functions not yet implemented in HiGHS interface.");
    return (false);
}

bool MIPSolverHighs::finalizeObjective(bool isMinimize, double constant)
{
    if(!isMinimize)
    {
        this->objectiveSense = ObjSense::kMaximize;
        isMinimizationProblem = false;
    }
    else
    {
        this->objectiveSense = ObjSense::kMinimize;
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
    // TODO: Not implemented
    throw new OperationNotImplementedException("Quadratic constraints not yet implemented in HiGHS interface.");
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

    if(highsInstance.passModel(highsModel) == HighsStatus::kError)
        return (false);

    setSolutionLimit(1);

    return (true);
}

void MIPSolverHighs::initializeSolverSettings()
{
    highsInstance.setOptionValue(
        "mip_rel_gap", env->settings->getSetting<double>("ObjectiveGap.Relative", "Termination"));
    highsInstance.setOptionValue(
        "mip_abs_gap", env->settings->getSetting<double>("ObjectiveGap.Absolute", "Termination"));
    highsInstance.setOptionValue(
        "mip_feasibility_tolerance", env->settings->getSetting<double>("Tolerance.Integer", "Primal"));

    // Adds a user-provided node limit
    if(auto nodeLimit = env->settings->getSetting<double>("MIP.NodeLimit", "Dual"); nodeLimit > 0)
    {
        if(nodeLimit > SHOT_INT_MAX)
            nodeLimit = SHOT_INT_MAX;

        highsInstance.setOptionValue("mip_max_nodes", (int)nodeLimit);
    }

    highsInstance.setOptionValue(
        "mip_allow_restart", env->settings->getSetting<bool>("Highs.MIPAllowRestart", "Subsolver"));
    highsInstance.setOptionValue(
        "mip_detect_symmetry", env->settings->getSetting<bool>("Highs.MIPDetectSymmetry", "Subsolver"));
    highsInstance.setOptionValue(
        "mip_heuristic_effort", env->settings->getSetting<double>("Highs.MIPHeuristicEffort", "Subsolver"));

    switch(env->settings->getSetting<int>("Highs.Parallel", "Subsolver"))
    {
    case 0:
        highsInstance.setOptionValue("parallel", "off");
        break;
    case 1:
        highsInstance.setOptionValue("parallel", "choose");
        break;
    case 2:
        highsInstance.setOptionValue("parallel", "on");
        break;
    default:
        highsInstance.setOptionValue("parallel", "choose");
        break;
    }

    switch(env->settings->getSetting<int>("Highs.Presolve", "Subsolver"))
    {
    case 0:
        highsInstance.setOptionValue("presolve", "off");
        break;
    case 1:
        highsInstance.setOptionValue("presolve", "choose");
        break;
    case 2:
        highsInstance.setOptionValue("presolve", "on");
        break;
    default:
        highsInstance.setOptionValue("presolve", "choose");
        break;
    }

    highsInstance.setOptionValue("threads", env->settings->getSetting<int>("MIP.NumberOfThreads", "Dual"));

    // highsInstance.setOptionValue("simplex_strategy", 0);
    // highsInstance.setOptionValue("solver", "choose");
    // highsInstance.setOptionValue("primal_feasibility_tolerance", 1e-6);
    // highsInstance.setOptionValue("dual_feasibility_tolerance", 1e-6);

    // highsInstance.setOptionValue("highs_debug_level", 3);
    // highsInstance.setOptionValue("mip_report_level", 2);
    // highsInstance.setOptionValue("output_flag", true);

    highsCallbackData.env = env;

    highsInstance.setCallback(highsCallback, reinterpret_cast<void*>(&highsCallbackData));
    highsInstance.startCallback(kCallbackMipSolution);
    highsInstance.startCallback(kCallbackMipInterrupt);
    highsInstance.startCallback(kCallbackLogging);
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
        highsInstance.passRowName(highsInstance.getNumRow() - 1, name);
        allowRepairOfConstraint.push_back(allowRepair);
    }
    else
    {
        env->output->outputWarning("        Linear constraint not added by Highs");
        return (-1);
    }

    return (highsInstance.getNumRow() - 1);
}

bool MIPSolverHighs::addSpecialOrderedSet(E_SOSType type, VectorInteger variableIndexes, VectorDouble variableWeights)
{
    // TODO: Not implemented
    throw new OperationNotImplementedException(
        "Special ordered set functionality not yet implemented in HiGHS interface.");
    return (false);
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
    else if(modelStatus == HighsModelStatus::kObjectiveBound)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::CutOff;
    }
    else if(modelStatus == HighsModelStatus::kObjectiveTarget)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::CutOff;
    }
    else if(modelStatus == HighsModelStatus::kTimeLimit)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
    }
    else if(modelStatus == HighsModelStatus::kIterationLimit)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::IterationLimit;
    }
    else if(modelStatus == HighsModelStatus::kSolutionLimit)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
    }
    else if(modelStatus == HighsModelStatus::kInterrupt)
    {
        // Since we interrup in the callback
        MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
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
    currentSolutions.clear();

    // HighsLp lp = highsInstance.getLp();

    highsReturnStatus = highsInstance.run();
    MIPSolutionStatus = getSolutionStatus();

    // TODO:: repair infeasible or unbounded problem (if needed)

    return (MIPSolutionStatus);
}

bool MIPSolverHighs::repairInfeasibility()
{
    throw new OperationNotImplementedException(
        "Cannot repair infeasible dual problem as HiGHS interface is not yet fully implemented.");
    return (false);
}

int MIPSolverHighs::increaseSolutionLimit(int increment)
{
    this->solutionLimit += increment;

    return (this->solutionLimit);
}

void MIPSolverHighs::setSolutionLimit(long int limit)
{
    if(limit > kHighsIInf)
    {
        this->solutionLimit = kHighsIInf;
    }
    else
    {
        this->solutionLimit = limit;
    }
}

int MIPSolverHighs::getSolutionLimit() { return (this->solutionLimit); }

void MIPSolverHighs::setTimeLimit(double seconds) { highsInstance.setOptionValue("time_limit", seconds); }

void MIPSolverHighs::setCutOff(double cutOff)
{
    // TODO: Some problems with maximization problems and cutoff, disabling for now
    return;

    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
        return;

    double cutOffTol = env->settings->getSetting<double>("MIP.CutOff.Tolerance", "Dual");
    double cutOffWithTol = (isMinimizationProblem) ? cutOff + cutOffTol : cutOff - cutOffTol;

    if(isMinimizationProblem)
    {
        highsInstance.setOptionValue("objective_bound", cutOffWithTol);

        env->output->outputInfo(fmt::format("        Setting cutoff value to {} for minimization.", cutOffWithTol));
    }
    else
    {
        highsInstance.setOptionValue("objective_bound", cutOffWithTol);

        env->output->outputInfo(fmt::format("        Setting cutoff value to {} for maximization.", cutOffWithTol));
    }
}

void MIPSolverHighs::setCutOffAsConstraint(double cutOff)
{
    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
        return;

    if(!cutOffConstraintDefined)
    {
        VectorInteger variableIndexes;
        VectorDouble coefficients;

        int numConstraintsBefore = highsInstance.getNumRow();

        for(size_t i = 0; i < variableCosts.size(); i++)
        {
            if(variableCosts[i] != 0.0)
            {
                variableIndexes.push_back(i);
                coefficients.push_back(variableCosts[i]);
            }
        }

        if(isMinimizationProblem)
        {
            highsInstance.addRow(-highsInstance.getInfinity(), (cutOff - this->objectiveConstant),
                variableIndexes.size(), &variableIndexes[0], &coefficients[0]);
            env->output->outputDebug(
                "        Setting cutoff constraint to " + Utilities::toString(cutOff) + " for minimization.");
        }
        else
        {
            highsInstance.addRow(cutOff - this->objectiveConstant, highsInstance.getInfinity(), variableIndexes.size(),
                &variableIndexes[0], &coefficients[0]);
            env->output->outputDebug(
                "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for maximization.");
        }

        allowRepairOfConstraint.push_back(false);

        cutOffConstraintDefined = true;
        cutOffConstraintIndex = highsInstance.getNumRow() - 1;
        highsInstance.passRowName(cutOffConstraintIndex, "CUTOFF_C");
    }
    else
    {
        if(isMinimizationProblem)
        {
            highsInstance.changeRowBounds(
                cutOffConstraintIndex, -highsInstance.getInfinity(), cutOff - this->objectiveConstant);
            env->output->outputDebug(
                "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for minimization.");
        }
        else
        {
            highsInstance.changeRowBounds(
                cutOffConstraintIndex, cutOff - this->objectiveConstant, highsInstance.getInfinity());
            env->output->outputDebug(
                "        Setting cutoff constraint value to " + Utilities::toString(cutOff) + " for maximization.");
        }
    }

    return;
}

void MIPSolverHighs::addMIPStart(VectorDouble point)
{
    HighsSolution solution;
    solution.col_value = point;

    auto return_status = highsInstance.setSolution(solution);

    if(return_status != HighsStatus::kOk)
        env->output->outputDebug("        Could not add MIP start in Highs.");
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
    double objectiveValue;
    bool isMIP = getDiscreteVariableStatus();

    if(isProblemDiscrete && isMIP)
    {
        objectiveValue = currentSolutions.at(solIdx).objectiveValue;
    }
    else
    {
        objectiveValue = highsInstance.getInfo().objective_function_value;
    }

    return (objectiveValue);
}

void MIPSolverHighs::deleteMIPStarts()
{
    // TODO: not yet implemented

    MIPStart.clear();
}

bool MIPSolverHighs::createIntegerCut(IntegerCut& integerCut)
{
    assert(integerCut.variableValues.size() == (size_t)env->reformulatedProblem->properties.numberOfDiscreteVariables);
    bool allowIntegerCutRepair = env->settings->getSetting<bool>("MIP.InfeasibilityRepair.IntegerCuts", "Dual");

    int numConstraintsBefore = highsInstance.getNumRow();
    int constraintCounter = highsInstance.getNumRow();

    // Verify that no integer values are outside of variable bounds
    for(size_t i = 0; i < integerCut.variableIndexes.size(); i++)
    {
        auto VAR = env->reformulatedProblem->getVariable(integerCut.variableIndexes[i]);
        int variableValue = integerCut.variableValues[i];

        if(variableValue < VAR->lowerBound || variableValue > VAR->upperBound)
            return (false);
    }

    try
    {
        if(integerCut.areAllVariablesBinary) // Integer cut for problem with binary variables only
        {
            size_t index = 0;
            VectorInteger cutIndexes;
            VectorDouble cutCoeffs;

            for(auto& VAR : env->reformulatedProblem->allVariables)
            {
                if(!(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
                       || VAR->properties.type == E_VariableType::Semiinteger))
                    continue;

                int variableValue = integerCut.variableValues[index];

                if(variableValue == 1.0)
                {
                    cutIndexes.push_back(VAR->index);
                    cutCoeffs.push_back(1.0);
                }
                else if(variableValue == 0.0)
                {
                    cutIndexes.push_back(VAR->index);
                    cutCoeffs.push_back(-1.0);
                }
                else
                {
                    env->output->outputDebug("        Integer cut not added by HiGHS ");
                    return (false);
                }

                index++;
            }

            int tmpNumConstraints = highsInstance.getNumRow();

            highsInstance.addRow(-highsInstance.getInfinity(), integerCut.variableValues.size() - 1.0,
                cutIndexes.size(), cutIndexes.data(), cutCoeffs.data());

            if(highsInstance.getNumRow() > tmpNumConstraints)
            {
                highsInstance.passRowName(
                    constraintCounter, fmt::format("IC_{}", env->solutionStatistics.numberOfIntegerCuts));
                allowRepairOfConstraint.push_back(allowIntegerCutRepair);
                integerCuts.push_back(constraintCounter);
                constraintCounter++;
            }
        }
        else // Integer cut for problem with general integers
        {
            env->output->outputInfo("        Adding general integer cut in HiGHS ");
            size_t index = 0;
            VectorInteger cutIndexes;
            VectorDouble cutCoeffs;
            double sumLB = 0.0;
            double sumUB = 0.0;

            for(auto& I : integerCut.variableIndexes)
            {
                auto VAR = env->reformulatedProblem->getVariable(I);
                int variableValue = integerCut.variableValues[index];

                assert(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
                    || VAR->properties.type == E_VariableType::Semiinteger);

                if(variableValue == VAR->upperBound)
                {
                    sumUB += VAR->upperBound;
                    cutIndexes.push_back(VAR->index);
                    cutCoeffs.push_back(-1.0);
                }
                else if(variableValue == VAR->lowerBound)
                {
                    sumLB -= VAR->lowerBound;
                    cutIndexes.push_back(VAR->index);
                    cutCoeffs.push_back(1.0);
                }
                else
                {
                    int wIndex = numberOfVariables;
                    int vIndex = numberOfVariables + 1;
                    numberOfVariables += 2;

                    double M1 = 2 * (variableValue - VAR->lowerBound);
                    double M2 = 2 * (VAR->upperBound - variableValue);

                    // Add auxiliary variables w (continuous) and v (binary)
                    highsInstance.addCol(0.0, 0.0, highsInstance.getInfinity(), 0, nullptr, nullptr);
                    highsInstance.addCol(0.0, 0.0, 1.0, 0, nullptr, nullptr);
                    highsInstance.changeColIntegrality(wIndex, HighsVarType::kContinuous);
                    highsInstance.changeColIntegrality(vIndex, HighsVarType::kInteger);
                    highsInstance.passColName(
                        wIndex, fmt::format("wIC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index));
                    highsInstance.passColName(
                        vIndex, fmt::format("vIC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index));

                    cutIndexes.push_back(wIndex);
                    cutCoeffs.push_back(1.0);

                    // Constraint 1a: x + w >= variableValue  =>  -w <= x - variableValue
                    VectorInteger cut1aIndexes = { VAR->index, wIndex };
                    VectorDouble cut1aCoeffs = { 1.0, 1.0 };
                    int tmpNumConstraints = highsInstance.getNumRow();
                    highsInstance.addRow(variableValue, highsInstance.getInfinity(), cut1aIndexes.size(),
                        cut1aIndexes.data(), cut1aCoeffs.data());

                    if(highsInstance.getNumRow() > tmpNumConstraints)
                    {
                        highsInstance.passRowName(constraintCounter,
                            fmt::format("IC{}_{}_1a", env->solutionStatistics.numberOfIntegerCuts, index));
                        allowRepairOfConstraint.push_back(false);
                        integerCuts.push_back(constraintCounter);
                        constraintCounter++;
                    }

                    // Constraint 1b: x - w <= variableValue
                    VectorInteger cut1bIndexes = { VAR->index, wIndex };
                    VectorDouble cut1bCoeffs = { 1.0, -1.0 };
                    tmpNumConstraints = highsInstance.getNumRow();
                    highsInstance.addRow(-highsInstance.getInfinity(), variableValue, cut1bIndexes.size(),
                        cut1bIndexes.data(), cut1bCoeffs.data());

                    if(highsInstance.getNumRow() > tmpNumConstraints)
                    {
                        highsInstance.passRowName(constraintCounter,
                            fmt::format("IC{}_{}_1b", env->solutionStatistics.numberOfIntegerCuts, index));
                        allowRepairOfConstraint.push_back(false);
                        integerCuts.push_back(constraintCounter);
                        constraintCounter++;
                    }

                    // Constraint 2: w - x + M1*v <= -variableValue + M1
                    VectorInteger cut2Indexes = { wIndex, VAR->index, vIndex };
                    VectorDouble cut2Coeffs = { 1.0, -1.0, M1 };
                    tmpNumConstraints = highsInstance.getNumRow();
                    highsInstance.addRow(-highsInstance.getInfinity(), -variableValue + M1, cut2Indexes.size(),
                        cut2Indexes.data(), cut2Coeffs.data());

                    if(highsInstance.getNumRow() > tmpNumConstraints)
                    {
                        highsInstance.passRowName(constraintCounter,
                            fmt::format("IC{}_{}_2", env->solutionStatistics.numberOfIntegerCuts, index));
                        allowRepairOfConstraint.push_back(false);
                        integerCuts.push_back(constraintCounter);
                        constraintCounter++;
                    }

                    // Constraint 3: w + x - M2*v <= variableValue
                    VectorInteger cut3Indexes = { wIndex, VAR->index, vIndex };
                    VectorDouble cut3Coeffs = { 1.0, 1.0, -M2 };
                    tmpNumConstraints = highsInstance.getNumRow();
                    highsInstance.addRow(-highsInstance.getInfinity(), variableValue, cut3Indexes.size(),
                        cut3Indexes.data(), cut3Coeffs.data());

                    if(highsInstance.getNumRow() > tmpNumConstraints)
                    {
                        highsInstance.passRowName(constraintCounter,
                            fmt::format("IC{}_{}_3", env->solutionStatistics.numberOfIntegerCuts, index));
                        allowRepairOfConstraint.push_back(false);
                        integerCuts.push_back(constraintCounter);
                        constraintCounter++;
                    }
                }

                index++;
            }

            // Final constraint: sum >= 1 - sumLB - sumUB
            int tmpNumConstraints = highsInstance.getNumRow();
            highsInstance.addRow(
                1 - sumLB - sumUB, highsInstance.getInfinity(), cutIndexes.size(), cutIndexes.data(), cutCoeffs.data());

            if(highsInstance.getNumRow() > tmpNumConstraints)
            {
                highsInstance.passRowName(
                    constraintCounter, fmt::format("IC{}_4", env->solutionStatistics.numberOfIntegerCuts));
                allowRepairOfConstraint.push_back(allowIntegerCutRepair);
                integerCuts.push_back(constraintCounter);
                constraintCounter++;
            }
        }

        if(constraintCounter == numConstraintsBefore)
        {
            env->output->outputDebug("        Integer cut not added by HiGHS");
            return (false);
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when adding integer cut in HiGHS: ", e.what());
        return (false);
    }

    return (true);
}

VectorDouble MIPSolverHighs::getVariableSolution(int solIdx)
{
    VectorDouble solution;
    bool isMIP = getDiscreteVariableStatus();

    if(isProblemDiscrete && isMIP)
    {
        solution = currentSolutions.at(solIdx).point;
    }
    else
    {
        solution = highsInstance.getSolution().col_value;
    }

    return (solution);
}

int MIPSolverHighs::getNumberOfSolutions()
{
    int numSols = 0;
    bool isMIP = getDiscreteVariableStatus();

    if(isProblemDiscrete && isMIP)
    {
        numSols = currentSolutions.size();
    }
    else
    {
        // LP problem
        numSols = 1;
        // TODO better way?
    }

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

    tmpBounds.first = variableLowerBounds[varIndex];
    tmpBounds.second = variableUpperBounds[varIndex];

    return (tmpBounds);
}

bool MIPSolverHighs::supportsQuadraticObjective() { return (false); }

bool MIPSolverHighs::supportsQuadraticConstraints() { return (false); }

double MIPSolverHighs::getUnboundedVariableBoundValue() { return (highsInstance.getInfinity()); }

double MIPSolverHighs::getDualObjectiveValue()
{
    bool isMIP = getDiscreteVariableStatus();
    double objVal = (isMinimizationProblem ? SHOT_DBL_MIN : SHOT_DBL_MAX);

    if(isMIP)
    {
        objVal = highsInstance.getInfo().mip_dual_bound;
    }
    else if(getSolutionStatus() == E_ProblemSolutionStatus::Optimal)
    {
        objVal = highsInstance.getInfo().objective_function_value;
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
