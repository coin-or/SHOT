/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverSCIP.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Output.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"
#include "../Model/ObjectiveFunction.h"

#include "scip/cons_nonlinear.h"
#include "scip/scip.h"
#include "scip/scipdefplugins.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

// The data of SHOT's event and message handlers in SCIP, which SCIP declares but leaves to be defined
struct SCIP_EventhdlrData
{
    SHOT::MIPSolverSCIP* solver;
    SHOT::EnvironmentPtr env;
};

struct SCIP_MessagehdlrData
{
    SHOT::EnvironmentPtr env;
};

namespace SHOT
{

namespace
{
    // Throws when a SCIP function fails, so that the calls do not each need their own error handling
    void scipCall(SCIP_RETCODE returnCode, const char* function)
    {
        if(returnCode != SCIP_OKAY)
            throw std::runtime_error(fmt::format("SCIP error {} in {}", static_cast<int>(returnCode), function));
    }

#define SCIP_CHECK(x) scipCall((x), #x)

    // Interrupts the solve when SHOT terminates, e.g., through a callback, and records the number of nodes
    SCIP_DECL_EVENTEXEC(eventExecSHOT)
    {
        (void)event;
        (void)eventdata;

        auto data = SCIPeventhdlrGetData(eventhdlr);

        if(data->env->results->getNumberOfIterations() > 0)
        {
            auto currentIteration = data->env->results->getCurrentIteration();
            currentIteration->numberOfExploredNodes = static_cast<int>(SCIPgetNNodes(scip));
            currentIteration->numberOfOpenNodes = SCIPgetNNodesLeft(scip);
        }

        if(data->solver->terminationHelper && data->solver->terminationHelper->checkTermination())
        {
            data->env->output->outputDebug("        Terminated by user.");
            SCIP_CALL(SCIPinterruptSolve(scip));
        }

        return (SCIP_OKAY);
    }

    SCIP_DECL_EVENTINIT(eventInitSHOT)
    {
        SCIP_CALL(
            SCIPcatchEvent(scip, SCIP_EVENTTYPE_NODESOLVED | SCIP_EVENTTYPE_BESTSOLFOUND, eventhdlr, nullptr, nullptr));

        return (SCIP_OKAY);
    }

    SCIP_DECL_EVENTEXIT(eventExitSHOT)
    {
        SCIP_CALL(SCIPdropEvent(scip, SCIP_EVENTTYPE_NODESOLVED | SCIP_EVENTTYPE_BESTSOLFOUND, eventhdlr, nullptr, -1));

        return (SCIP_OKAY);
    }

    SCIP_DECL_EVENTFREE(eventFreeSHOT)
    {
        (void)scip;

        delete SCIPeventhdlrGetData(eventhdlr);
        SCIPeventhdlrSetData(eventhdlr, nullptr);

        return (SCIP_OKAY);
    }

    // Writes the output of SCIP to SHOT's log, indented as the output of the other MIP solvers. SCIP also writes
    // files, e.g. of the problem, through the message handler, which are written as they are.
    void writeMessage(SCIP_MESSAGEHDLR* messagehdlr, FILE* file, const char* message, bool isWarning)
    {
        if(message == nullptr)
            return;

        if(file != nullptr && file != stdout && file != stderr)
        {
            fputs(message, file);
            return;
        }

        auto data = (SCIPmessagehdlrGetData(messagehdlr));

        if(data == nullptr)
            return;

        for(auto& line : Utilities::splitStringByCharacter(std::string(message), '\n'))
        {
            if(line.empty())
                continue;

            if(isWarning)
                data->env->output->outputDebug(fmt::format("      | {} ", line));
            else
                data->env->output->outputInfo(fmt::format("      | {} ", line));
        }
    }

    SCIP_DECL_MESSAGEINFO(messageInfoSHOT) { writeMessage(messagehdlr, file, msg, false); }

    SCIP_DECL_MESSAGEWARNING(messageWarningSHOT) { writeMessage(messagehdlr, file, msg, true); }

    SCIP_DECL_MESSAGEDIALOG(messageDialogSHOT) { writeMessage(messagehdlr, file, msg, false); }

    SCIP_DECL_MESSAGEHDLRFREE(messageFreeSHOT)
    {
        // SCIPmessagehdlrGetData is a macro without outer parentheses in optimized builds of SCIP
        delete(SCIPmessagehdlrGetData(messagehdlr));
        SCIP_CALL(SCIPmessagehdlrSetData(messagehdlr, nullptr));

        return (SCIP_OKAY);
    }

    // The environment of the SCIP calls running in this thread, whose log gets SCIP's error messages
    thread_local SHOT::Environment* currentEnvironment = nullptr;

    // SCIP prints the location and the text of an error message separately, so they are collected into lines
    thread_local std::string errorLine;

    // SCIP prints its error messages through one method for the whole process, and not through the message handler.
    // They are written to the debug log, since SHOT handles the errors, e.g., by solving the problem again.
    SCIP_DECL_ERRORPRINTING(errorPrintingSHOT)
    {
        (void)data;

        if(currentEnvironment == nullptr)
        {
            fputs(msg, file != nullptr ? file : stderr);
            return;
        }

        errorLine += msg;

        for(auto position = errorLine.find('\n'); position != std::string::npos; position = errorLine.find('\n'))
        {
            auto line = errorLine.substr(0, position);
            errorLine.erase(0, position + 1);

            if(!line.empty())
                currentEnvironment->output->outputDebug(fmt::format("      | {} ", line));
        }
    }

    // Sets the environment that gets SCIP's error messages while SCIP is called
    struct ErrorPrintingScope
    {
        SHOT::Environment* previousEnvironment;

        ErrorPrintingScope(SHOT::Environment* environment) : previousEnvironment(currentEnvironment)
        {
            currentEnvironment = environment;
            SCIPmessageSetErrorPrinting(errorPrintingSHOT, nullptr);
        }

        ~ErrorPrintingScope() { currentEnvironment = previousEnvironment; }
    };

    // SHOT's infinite bounds are given to SCIP as its infinity
    double toSCIPBound(SCIP* scip, double value)
    {
        if(value >= SCIPinfinity(scip))
            return (SCIPinfinity(scip));

        if(value <= -SCIPinfinity(scip))
            return (-SCIPinfinity(scip));

        return (value);
    }
} // namespace

MIPSolverSCIP::SCIPInstance::~SCIPInstance()
{
    if(scip != nullptr)
        SCIPfree(&scip);
}

MIPSolverSCIP::MIPSolverSCIP(EnvironmentPtr envPtr)
{
    env = envPtr;
    terminationHelper = std::make_unique<SCIPTerminationHelper>(env);
}

MIPSolverSCIP::~MIPSolverSCIP() = default;

bool MIPSolverSCIP::initializeProblem()
{
    discreteVariablesActivated = true;
    repairableCuts.clear();

    if(!alreadyInitialized)
    {
        prevSolutionLimit = 1;
        alreadyInitialized = true;
    }

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();

    return (true);
}

bool MIPSolverSCIP::addVariable(
    std::string name, E_VariableType type, double lowerBound, double upperBound, double semiBound)
{
    bool isSemiVariable = (type == E_VariableType::Semicontinuous || type == E_VariableType::Semiinteger);

    if(isSemiVariable && semiBound < 0.0)
    {
        env->output->outputError(" Negative semi-variable branches are not supported by SHOT's SCIP interface.");
        return (false);
    }

    // A semi-variable is zero or in [semiBound, upperBound], so its lower bound is semiBound
    if(isSemiVariable)
        lowerBound = semiBound;

    if(lowerBound < -getUnboundedVariableBoundValue())
        lowerBound = -getUnboundedVariableBoundValue();

    if(upperBound > getUnboundedVariableBoundValue())
        upperBound = getUnboundedVariableBoundValue();

    if(type != E_VariableType::Real)
        isProblemDiscrete = true;

    variableTypes.push_back(type);
    variableNames.push_back(name);
    variableLowerBounds.push_back(lowerBound);
    variableUpperBounds.push_back(upperBound);
    currentLowerBounds.push_back(lowerBound);
    currentUpperBounds.push_back(upperBound);
    objectiveCoefficients.push_back(0.0);
    numberOfVariables++;

    return (true);
}

bool MIPSolverSCIP::initializeObjective()
{
    std::fill(objectiveCoefficients.begin(), objectiveCoefficients.end(), 0.0);
    objectiveQuadraticTerms.clear();
    objectiveConstant = 0.0;

    return (true);
}

bool MIPSolverSCIP::addLinearTermToObjective(double coefficient, int variableIndex)
{
    objectiveCoefficients.at(variableIndex) += coefficient;

    return (true);
}

bool MIPSolverSCIP::addQuadraticTermToObjective(double coefficient, int firstVariableIndex, int secondVariableIndex)
{
    objectiveQuadraticTerms.push_back(QuadraticTerm { firstVariableIndex, secondVariableIndex, coefficient });
    hasQuadraticObjective = true;

    return (true);
}

bool MIPSolverSCIP::finalizeObjective(bool isMinimize, double constant)
{
    objectiveConstant = constant;
    isMinimizationProblem = isMinimize;

    return (true);
}

bool MIPSolverSCIP::initializeConstraint()
{
    currentConstraintTerms.clear();
    currentConstraintQuadraticTerms.clear();

    return (true);
}

bool MIPSolverSCIP::addLinearTermToConstraint(double coefficient, int variableIndex)
{
    currentConstraintTerms[variableIndex] += coefficient;

    return (true);
}

bool MIPSolverSCIP::addQuadraticTermToConstraint(double coefficient, int firstVariableIndex, int secondVariableIndex)
{
    currentConstraintQuadraticTerms.push_back(QuadraticTerm { firstVariableIndex, secondVariableIndex, coefficient });
    hasQuadraticConstraint = true;

    return (true);
}

bool MIPSolverSCIP::finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant)
{
    VectorInteger indexes;
    VectorDouble coefficients;

    for(auto& [index, coefficient] : currentConstraintTerms)
    {
        indexes.push_back(index);
        coefficients.push_back(coefficient);
    }

    // The sides are swapped if they are given in the opposite order, as in the other MIP solvers
    if(valueLHS > valueRHS)
        std::swap(valueLHS, valueRHS);

    double lhs = (valueLHS > SHOT_DBL_MIN) ? valueLHS - constant : -1e20;
    double rhs = (valueRHS < SHOT_DBL_MAX) ? valueRHS - constant : 1e20;

    if(currentConstraintQuadraticTerms.empty())
    {
        addRow(name, indexes, coefficients, lhs, rhs, false);
    }
    else
    {
        quadraticRows.push_back(
            Row { name, std::move(indexes), std::move(coefficients), lhs, rhs, currentConstraintQuadraticTerms });
        modelUpdated = true;
    }

    numberOfConstraints++;

    return (true);
}

bool MIPSolverSCIP::finalizeProblem()
{
    if(env->settings->getSetting<bool>("Dual.TreeStrategy.Multi.Reinitialize"))
    {
        int setSolLimit;
        bool discreteVariablesActivated = getDiscreteVariableStatus();

        if(env->results->getNumberOfIterations() > 0)
        {
            setSolLimit = env->results->getCurrentIteration()->usedMIPSolutionLimit;
            discreteVariablesActivated = env->results->getCurrentIteration()->isMIP();
        }
        else
        {
            setSolLimit = env->settings->getSetting<int>("Dual.MIP.SolutionLimit.Initial");
        }

        setSolutionLimit(setSolLimit);

        if(!discreteVariablesActivated)
            activateDiscreteVariables(false);
    }

    modelUpdated = true;

    return (true);
}

// The settings are given to SCIP when an instance is created for a solve
void MIPSolverSCIP::initializeSolverSettings() { }

int MIPSolverSCIP::addRow(const std::string& name, VectorInteger variableIndexes, VectorDouble coefficients, double lhs,
    double rhs, bool allowRepair)
{
    rows.push_back(Row { name, std::move(variableIndexes), std::move(coefficients), lhs, rhs, { } });
    allowRepairOfConstraint.push_back(allowRepair);
    modelUpdated = true;

    return (static_cast<int>(rows.size()) - 1);
}

int MIPSolverSCIP::addLinearConstraint(
    const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan, bool allowRepair)
{
    VectorInteger indexes;
    VectorDouble coefficients;

    for(auto& [index, coefficient] : elements)
    {
        if(coefficient == 0.0)
            continue;

        indexes.push_back(index);
        coefficients.push_back(coefficient);
    }

    if(isGreaterThan)
        return (addRow(name, indexes, coefficients, -constant, 1e20, allowRepair));

    return (addRow(name, indexes, coefficients, -1e20, -constant, allowRepair));
}

bool MIPSolverSCIP::addSpecialOrderedSet(E_SOSType type, VectorInteger variableIndexes, VectorDouble variableWeights)
{
    if(variableWeights.size() == 0)
    {
        variableWeights.resize(variableIndexes.size());

        for(size_t i = 0; i < variableIndexes.size(); i++)
            variableWeights[i] = static_cast<double>(i + 1);
    }

    assert(variableWeights.size() == variableIndexes.size());

    specialOrderedSets.push_back(SpecialOrderedSet { type, std::move(variableIndexes), std::move(variableWeights) });
    modelUpdated = true;

    return (true);
}

bool MIPSolverSCIP::createIntegerCut(IntegerCut& integerCut)
{
    bool allowIntegerCutRepair = env->settings->getSetting<bool>("Dual.MIP.InfeasibilityRepair.IntegerCuts");

    // Verify that no integer values are outside of variable bounds
    for(size_t i = 0; i < integerCut.variableIndexes.size(); i++)
    {
        auto VAR = env->reformulatedProblem->getVariable(integerCut.variableIndexes[i]);
        int variableValue = integerCut.variableValues[i];

        if(variableValue < VAR->lowerBound || variableValue > VAR->upperBound)
            return (false);
    }

    // The cut sum(terms) + constant >= 1, where the terms are variables at a bound, and the auxiliary variable w with
    // |x - value| = w for the other variables
    std::map<int, double> cutTerms;
    double cutConstant = 0.0;
    size_t index = 0;

    for(auto& I : integerCut.variableIndexes)
    {
        auto VAR = env->reformulatedProblem->getVariable(I);
        int variableValue = integerCut.variableValues[index];

        assert(VAR->properties.type == E_VariableType::Binary || VAR->properties.type == E_VariableType::Integer
            || VAR->properties.type == E_VariableType::Semiinteger);

        if(variableValue == VAR->upperBound)
        {
            cutTerms[I] -= 1.0;
            cutConstant += variableValue;
        }
        else if(variableValue == VAR->lowerBound)
        {
            cutTerms[I] += 1.0;
            cutConstant -= variableValue;
        }
        else
        {
            auto name = fmt::format("IC{}_{}", env->solutionStatistics.numberOfIntegerCuts, index);

            int w = numberOfVariables;
            addVariable("w" + name, E_VariableType::Real, 0.0, getUnboundedVariableBoundValue(), 0.0);
            int v = numberOfVariables;
            addVariable("v" + name, E_VariableType::Binary, 0.0, 1.0, 0.0);
            numberOfIntegerCutVariables += 2;

            cutTerms[w] += 1.0;

            double M1 = 2 * (variableValue - VAR->lowerBound);
            double M2 = 2 * (VAR->upperBound - variableValue);

            // -w <= x - value, x - value <= w, w <= x - value + M1 * (1 - v) and w <= value - x + M2 * v
            integerCuts.push_back(addRow(name + "_1a", { I, w }, { 1.0, 1.0 }, variableValue, 1e20, false));
            integerCuts.push_back(addRow(name + "_1b", { I, w }, { 1.0, -1.0 }, -1e20, variableValue, false));
            integerCuts.push_back(
                addRow(name + "_2", { w, I, v }, { 1.0, -1.0, M1 }, -1e20, M1 - variableValue, false));
            integerCuts.push_back(
                addRow(name + "_3", { w, I, v }, { 1.0, 1.0, -M2 }, -1e20, static_cast<double>(variableValue), false));
        }

        index++;
    }

    VectorInteger indexes;
    VectorDouble coefficients;

    for(auto& [variableIndex, coefficient] : cutTerms)
    {
        if(coefficient == 0.0)
            continue;

        indexes.push_back(variableIndex);
        coefficients.push_back(coefficient);
    }

    integerCuts.push_back(addRow(fmt::format("IC{}_4", env->solutionStatistics.numberOfIntegerCuts), indexes,
        coefficients, 1.0 - cutConstant, 1e20, allowIntegerCutRepair));

    return (true);
}

std::unique_ptr<MIPSolverSCIP::SCIPInstance> MIPSolverSCIP::createInstance(
    bool allowDualReductions, const std::map<int, double>* repairPenalties, std::map<int, SCIP_VAR*>* repairSlacks)
{
    auto result = std::make_unique<SCIPInstance>();
    bool isRepair = (repairPenalties != nullptr);

    SCIP_CHECK(SCIPcreate(&result->scip));
    SCIP* scip = result->scip;

    // The output is written to SHOT's log, or not at all
    bool showOutput = !isRepair && env->settings->getSetting<bool>("Output.Console.DualSolver.Show");

    SCIP_MESSAGEHDLR* messageHandler = nullptr;
    SCIP_CHECK(SCIPmessagehdlrCreate(&messageHandler, TRUE, nullptr, !showOutput, messageWarningSHOT, messageDialogSHOT,
        messageInfoSHOT, messageFreeSHOT, new SCIP_MessagehdlrData { env }));
    SCIP_CHECK(SCIPsetMessagehdlr(scip, messageHandler));
    SCIP_CHECK(SCIPmessagehdlrRelease(&messageHandler));

    SCIP_CHECK(SCIPincludeDefaultPlugins(scip));

    // Set after SCIP's numerical trouble, before SHOT's parameters, so that SHOT's tolerances are kept
    if(useNumericsEmphasis)
        SCIP_CHECK(SCIPsetEmphasis(scip, SCIP_PARAMEMPHASIS_NUMERICS, TRUE));

    SCIP_EVENTHDLR* eventHandler = nullptr;
    SCIP_CHECK(SCIPincludeEventhdlrBasic(scip, &eventHandler, "shot", "Termination and node counts of SHOT",
        eventExecSHOT, new SCIP_EventhdlrData { this, env }));
    SCIP_CHECK(SCIPsetEventhdlrInit(scip, eventHandler, eventInitSHOT));
    SCIP_CHECK(SCIPsetEventhdlrExit(scip, eventHandler, eventExitSHOT));
    SCIP_CHECK(SCIPsetEventhdlrFree(scip, eventHandler, eventFreeSHOT));

    SCIP_CHECK(SCIPsetIntParam(scip, "display/verblevel", showOutput ? SCIP_VERBLEVEL_NORMAL : SCIP_VERBLEVEL_NONE));

    // Limits and tolerances
    SCIP_CHECK(SCIPsetRealParam(scip, "limits/time", std::max(0.0, std::min(timeLimit, 1e20))));

    SCIP_CHECK(SCIPsetRealParam(
        scip, "limits/gap", getToleranceForSolver("Termination.ObjectiveGap.Relative", 0.0, 1e20, "limits/gap")));
    SCIP_CHECK(SCIPsetRealParam(
        scip, "limits/absgap", getToleranceForSolver("Termination.ObjectiveGap.Absolute", 0.0, 1e20, "limits/absgap")));

    // SCIP's integrality tolerance is its feasibility tolerance
    SCIP_CHECK(SCIPsetRealParam(scip, "numerics/feastol",
        std::min(getToleranceForSolver("Primal.Tolerance.LinearConstraint", 1e-17, 1e-3, "numerics/feastol"),
            getToleranceForSolver("Primal.Tolerance.Integer", 1e-17, 1e-3, "numerics/feastol"))));
    SCIP_CHECK(SCIPsetRealParam(scip, "numerics/dualfeastol",
        getToleranceForSolver("Dual.MIP.OptimalityTolerance", 1e-17, 1e-3, "numerics/dualfeastol")));

    if(int nodeLimit = env->settings->getSetting<int>("Dual.MIP.NodeLimit"); nodeLimit > 0)
        SCIP_CHECK(SCIPsetLongintParam(scip, "limits/totalnodes", static_cast<SCIP_Longint>(nodeLimit)));

    // The random seed, where zero means that the default of the solver is kept
    if(int randomSeed = env->settings->getSetting<int>("Dual.MIP.RandomSeed"); randomSeed > 0)
        SCIP_CHECK(SCIPsetIntParam(scip, "randomization/randomseedshift", randomSeed));

    switch(env->settings->getSetting<int>("Subsolver.SCIP.Presolve"))
    {
    case 0:
        SCIP_CHECK(SCIPsetPresolving(scip, SCIP_PARAMSETTING_OFF, TRUE));
        break;
    case 1:
        SCIP_CHECK(SCIPsetPresolving(scip, SCIP_PARAMSETTING_FAST, TRUE));
        break;
    case 3:
        SCIP_CHECK(SCIPsetPresolving(scip, SCIP_PARAMSETTING_AGGRESSIVE, TRUE));
        break;
    default:
        break;
    }

    if(!env->settings->getSetting<bool>("Subsolver.SCIP.MIPAllowRestart"))
        SCIP_CHECK(SCIPsetIntParam(scip, "presolving/maxrestarts", 0));

    if(!env->settings->getSetting<bool>("Subsolver.SCIP.MIPDetectSymmetry"))
        SCIP_CHECK(SCIPsetIntParam(scip, "misc/usesymmetry", 0));

    // Dual reductions can make SCIP report a problem as infeasible or unbounded without telling which
    if(!allowDualReductions)
    {
        SCIP_CHECK(SCIPsetBoolParam(scip, "misc/allowstrongdualreds", FALSE));
        SCIP_CHECK(SCIPsetBoolParam(scip, "misc/allowweakdualreds", FALSE));
    }

    // The problem
    SCIP_CHECK(SCIPcreateProbBasic(scip, "SHOT"));

    if(isRepair)
    {
        SCIP_CHECK(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));
    }
    else
    {
        SCIP_CHECK(SCIPsetObjsense(scip, isMinimizationProblem ? SCIP_OBJSENSE_MINIMIZE : SCIP_OBJSENSE_MAXIMIZE));

        if(objectiveConstant != 0.0)
            SCIP_CHECK(SCIPaddOrigObjoffset(scip, objectiveConstant));
    }

    result->variables.resize(numberOfVariables);

    for(int i = 0; i < numberOfVariables; i++)
    {
        SCIP_VARTYPE type = SCIP_VARTYPE_CONTINUOUS;

        if(discreteVariablesActivated)
        {
            if(variableTypes[i] == E_VariableType::Binary)
                type = SCIP_VARTYPE_BINARY;
            else if(variableTypes[i] == E_VariableType::Integer || variableTypes[i] == E_VariableType::Semiinteger)
                type = SCIP_VARTYPE_INTEGER;
        }

        // A binary variable in SCIP must have bounds in [0, 1]
        if(type == SCIP_VARTYPE_BINARY && (currentLowerBounds[i] < 0.0 || currentUpperBounds[i] > 1.0))
            type = SCIP_VARTYPE_INTEGER;

        // A semi-variable can also be zero, which is its lower bound in SCIP, and in the relaxation it is in
        // [0, upperBound]
        bool isSemiVariable
            = (variableTypes[i] == E_VariableType::Semicontinuous || variableTypes[i] == E_VariableType::Semiinteger);
        double lowerBound = isSemiVariable ? std::min(0.0, currentLowerBounds[i]) : currentLowerBounds[i];

        SCIP_VAR* variable = nullptr;
        SCIP_CHECK(SCIPcreateVarBasic(scip, &variable, variableNames[i].c_str(), toSCIPBound(scip, lowerBound),
            toSCIPBound(scip, currentUpperBounds[i]), isRepair ? 0.0 : objectiveCoefficients[i], type));
        SCIP_CHECK(SCIPaddVar(scip, variable));

        // The semi-variable is zero or at least its semi bound, x <= 0 or x >= semiBound
        if(isSemiVariable && discreteVariablesActivated && currentLowerBounds[i] > 0.0)
        {
            SCIP_VAR* disjunctionVariables[2] = { variable, variable };
            SCIP_BOUNDTYPE boundTypes[2] = { SCIP_BOUNDTYPE_UPPER, SCIP_BOUNDTYPE_LOWER };
            SCIP_Real bounds[2] = { 0.0, currentLowerBounds[i] };

            SCIP_CONS* constraint = nullptr;
            SCIP_CHECK(SCIPcreateConsBasicBounddisjunction(
                scip, &constraint, ("semi_" + variableNames[i]).c_str(), 2, disjunctionVariables, boundTypes, bounds));
            SCIP_CHECK(SCIPaddCons(scip, constraint));
            SCIP_CHECK(SCIPreleaseCons(scip, &constraint));
        }

        // The problem keeps the variable, so it stays valid until the instance is freed
        result->variables[i] = variable;
        SCIP_CHECK(SCIPreleaseVar(scip, &variable));
    }

    std::vector<SCIP_VAR*> constraintVariables;

    for(size_t i = 0; i < rows.size(); i++)
    {
        auto& row = rows[i];

        constraintVariables.clear();

        for(int index : row.variableIndexes)
            constraintVariables.push_back(result->variables[index]);

        SCIP_CONS* constraint = nullptr;
        SCIP_CHECK(SCIPcreateConsBasicLinear(scip, &constraint, row.name.c_str(),
            static_cast<int>(constraintVariables.size()), constraintVariables.data(),
            const_cast<double*>(row.coefficients.data()), toSCIPBound(scip, row.lhs), toSCIPBound(scip, row.rhs)));

        if(isRepair)
        {
            if(auto penalty = repairPenalties->find(static_cast<int>(i)); penalty != repairPenalties->end())
            {
                SCIP_VAR* slack = nullptr;
                SCIP_CHECK(SCIPcreateVarBasic(scip, &slack, ("slack_" + row.name).c_str(), 0.0, SCIPinfinity(scip),
                    penalty->second, SCIP_VARTYPE_CONTINUOUS));
                SCIP_CHECK(SCIPaddVar(scip, slack));

                // A <= constraint is relaxed by decreasing its left-hand side and a >= constraint by increasing it
                bool isLessThan = !SCIPisInfinity(scip, toSCIPBound(scip, row.rhs));
                SCIP_CHECK(SCIPaddCoefLinear(scip, constraint, slack, isLessThan ? -1.0 : 1.0));

                (*repairSlacks)[static_cast<int>(i)] = slack;
                SCIP_CHECK(SCIPreleaseVar(scip, &slack));
            }
        }

        SCIP_CHECK(SCIPaddCons(scip, constraint));
        SCIP_CHECK(SCIPreleaseCons(scip, &constraint));
    }

    for(auto& row : quadraticRows)
    {
        addQuadraticConstraint(scip, result->variables, row.name, row.variableIndexes, row.coefficients,
            row.quadraticTerms, row.lhs, row.rhs);
    }

    // SCIP only has a linear objective function, so the quadratic terms are given by a variable z with
    // quadratic terms - z <= 0 for a minimization problem and >= 0 for a maximization problem
    if(!isRepair && !objectiveQuadraticTerms.empty())
    {
        SCIP_VAR* objectiveVariable = nullptr;
        SCIP_CHECK(SCIPcreateVarBasic(scip, &objectiveVariable, "shot_quadobjvar", -SCIPinfinity(scip),
            SCIPinfinity(scip), 1.0, SCIP_VARTYPE_CONTINUOUS));
        SCIP_CHECK(SCIPaddVar(scip, objectiveVariable));

        addQuadraticConstraint(scip, result->variables, "shot_quadobj", { }, { }, objectiveQuadraticTerms,
            isMinimizationProblem ? -1e20 : 0.0, isMinimizationProblem ? 0.0 : 1e20, objectiveVariable);

        SCIP_CHECK(SCIPreleaseVar(scip, &objectiveVariable));
    }

    for(size_t i = 0; i < specialOrderedSets.size(); i++)
    {
        auto& set = specialOrderedSets[i];

        constraintVariables.clear();

        for(int index : set.variableIndexes)
            constraintVariables.push_back(result->variables[index]);

        SCIP_CONS* constraint = nullptr;
        auto name = fmt::format("sos{}", i);

        if(set.type == E_SOSType::One)
            SCIP_CHECK(SCIPcreateConsBasicSOS1(scip, &constraint, name.c_str(),
                static_cast<int>(constraintVariables.size()), constraintVariables.data(), set.weights.data()));
        else
            SCIP_CHECK(SCIPcreateConsBasicSOS2(scip, &constraint, name.c_str(),
                static_cast<int>(constraintVariables.size()), constraintVariables.data(), set.weights.data()));

        SCIP_CHECK(SCIPaddCons(scip, constraint));
        SCIP_CHECK(SCIPreleaseCons(scip, &constraint));
    }

    bool hasCutOff = cutOffDefined && std::abs(cutOff) < getUnboundedVariableBoundValue();

    if(isRepair)
    {
        // The repair must find a solution that is better than the cutoff, or it finds that no constraint needs to be
        // relaxed. Without a cutoff constraint in the model, the objective function is bounded by a constraint.
        if(hasCutOff && !cutOffConstraintDefined)
        {
            constraintVariables.clear();
            VectorDouble coefficients;

            for(int i = 0; i < numberOfVariables; i++)
            {
                if(objectiveCoefficients[i] == 0.0)
                    continue;

                constraintVariables.push_back(result->variables[i]);
                coefficients.push_back(objectiveCoefficients[i]);
            }

            double bound = cutOff - objectiveConstant;

            if(objectiveQuadraticTerms.empty())
            {
                SCIP_CONS* constraint = nullptr;
                SCIP_CHECK(SCIPcreateConsBasicLinear(scip, &constraint, "CUTOFF_REPAIR",
                    static_cast<int>(constraintVariables.size()), constraintVariables.data(), coefficients.data(),
                    isMinimizationProblem ? -SCIPinfinity(scip) : bound,
                    isMinimizationProblem ? bound : SCIPinfinity(scip)));
                SCIP_CHECK(SCIPaddCons(scip, constraint));
                SCIP_CHECK(SCIPreleaseCons(scip, &constraint));
            }
            else
            {
                VectorInteger indexes;

                for(int i = 0; i < numberOfVariables; i++)
                {
                    if(objectiveCoefficients[i] != 0.0)
                        indexes.push_back(i);
                }

                addQuadraticConstraint(scip, result->variables, "CUTOFF_REPAIR", indexes, coefficients,
                    objectiveQuadraticTerms, isMinimizationProblem ? -1e20 : bound,
                    isMinimizationProblem ? bound : 1e20);
            }
        }
    }
    else if(hasCutOff && objectiveCoefficientsToRestore.empty())
    {
        // The cutoff applies to the objective function, which no longer is the original one when coefficients have
        // been removed to find a point of an unbounded problem
        SCIP_CHECK(SCIPsetObjlimit(scip, cutOff));
    }

    // The starting points, which SCIP counts as found solutions, so the solution limit is increased by their number,
    // since the solve would otherwise stop at a start when the limit is one
    if(!isRepair)
    {
        long numberOfStarts = 0;

        auto addStart = [&](const VectorInteger& indexes, const VectorDouble& values, bool isPartial)
        {
            SCIP_SOL* solution = nullptr;

            if(isPartial)
                SCIP_CHECK(SCIPcreatePartialSol(scip, &solution, nullptr));
            else
                SCIP_CHECK(SCIPcreateSol(scip, &solution, nullptr));

            for(size_t i = 0; i < indexes.size(); i++)
                SCIP_CHECK(SCIPsetSolVal(scip, solution, result->variables[indexes[i]], values[i]));

            SCIP_Bool isStored = FALSE;
            SCIP_CHECK(SCIPaddSolFree(scip, &solution, &isStored));

            if(isStored)
                numberOfStarts++;
        };

        if(!MIPStart.empty())
        {
            VectorInteger indexes(MIPStart.size());

            for(size_t i = 0; i < MIPStart.size(); i++)
                indexes[i] = static_cast<int>(i);

            // The auxiliary variables of integer cuts are not given, so SCIP completes such a start
            addStart(indexes, MIPStart, static_cast<int>(MIPStart.size()) < numberOfVariables);
        }

        if(!partialMIPStartIndexes.empty())
            addStart(partialMIPStartIndexes, partialMIPStartValues, true);

        // The solution limit only applies to discrete problems, as in the other MIP solvers. SCIP solves also a
        // continuous problem with branch and bound, where it would stop at the first solution found.
        bool isDiscrete = discreteVariablesActivated && (isProblemDiscrete || !specialOrderedSets.empty());
        long limit = std::max(1L, solutionLimit) + numberOfStarts;

        SCIP_CHECK(SCIPsetIntParam(scip, "limits/solutions",
            (!isDiscrete || limit >= std::numeric_limits<int>::max()) ? -1 : static_cast<int>(limit)));
    }

    return (result);
}

void MIPSolverSCIP::addQuadraticConstraint(SCIP* scip, const std::vector<SCIP_VAR*>& variables, const std::string& name,
    const VectorInteger& variableIndexes, const VectorDouble& coefficients,
    const std::vector<QuadraticTerm>& quadraticTerms, double lhs, double rhs, SCIP_VAR* objectiveVariable)
{
    std::vector<SCIP_VAR*> linearVariables;
    VectorDouble linearCoefficients(coefficients);

    for(int index : variableIndexes)
        linearVariables.push_back(variables[index]);

    if(objectiveVariable != nullptr)
    {
        linearVariables.push_back(objectiveVariable);
        linearCoefficients.push_back(-1.0);
    }

    std::vector<SCIP_VAR*> firstVariables;
    std::vector<SCIP_VAR*> secondVariables;
    VectorDouble quadraticCoefficients;

    for(auto& term : quadraticTerms)
    {
        firstVariables.push_back(variables[term.firstVariableIndex]);
        secondVariables.push_back(variables[term.secondVariableIndex]);
        quadraticCoefficients.push_back(term.coefficient);
    }

    SCIP_CONS* constraint = nullptr;
    SCIP_CHECK(SCIPcreateConsBasicQuadraticNonlinear(scip, &constraint, name.c_str(),
        static_cast<int>(linearVariables.size()), linearVariables.data(), linearCoefficients.data(),
        static_cast<int>(firstVariables.size()), firstVariables.data(), secondVariables.data(),
        quadraticCoefficients.data(), toSCIPBound(scip, lhs), toSCIPBound(scip, rhs)));
    SCIP_CHECK(SCIPaddCons(scip, constraint));
    SCIP_CHECK(SCIPreleaseCons(scip, &constraint));
}

E_ProblemSolutionStatus MIPSolverSCIP::getSolutionStatus(SCIP* scip)
{
    switch(SCIPgetStatus(scip))
    {
    case SCIP_STATUS_OPTIMAL:
    // The gap limit is SHOT's termination tolerance, as the MIP gap of the other solvers
    case SCIP_STATUS_GAPLIMIT:
        return (E_ProblemSolutionStatus::Optimal);

    case SCIP_STATUS_INFEASIBLE:
        return (E_ProblemSolutionStatus::Infeasible);

    case SCIP_STATUS_UNBOUNDED:
    case SCIP_STATUS_INFORUNBD:
        return (E_ProblemSolutionStatus::Unbounded);

    case SCIP_STATUS_USERINTERRUPT:
    case SCIP_STATUS_TERMINATE:
    case SCIP_STATUS_MEMLIMIT:
        return (E_ProblemSolutionStatus::Abort);

    case SCIP_STATUS_NODELIMIT:
    case SCIP_STATUS_TOTALNODELIMIT:
    case SCIP_STATUS_STALLNODELIMIT:
        return (E_ProblemSolutionStatus::NodeLimit);

    case SCIP_STATUS_TIMELIMIT:
        return (E_ProblemSolutionStatus::TimeLimit);

    case SCIP_STATUS_SOLLIMIT:
    case SCIP_STATUS_BESTSOLLIMIT:
        return (E_ProblemSolutionStatus::SolutionLimit);

    default:
        env->output->outputError(
            "        MIP solver return status " + std::to_string(static_cast<int>(SCIPgetStatus(scip))));
        return (E_ProblemSolutionStatus::Error);
    }
}

E_ProblemSolutionStatus MIPSolverSCIP::getSolutionStatus()
{
    if(!instance)
        return (E_ProblemSolutionStatus::Error);

    auto status = getSolutionStatus(instance->scip);

    // The point is only feasible when the objective function has been changed
    if(status == E_ProblemSolutionStatus::Optimal && !objectiveCoefficientsToRestore.empty())
        status = E_ProblemSolutionStatus::Feasible;

    return (status);
}

E_ProblemSolutionStatus MIPSolverSCIP::solveInstance(bool allowDualReductions)
{
    currentSolutions.clear();
    cachedSolutionHasChanged = true;

    ErrorPrintingScope errorPrinting(env.get());

    instance = createInstance(allowDualReductions);
    SCIP* scip = instance->scip;

    // SCIP gives up on an LP with unresolved numerical trouble, and the problem is then solved once more with SCIP's
    // numerics emphasis, in the time left
    if(auto returnCode = SCIPsolve(scip); returnCode == SCIP_LPERROR && !useNumericsEmphasis)
    {
        env->output->outputDebug("        Numerical trouble in SCIP, so the problem is solved again with numerics "
                                 "emphasis.");

        double originalTimeLimit = timeLimit;
        timeLimit = std::max(0.0, timeLimit - SCIPgetSolvingTime(scip));
        useNumericsEmphasis = true;

        try
        {
            instance = createInstance(allowDualReductions);
            scip = instance->scip;
            SCIP_CHECK(SCIPsolve(scip));
        }
        catch(...)
        {
            timeLimit = originalTimeLimit;
            useNumericsEmphasis = false;
            throw;
        }

        timeLimit = originalTimeLimit;
        useNumericsEmphasis = false;
    }
    else
    {
        SCIP_CHECK(returnCode);
    }

    int numberOfSolutions = SCIPgetNSols(scip);
    SCIP_SOL** solutions = SCIPgetSols(scip);

    for(int i = 0; i < numberOfSolutions; i++)
    {
        SolutionPoint solution;
        solution.point.resize(numberOfVariables);

        SCIP_CHECK(
            SCIPgetSolVals(scip, solutions[i], numberOfVariables, instance->variables.data(), solution.point.data()));
        solution.objectiveValue = SCIPgetSolOrigObj(scip, solutions[i]);

        currentSolutions.push_back(std::move(solution));
    }

    modelUpdated = false;

    return (getSolutionStatus());
}

void MIPSolverSCIP::restoreObjectiveCoefficients()
{
    for(auto& coefficient : objectiveCoefficientsToRestore)
        objectiveCoefficients[coefficient.index] = coefficient.value;

    objectiveCoefficientsToRestore.clear();
}

E_ProblemSolutionStatus MIPSolverSCIP::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;

    try
    {
        restoreObjectiveCoefficients();

        MIPSolutionStatus = solveInstance();

        // An unbounded exact dual problem means that the problem is unbounded, so no point is needed. SCIP can report
        // a problem as infeasible or unbounded because of dual reductions, so this and the problems solved before the
        // main iterations, e.g., the minimax problem, are solved again without them to find out.
        if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded
            && (env->results->getNumberOfIterations() == 0 || env->dualSolver->isDualProblemExact()))
        {
            if(SCIPgetStatus(instance->scip) == SCIP_STATUS_INFORUNBD)
                MIPSolutionStatus = solveInstance(false);
        }
        // To find a feasible point for an unbounded dual problem and not when solving the minimax-problem
        else if(MIPSolutionStatus == E_ProblemSolutionStatus::Unbounded && env->results->getNumberOfIterations() > 0)
        {
            bool problemUpdated = false;

            auto objectiveClass = env->reformulatedProblem->objectiveFunction->properties.classification;

            if((objectiveClass == E_ObjectiveFunctionClassification::Linear
                   || objectiveClass == E_ObjectiveFunctionClassification::Quadratic)
                && (env->reformulatedProblem->objectiveFunction->isUnbounded()
                    || hasUnboundedObjectiveVariableInSolver()))
            {
                for(auto& V : env->reformulatedProblem->allVariables)
                {
                    if(!V->properties.inObjectiveFunction)
                        continue;

                    if(V->isUnbounded() || isVariableUnboundedInSolver(V->getIndex()))
                    {
                        // Temporarily remove unbounded terms from the objective function
                        objectiveCoefficientsToRestore.emplace_back(
                            V->getIndex(), objectiveCoefficients[V->getIndex()]);
                        objectiveCoefficients[V->getIndex()] = 0.0;
                        problemUpdated = true;
                    }
                }
            }
            else if(objectiveClass >= E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear
                && hasDualAuxiliaryObjectiveVariable())
            {
                // The auxiliary variable in the dual problem is unbounded
                updateVariableBound(getDualAuxiliaryObjectiveVariableIndex(), -getUnboundedVariableBoundValue() / 1.1,
                    getUnboundedVariableBoundValue() / 1.1);
                problemUpdated = true;
            }

            if(problemUpdated)
            {
                MIPSolutionStatus = solveInstance();

                // Might not have iterations if we are using the minimax solver
                if(env->results->iterations.size() > 0)
                    env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed = true;
            }
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when solving MIP/LP problem with SCIP", e.what());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    // A partial start is only used in the next solve
    partialMIPStartIndexes.clear();
    partialMIPStartValues.clear();

    return (MIPSolutionStatus);
}

bool MIPSolverSCIP::repairInfeasibility()
{
    if(env->dualSolver->generatedHyperplanes.size() == 0)
        return (false);

    try
    {
        int numOrigConstraints = env->reformulatedProblem->properties.numberOfLinearConstraints;
        int numCurrConstraints = static_cast<int>(rows.size());

        // Newer constraints get smaller penalties, so that they are relaxed first
        std::map<int, double> penalties;

        for(int i = numOrigConstraints; i < numCurrConstraints; i++)
        {
            if(allowRepairOfConstraint[i])
                penalties[i] = 1.0 / (static_cast<double>(i - numOrigConstraints) + 1.0);
        }

        if(penalties.empty())
        {
            env->output->outputDebug("        No constraints available for repair.");
            return (false);
        }

        // Saves the relaxation weights to a file
        if(env->settings->getSetting<bool>("Output.Debug.Enable"))
        {
            VectorString constraints;
            VectorDouble relaxParameters;

            for(auto& [index, penalty] : penalties)
            {
                constraints.push_back(rows[index].name);
                relaxParameters.push_back(penalty);
            }

            auto filename = fmt::format("{}/dualiter{}_infeasrelaxweights.txt",
                env->settings->getSetting<std::string>("Output.Debug.Path"),
                env->results->getCurrentIteration()->iterationNumber - 1);

            Utilities::saveVariablePointVectorToFile(relaxParameters, constraints, filename);
        }

        ErrorPrintingScope errorPrinting(env.get());

        std::map<int, SCIP_VAR*> slacks;
        auto repairInstance = createInstance(true, &penalties, &slacks);
        SCIP* scip = repairInstance->scip;

        if(env->settings->getSetting<bool>("Output.Debug.Enable"))
        {
            auto filename = fmt::format("{}/dualiter{}_infeasrelax.lp",
                env->settings->getSetting<std::string>("Output.Debug.Path"),
                env->results->getCurrentIteration()->iterationNumber - 1);

            SCIP_CHECK(SCIPwriteOrigProblem(scip, filename.c_str(), nullptr, FALSE));
        }

        SCIP_CHECK(SCIPsolve(scip));

        if(SCIPgetStatus(scip) != SCIP_STATUS_OPTIMAL && SCIPgetStatus(scip) != SCIP_STATUS_GAPLIMIT)
        {
            env->output->outputDebug("        Could not repair the infeasible dual problem.");
            return (false);
        }

        SCIP_SOL* solution = SCIPgetBestSol(scip);
        int numRepairs = 0;

        for(auto& [index, slack] : slacks)
        {
            double slackValue = SCIPgetSolVal(scip, solution, slack);

            if(slackValue <= 0.0)
                continue;

            auto& row = rows[index];

            // A <= constraint, e.g. a cut, is relaxed by increasing its right-hand side and a >= constraint, e.g.
            // an integer cut, by decreasing its left-hand side
            if(row.rhs < 1e20)
            {
                row.rhs += 1.5 * slackValue;
                env->output->outputDebug("        Constraint: " + row.name
                    + " repaired with infeasibility = " + std::to_string(1.5 * slackValue));
            }
            else
            {
                row.lhs -= 1.5 * slackValue;
                env->output->outputDebug("        Constraint: " + row.name
                    + " repaired with infeasibility = " + std::to_string(-1.5 * slackValue));
            }

            numRepairs++;
        }

        env->results->getCurrentIteration()->numberOfInfeasibilityRepairedConstraints = numRepairs;

        if(numRepairs == 0)
        {
            env->output->outputDebug("        Could not repair the infeasible dual problem.");
            return (false);
        }

        modelUpdated = true;

        if(env->settings->getSetting<bool>("Output.Debug.Enable"))
        {
            auto filename
                = fmt::format("{}/dualiter{}_repaired", env->settings->getSetting<std::string>("Output.Debug.Path"),
                    env->results->getCurrentIteration()->iterationNumber - 1);

            writeProblemToDebugFiles(filename);
        }

        env->output->outputDebug("        Number of constraints modified: " + std::to_string(numRepairs));

        return (true);
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when trying to repair infeasibility", e.what());
    }

    return (false);
}

VectorDouble MIPSolverSCIP::getVariableSolution(int solIdx)
{
    if(solIdx < 0 || solIdx >= static_cast<int>(currentSolutions.size()))
    {
        env->output->outputError("        Error when reading solution with index " + std::to_string(solIdx));
        return (VectorDouble(numberOfVariables, 0.0));
    }

    return (currentSolutions[solIdx].point);
}

int MIPSolverSCIP::getNumberOfSolutions() { return (static_cast<int>(currentSolutions.size())); }

double MIPSolverSCIP::getObjectiveValue(int solIdx)
{
    if(solIdx < 0 || solIdx >= static_cast<int>(currentSolutions.size()))
    {
        env->output->outputError(
            "        Error when obtaining objective value for solution index " + std::to_string(solIdx));
        return (NAN);
    }

    // With removed objective coefficients, the value of the original objective function is calculated
    if(!objectiveCoefficientsToRestore.empty())
        return (calculateObjectiveValue(currentSolutions[solIdx].point));

    return (currentSolutions[solIdx].objectiveValue);
}

double MIPSolverSCIP::calculateObjectiveValue(const VectorDouble& point)
{
    double value = objectiveConstant;

    for(int i = 0; i < numberOfVariables; i++)
        value += objectiveCoefficients[i] * point[i];

    for(auto& coefficient : objectiveCoefficientsToRestore)
        value += coefficient.value * point[coefficient.index];

    for(auto& term : objectiveQuadraticTerms)
        value += term.coefficient * point[term.firstVariableIndex] * point[term.secondVariableIndex];

    return (value);
}

void MIPSolverSCIP::activateDiscreteVariables(bool activate)
{
    if(activate)
        env->output->outputDebug("        Activating MIP strategy.");
    else
        env->output->outputDebug("        Activating LP strategy.");

    discreteVariablesActivated = activate;
    modelUpdated = true;
}

int MIPSolverSCIP::increaseSolutionLimit(int increment)
{
    solutionLimit += increment;

    return (static_cast<int>(std::min<long>(solutionLimit, std::numeric_limits<int>::max())));
}

void MIPSolverSCIP::setSolutionLimit(long limit) { solutionLimit = limit; }

int MIPSolverSCIP::getSolutionLimit()
{
    return (static_cast<int>(std::min<long>(solutionLimit, std::numeric_limits<int>::max())));
}

void MIPSolverSCIP::setTimeLimit(double seconds) { timeLimit = (seconds > 0) ? seconds : 0.0; }

void MIPSolverSCIP::setCutOff(double cutOff)
{
    if(std::abs(cutOff) > 1e20)
        return;

    // The cutoff given already includes the tolerance Dual.MIP.CutOff.Tolerance
    this->cutOff = cutOff;
    cutOffDefined = true;

    env->output->outputDebug(fmt::format(
        "        Setting cutoff value to {} for {}.", cutOff, isMinimizationProblem ? "minimization" : "maximization"));
}

void MIPSolverSCIP::setCutOffAsConstraint(double cutOff)
{
    if(cutOff == SHOT_DBL_MAX || cutOff == SHOT_DBL_MIN)
        return;

    // With a quadratic objective function, the objective limit of SCIP is used, since it applies to the whole
    // objective function, including the variable that gives the quadratic terms
    if(!objectiveQuadraticTerms.empty())
    {
        setCutOff(cutOff);
        return;
    }

    // The constraint is objective(x) <= cutoff for a minimization problem and -objective(x) <= -cutoff for a
    // maximization problem
    double sign = isMinimizationProblem ? 1.0 : -1.0;
    double rhs = sign * (cutOff - objectiveConstant);

    if(!cutOffConstraintDefined)
    {
        VectorInteger indexes;
        VectorDouble coefficients;

        for(int i = 0; i < numberOfVariables; i++)
        {
            if(objectiveCoefficients[i] == 0.0)
                continue;

            indexes.push_back(i);
            coefficients.push_back(sign * objectiveCoefficients[i]);
        }

        cutOffConstraintIndex = addRow("CUTOFF_C", indexes, coefficients, -1e20, rhs, false);
        cutOffConstraintDefined = true;
    }
    else
    {
        rows[cutOffConstraintIndex].rhs = rhs;
        modelUpdated = true;
    }

    env->output->outputDebug(fmt::format("        Setting cutoff constraint to {} for {}.", cutOff,
        isMinimizationProblem ? "minimization" : "maximization"));
}

void MIPSolverSCIP::addMIPStart(VectorDouble point)
{
    assert(point.size() == static_cast<size_t>(this->numberOfVariables - numberOfIntegerCutVariables));

    // A new start replaces the previous one, as in the other MIP solvers
    MIPStart = std::move(point);

    env->output->outputDebug("        Added MIP starting point.");
}

void MIPSolverSCIP::addPartialMIPStart(VectorInteger variableIndexes, VectorDouble variableValues)
{
    assert(variableIndexes.size() == variableValues.size());

    partialMIPStartIndexes = std::move(variableIndexes);
    partialMIPStartValues = std::move(variableValues);

    env->output->outputDebug("        Added partial MIP starting point.");
}

void MIPSolverSCIP::deleteMIPStarts()
{
    MIPStart.clear();

    env->output->outputDebug("        Deleted MIP starting points.");
}

void MIPSolverSCIP::writeProblemToFile(std::string filename)
{
    try
    {
        ErrorPrintingScope errorPrinting(env.get());

        auto fileInstance = createInstance();
        SCIP_CHECK(SCIPwriteOrigProblem(fileInstance->scip, filename.c_str(), nullptr, FALSE));
    }
    catch(std::exception& e)
    {
        env->output->outputError("        Error when saving model to file", e.what());
    }
}

void MIPSolverSCIP::fixVariable(int varIndex, double value) { updateVariableBound(varIndex, value, value); }

void MIPSolverSCIP::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
    currentLowerBounds.at(varIndex) = lowerBound;
    currentUpperBounds.at(varIndex) = upperBound;
    modelUpdated = true;
}

void MIPSolverSCIP::updateVariableLowerBound(int varIndex, double lowerBound)
{
    currentLowerBounds.at(varIndex) = lowerBound;
    modelUpdated = true;
}

void MIPSolverSCIP::updateVariableUpperBound(int varIndex, double upperBound)
{
    currentUpperBounds.at(varIndex) = upperBound;
    modelUpdated = true;
}

PairDouble MIPSolverSCIP::getCurrentVariableBounds(int varIndex)
{
    return (PairDouble(currentLowerBounds.at(varIndex), currentUpperBounds.at(varIndex)));
}

double MIPSolverSCIP::getUnboundedVariableBoundValue() { return (capabilities.unboundedVariableBound); }

double MIPSolverSCIP::getDualObjectiveValue()
{
    double objVal = (isMinimizationProblem ? SHOT_DBL_MIN : SHOT_DBL_MAX);

    // SCIP solves also a continuous problem with branch and bound, so its bound is valid when it is interrupted. With
    // removed objective coefficients, the bound is not one of the original problem.
    if(!instance || !objectiveCoefficientsToRestore.empty() || !isDualBoundAvailable(getSolutionStatus(), true))
        return (objVal);

    double bound = SCIPgetDualbound(instance->scip);

    if(!SCIPisInfinity(instance->scip, std::abs(bound)))
        objVal = bound;

    return (objVal);
}

void MIPSolverSCIP::checkParameters() { }

int MIPSolverSCIP::getNumberOfExploredNodes()
{
    if(!instance)
        return (0);

    return (static_cast<int>(std::min<SCIP_Longint>(SCIPgetNNodes(instance->scip), std::numeric_limits<int>::max())));
}

// SCIP solves a problem with one thread
int MIPSolverSCIP::getNumberOfThreads() { return (1); }

std::string MIPSolverSCIP::getSolverVersion()
{
    return (fmt::format("{}.{}.{}", SCIPmajorVersion(), SCIPminorVersion(), SCIPtechVersion()));
}

double MIPSolverSCIP::getConstraintUpperBound(int constraintIndex)
{
    double rhs = rows.at(constraintIndex).rhs;

    return (rhs >= 1e20 ? SHOT_DBL_MAX : rhs);
}

bool MIPSolverSCIP::setConstraintUpperBound(int constraintIndex, double upperBound)
{
    rows.at(constraintIndex).rhs = (upperBound >= 1e20) ? 1e20 : upperBound;
    modelUpdated = true;

    return (true);
}

} // namespace SHOT
