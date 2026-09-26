/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverUno.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "../Output.h"
#include "../Settings.h"
#include "../Utilities.h"

#include "Uno_C_API.h"

namespace SHOT
{

namespace
{

    /* Bounds at least this large in magnitude are handed to Uno as actual infinities. Uno considers a bound infinite
       only when it is an infinity, whereas SHOT uses SHOT_DBL_MIN/SHOT_DBL_MAX. The threshold matches the default of
       Ipopt's nlp_lower_bound_inf/nlp_upper_bound_inf options, so that a problem is presented to Uno with the same set
       of variables considered unbounded as it is to Ipopt. */
    const double UNO_BOUND_INFINITY = 1e19;

    /* Uno passes back the user data registered with uno_set_user_data, which is the NLPSolverUno instance that owns
    the model. */

    uno_int objectiveCallback(uno_int numberOfVariables, const double* x, double* value, void* userData)
    {
        auto* solver = static_cast<NLPSolverUno*>(userData);
        return (solver->evaluateObjective(numberOfVariables, x, value) ? 0 : 1);
    }

    uno_int objectiveGradientCallback(uno_int numberOfVariables, const double* x, double* gradient, void* userData)
    {
        auto* solver = static_cast<NLPSolverUno*>(userData);
        return (solver->evaluateObjectiveGradient(numberOfVariables, x, gradient) ? 0 : 1);
    }

    uno_int constraintsCallback(
        uno_int numberOfVariables, uno_int numberOfConstraints, const double* x, double* values, void* userData)
    {
        auto* solver = static_cast<NLPSolverUno*>(userData);
        return (solver->evaluateConstraints(numberOfVariables, numberOfConstraints, x, values) ? 0 : 1);
    }

    uno_int jacobianCallback(
        uno_int numberOfVariables, uno_int numberOfNonzeros, const double* x, double* values, void* userData)
    {
        auto* solver = static_cast<NLPSolverUno*>(userData);
        return (solver->evaluateJacobian(numberOfVariables, numberOfNonzeros, x, values) ? 0 : 1);
    }

    uno_int lagrangianHessianCallback(uno_int numberOfVariables, uno_int numberOfConstraints, uno_int numberOfNonzeros,
        const double* x, double objectiveMultiplier, const double* multipliers, double* values, void* userData)
    {
        auto* solver = static_cast<NLPSolverUno*>(userData);
        return (solver->evaluateLagrangianHessian(numberOfVariables, numberOfConstraints, numberOfNonzeros, x,
                    objectiveMultiplier, multipliers, values)
                ? 0
                : 1);
    }

    uno_int loggerCallback(const char* buffer, uno_int length, void* userData)
    {
        auto* solver = static_cast<NLPSolverUno*>(userData);
        solver->writeLoggerOutput(buffer, length);
        return (0);
    }

    /* Uno's logger stream is registered per process rather than per solver, while SHOT can have more than one
       NLPSolverUno alive at a time (the solution strategies create a fixed-integer NLP task both for the search and for
       the final polish). The most recent registration is the one in effect, so it is tracked here to keep a solver that
       is destroyed from tearing down another solver's logging. */
    NLPSolverUno* activeLoggerOwner = nullptr;

} // namespace

double NLPSolverUno::toUnoBound(double bound)
{
    if(bound >= UNO_BOUND_INFINITY)
        return (std::numeric_limits<double>::infinity());

    if(bound <= -UNO_BOUND_INFINITY)
        return (-std::numeric_limits<double>::infinity());

    return (bound);
}

NLPSolverUno::NLPSolverUno(EnvironmentPtr envPtr, ProblemPtr source) : INLPSolver(envPtr)
{
    sourceProblem = source;

    lowerBounds = sourceProblem->getVariableLowerBounds();
    upperBounds = sourceProblem->getVariableUpperBounds();

    variableSolution.resize(sourceProblem->properties.numberOfVariables);

    unoSolver = uno_create_solver();

    if(unoSolver == nullptr)
        throw std::runtime_error("Could not create Uno solver.");

    /* The logger stream is registered globally in Uno rather than per solver, so it is set here and released again
       in the destructor. It plays the same role as IpoptJournal does for Ipopt. */
    uno_set_logger_stream_callback(loggerCallback, this);
    activeLoggerOwner = this;

    createModel();
    setInitialSettings();

    uno_int major = 0;
    uno_int minor = 0;
    uno_int patch = 0;
    uno_get_version(&major, &minor, &patch);

    std::string preset;

    switch(static_cast<ES_UnoPreset>(env->settings->getSetting<int>("Subsolver.Uno.Preset")))
    {
    case(ES_UnoPreset::ipopt):
        preset = "Ipopt preset";
        break;

    case(ES_UnoPreset::filtersqp):
        preset = "filterSQP preset";
        break;

    default:
        preset = "automatically selected preset";
        break;
    }

    /* uno_get_method_description() only describes a strategy that has actually been built, so it is not meaningful
       until the first solve. This is called by TaskSelectPrimalCandidatesFromNLP before any solve, so the
       configured preset is reported instead. */
    solverDescription = fmt::format("Uno {}.{}.{} (with the {})", major, minor, patch, preset);
}

NLPSolverUno::~NLPSolverUno()
{
    /* Released unconditionally. Uno's logger stream is registered per process, so leaving it in place once any
       solver goes away risks the callback being invoked with a solver, or an environment, that no longer exists,
       which crashes during teardown. Releasing it early only means that a second, still-live solver stops writing
       to the log, which is preferable. */
    uno_reset_logger_stream();

    if(activeLoggerOwner == this)
        activeLoggerOwner = nullptr;

    if(unoModel != nullptr)
        uno_destroy_model(unoModel);

    if(unoSolver != nullptr)
        uno_destroy_solver(unoSolver);
}

void NLPSolverUno::createModel()
{
    int numberOfVariables = sourceProblem->properties.numberOfVariables;
    int numberOfConstraints = sourceProblem->properties.numberOfNumericConstraints;

    VectorDouble variableLowerBounds(numberOfVariables);
    VectorDouble variableUpperBounds(numberOfVariables);

    for(int i = 0; i < numberOfVariables; i++)
    {
        variableLowerBounds[i] = toUnoBound(lowerBounds[i]);
        variableUpperBounds[i] = toUnoBound(upperBounds[i]);
    }

    unoModel = uno_create_model(UNO_PROBLEM_NONLINEAR, numberOfVariables, variableLowerBounds.data(),
        variableUpperBounds.data(), UNO_ZERO_BASED_INDEXING);

    if(unoModel == nullptr)
        throw std::runtime_error("Could not create Uno model.");

    uno_set_model_name(unoModel, sourceProblem->name.c_str());
    uno_set_user_data(unoModel, this);

    uno_set_objective(unoModel, sourceProblem->objectiveFunction->properties.isMinimize ? UNO_MINIMIZE : UNO_MAXIMIZE,
        objectiveCallback, objectiveGradientCallback);

    /* SHOT accumulates the Lagrangian Hessian as obj_factor*d2f + sum(lambda_i*d2c_i), i.e. with positive
       multipliers, which is the opposite of Uno's default convention. Declaring the convention makes Uno flip the
       multipliers before handing them to the Hessian callback; without this the Hessian is silently wrong. */
    uno_set_lagrangian_sign_convention(unoModel, UNO_MULTIPLIER_POSITIVE);

    /* Unlike Ipopt, Uno takes the sparsity patterns once when the model is defined rather than through a structure
       pass of the evaluation callbacks, so both patterns and the maps from an entry to its position in the value
       array are built here and then reused by every evaluation. */

    VectorDouble constraintLowerBounds(numberOfConstraints);
    VectorDouble constraintUpperBounds(numberOfConstraints);

    for(int i = 0; i < numberOfConstraints; i++)
    {
        auto constraint = sourceProblem->numericConstraints[i];

        constraintLowerBounds[i] = toUnoBound(constraint->valueLHS);
        constraintUpperBounds[i] = toUnoBound(constraint->valueRHS);
    }

    std::vector<uno_int> jacobianRows;
    std::vector<uno_int> jacobianColumns;

    jacobianCounterPlacement.clear();

    for(auto& C : sourceProblem->numericConstraints)
    {
        for(auto& G : *C->getGradientSparsityPattern())
        {
            jacobianCounterPlacement.emplace(
                std::make_pair(C->getIndex(), G->getIndex()), static_cast<int>(jacobianRows.size()));

            jacobianRows.push_back(C->getIndex());
            jacobianColumns.push_back(G->getIndex());
        }
    }

    numberOfJacobianNonzeros = static_cast<int>(jacobianRows.size());

    uno_set_constraints(unoModel, numberOfConstraints, constraintsCallback, constraintLowerBounds.data(),
        constraintUpperBounds.data(), numberOfJacobianNonzeros, jacobianRows.data(), jacobianColumns.data(),
        jacobianCallback);

    std::vector<uno_int> hessianRows;
    std::vector<uno_int> hessianColumns;

    lagrangianHessianCounterPlacement.clear();

    for(auto& E : *sourceProblem->getLagrangianHessianSparsityPattern())
    {
        assert(E.first->getIndex() <= E.second->getIndex());

        lagrangianHessianCounterPlacement.emplace(
            std::make_pair(E.first->getIndex(), E.second->getIndex()), static_cast<int>(hessianRows.size()));

        hessianRows.push_back(E.first->getIndex());
        hessianColumns.push_back(E.second->getIndex());
    }

    numberOfHessianNonzeros = static_cast<int>(hessianRows.size());

    /* SHOT's Lagrangian Hessian sparsity pattern is stored with the row index not exceeding the column index, so
       it is the upper triangle that is provided. Uno transposes it into its own storage. */
    uno_set_lagrangian_hessian(unoModel, numberOfHessianNonzeros, UNO_UPPER_TRIANGLE, hessianRows.data(),
        hessianColumns.data(), lagrangianHessianCallback);

    // Uno copies the bounds and the sparsity patterns, so the local arrays do not need to be kept alive.
}

void NLPSolverUno::setInitialSettings()
{
    /* A preset must always be selected explicitly, and it is applied first since it assigns whole groups of
       options that the settings below then override individually.

       Uno resolves its own "auto" preset only inside uno_optimize, and by then it has already been overridden:
       uno_create_solver() preloads every default option, and uno_optimize() lets those defaults overwrite the
       preset it just picked. The defaults combine an SQP method with inertia correction, which needs a symmetric
       indefinite linear solver, so on a build without one every solve fails with an algorithmic error. Calling
       uno_set_solver_preset() is what puts the preset values among the options that win, but it rejects "auto"
       because that choice depends on the model. The size rule Uno uses for "auto" is therefore applied here. */
    auto preset = static_cast<ES_UnoPreset>(env->settings->getSetting<int>("Subsolver.Uno.Preset"));

    if(preset == ES_UnoPreset::UnoAuto)
    {
        int size = sourceProblem->properties.numberOfVariables + sourceProblem->properties.numberOfNumericConstraints;
        int nonzeros = numberOfJacobianNonzeros + numberOfHessianNonzeros;

        preset = (size >= 2000 || nonzeros >= 50000) ? ES_UnoPreset::ipopt : ES_UnoPreset::filtersqp;
    }

    uno_set_solver_preset(unoSolver, preset == ES_UnoPreset::ipopt ? "ipopt" : "filtersqp");

    uno_set_solver_double_option(
        unoSolver, "primal_tolerance", env->settings->getSetting<double>("Subsolver.Uno.ConstraintViolationTolerance"));

    uno_set_solver_double_option(
        unoSolver, "dual_tolerance", env->settings->getSetting<double>("Subsolver.Uno.RelativeConvergenceTolerance"));

    uno_set_solver_integer_option(
        unoSolver, "max_iterations", env->settings->getSetting<int>("Subsolver.Uno.MaxIterations"));

    uno_set_solver_double_option(
        unoSolver, "time_limit", env->settings->getSetting<double>("Primal.FixedInteger.TimeLimit"));

    switch(static_cast<ES_UnoHessianModel>(env->settings->getSetting<int>("Subsolver.Uno.HessianModel")))
    {
    case(ES_UnoHessianModel::LBFGS):
        uno_set_solver_string_option(unoSolver, "hessian_model", "LBFGS");
        break;

    case(ES_UnoHessianModel::LSR1):
        uno_set_solver_string_option(unoSolver, "hessian_model", "LSR1");
        break;

    case(ES_UnoHessianModel::exact):
    default:
        uno_set_solver_string_option(unoSolver, "hessian_model", "exact");
        break;
    }

    /* Uno warns once per fixed variable each time a model is solved, and SHOT fixes every discrete variable on
       every call, so the log has to be quietened rather than only redirected. */
    uno_set_solver_string_option(
        unoSolver, "logger", env->settings->getSetting<bool>("Output.Console.PrimalSolver.Show") ? "INFO" : "SILENT");

    /* Read last so that an option set explicitly by the user takes precedence over the ones derived from the SHOT
       settings above. */
    auto optionsFile = env->settings->getSetting<std::string>("Subsolver.Uno.OptionsFile");

    if(optionsFile != "")
    {
        if(!uno_load_solver_option_file(unoSolver, optionsFile.c_str()))
            env->output->outputWarning("        Could not read the Uno option file " + optionsFile + ".");
    }
}

void NLPSolverUno::pushBoundsToModel()
{
    int numberOfVariables = sourceProblem->properties.numberOfVariables;

    VectorDouble variableLowerBounds(numberOfVariables);
    VectorDouble variableUpperBounds(numberOfVariables);

    for(int i = 0; i < numberOfVariables; i++)
    {
        variableLowerBounds[i] = toUnoBound(lowerBounds[i]);
        variableUpperBounds[i] = toUnoBound(upperBounds[i]);
    }

    uno_set_variables_lower_bounds(unoModel, variableLowerBounds.data());
    uno_set_variables_upper_bounds(unoModel, variableUpperBounds.data());
}

void NLPSolverUno::pushStartingPointToModel()
{
    int numberOfVariables = sourceProblem->properties.numberOfVariables;

    VectorDouble startingPoint(numberOfVariables);
    std::vector<bool> isInitialized(numberOfVariables, false);

    for(size_t k = 0; k < startingPointVariableIndexes.size(); k++)
    {
        int variableIndex = startingPointVariableIndexes[k];
        double variableValue = startingPointVariableValues[k];

        double variableLB = sourceProblem->getVariableLowerBound(variableIndex);
        double variableUB = sourceProblem->getVariableUpperBound(variableIndex);

        if(variableUB == SHOT_DBL_MAX)
        {
            if(variableValue < variableLB)
            {
                env->output->outputDebug("         Initial value " + std::to_string(variableValue)
                    + " for variable with index " + std::to_string(variableIndex) + " is less than the lower bound "
                    + std::to_string(variableLB));
                continue;
            }
        }
        else if(variableLB == SHOT_DBL_MIN)
        {
            if(variableValue > variableUB)
            {
                env->output->outputDebug("         Initial value " + std::to_string(variableValue)
                    + " for variable with index " + std::to_string(variableIndex) + " is larger than the upper bound "
                    + std::to_string(variableUB));
                continue;
            }
        }
        else
        {
            if(variableValue < variableLB || variableValue > variableUB)
            {
                env->output->outputDebug("         Initial value " + std::to_string(variableValue)
                    + " for variable with index " + std::to_string(variableIndex) + " is not within variable bounds ["
                    + std::to_string(variableLB) + "," + std::to_string(variableUB));
                continue;
            }
        }

        if(variableValue < -divergingIterativesTolerance)
            variableValue = -0.99 * divergingIterativesTolerance;
        else if(variableValue > divergingIterativesTolerance)
            variableValue = 0.99 * divergingIterativesTolerance;

        startingPoint[variableIndex] = variableValue;
        isInitialized[variableIndex] = true;
    }

    double defaultInitValue = 1.7171;

    for(int k = 0; k < numberOfVariables; k++)
    {
        if(isInitialized[k])
            continue;

        double variableLB = sourceProblem->getVariableLowerBound(k);
        double variableUB = sourceProblem->getVariableUpperBound(k);

        if(variableUB == SHOT_DBL_MAX)
            startingPoint[k] = (defaultInitValue > variableLB) ? defaultInitValue : variableLB;
        else if(variableLB == SHOT_DBL_MIN)
            startingPoint[k] = (defaultInitValue < variableUB) ? defaultInitValue : variableUB;
        else if(variableLB <= defaultInitValue && defaultInitValue <= variableUB)
            startingPoint[k] = defaultInitValue;
        else if(variableLB > defaultInitValue)
            startingPoint[k] = variableLB;
        else
            startingPoint[k] = variableUB;
    }

    uno_set_initial_primal_iterate(unoModel, startingPoint.data());
}

bool NLPSolverUno::evaluateObjective(int numberOfVariables, const double* x, double* value)
{
    VectorDouble vectorPoint(x, x + numberOfVariables);

    *value = sourceProblem->objectiveFunction->calculateValue(vectorPoint);

    return (true);
}

bool NLPSolverUno::evaluateObjectiveGradient(int numberOfVariables, const double* x, double* gradient)
{
    VectorDouble vectorPoint(x, x + numberOfVariables);

    std::memset(gradient, 0, numberOfVariables * sizeof(double));

    for(auto& G : sourceProblem->objectiveFunction->calculateGradient(vectorPoint, false))
        gradient[G.first->getIndex()] = G.second;

    return (true);
}

bool NLPSolverUno::evaluateConstraints(int numberOfVariables, int numberOfConstraints, const double* x, double* values)
{
    VectorDouble vectorPoint(x, x + numberOfVariables);

    for(int i = 0; i < numberOfConstraints; i++)
        values[i] = sourceProblem->numericConstraints[i]->calculateFunctionValue(vectorPoint);

    return (true);
}

bool NLPSolverUno::evaluateJacobian(int numberOfVariables, int numberOfNonzeros, const double* x, double* values)
{
    VectorDouble vectorPoint(x, x + numberOfVariables);

    std::memset(values, 0, numberOfNonzeros * sizeof(double));

    for(auto& C : sourceProblem->numericConstraints)
    {
        for(auto& G : C->calculateGradient(vectorPoint, false))
        {
            int location = jacobianCounterPlacement[std::make_pair(C->getIndex(), G.first->getIndex())];

            assert(location < numberOfNonzeros);
            assert(location >= 0);

            values[location] += G.second;
        }
    }

    return (true);
}

bool NLPSolverUno::evaluateLagrangianHessian(int numberOfVariables, [[maybe_unused]] int numberOfConstraints,
    int numberOfNonzeros, const double* x, double objectiveMultiplier, const double* multipliers, double* values)
{
    VectorDouble vectorPoint(x, x + numberOfVariables);

    std::memset(values, 0, numberOfNonzeros * sizeof(double));

    if(objectiveMultiplier != 0.0)
    {
        for(auto& E : sourceProblem->objectiveFunction->calculateHessian(vectorPoint, false))
        {
            int location = lagrangianHessianCounterPlacement[std::make_pair(
                E.first.first->getIndex(), E.first.second->getIndex())];

            assert(location < numberOfNonzeros);
            assert(location >= 0);

            values[location] = objectiveMultiplier * E.second;
        }
    }

    for(auto& C : sourceProblem->numericConstraints)
    {
        if(C->properties.classification == E_ConstraintClassification::Linear)
            continue;

        if(multipliers[C->getIndex()] == 0.0)
            continue;

        for(auto& E : C->calculateHessian(vectorPoint, false))
        {
            int location = lagrangianHessianCounterPlacement[std::make_pair(
                E.first.first->getIndex(), E.first.second->getIndex())];

            assert(location < numberOfNonzeros);
            assert(location >= 0);

            values[location] += multipliers[C->getIndex()] * E.second;
        }
    }

    return (true);
}

void NLPSolverUno::writeLoggerOutput(const char* buffer, int length)
{
    /* Releasing Uno's logger stream deletes the stream object, which flushes whatever it still holds and therefore
       calls back in here. That happens from the destructor, by which point the environment this solver was created
       with may already be gone, so there is nothing left to write to. */
    if(!env || !env->output)
    {
        loggerBuffer.clear();
        return;
    }

    /* Uno writes its output in pieces that do not line up with line boundaries, so the text is buffered here and
       only forwarded once a newline has been seen. Otherwise a message that Uno emits without a trailing newline,
       such as the reason it stopped, is split across calls and lost. */
    loggerBuffer.append(buffer, length);

    size_t lineEnd;

    while((lineEnd = loggerBuffer.find('\n')) != std::string::npos)
    {
        std::string line = loggerBuffer.substr(0, lineEnd);
        loggerBuffer.erase(0, lineEnd + 1);

        if(line != "")
            env->output->outputInfo("      | " + line);
    }
}

void NLPSolverUno::flushLoggerOutput()
{
    if(loggerBuffer != "")
        env->output->outputInfo("      | " + loggerBuffer);

    loggerBuffer.clear();
}

E_NLPSolutionStatus NLPSolverUno::solveProblemInstance()
{
    env->output->outputDebug("        Starting solution of Uno problem.");

    E_NLPSolutionStatus status;

    variableSolution.clear();
    variableSolution.resize(sourceProblem->properties.numberOfVariables);
    objectiveValue = 0.0;

    try
    {
        pushBoundsToModel();
        pushStartingPointToModel();

        uno_optimize(unoSolver, unoModel);

        // Uno's last message may not end in a newline, so make sure it is not left sitting in the buffer
        flushLoggerOutput();

        uno_int optimizationStatus = uno_get_optimization_status(unoSolver);

        if(optimizationStatus == UNO_ITERATION_LIMIT)
        {
            status = E_NLPSolutionStatus::IterationLimit;
            env->output->outputDebug("        No solution found to problem with Uno: Iteration limit exceeded.");
        }
        else if(optimizationStatus == UNO_TIME_LIMIT)
        {
            status = E_NLPSolutionStatus::TimeLimit;
            env->output->outputDebug("        No solution found to problem with Uno: Time limit exceeded.");
        }
        else if(optimizationStatus != UNO_SUCCESS)
        {
            status = E_NLPSolutionStatus::Error;

            std::string reason;

            if(optimizationStatus == UNO_EVALUATION_ERROR)
                reason = "a function evaluation failed";
            else if(optimizationStatus == UNO_ALGORITHMIC_ERROR)
                reason = "of an algorithmic error";
            else if(optimizationStatus == UNO_USER_TERMINATION)
                reason = "the solve was terminated";
            else
                reason = fmt::format("of an unknown optimization status {}", optimizationStatus);

            env->output->outputDebug(
                fmt::format("        Error when solving NLP problem with Uno: {} (solution status {}).", reason,
                    uno_get_solution_status(unoSolver)));
        }
        else
        {
            /* Uno reports the reason it stopped separately from the quality of the point it stopped at, so the
               solution status decides what the point can be used for. */
            uno_int solutionStatus = uno_get_solution_status(unoSolver);

            if(solutionStatus == UNO_FEASIBLE_KKT_POINT)
            {
                status = E_NLPSolutionStatus::Optimal;
                env->output->outputDebug("        Local solution found with Uno.");
            }
            else if(solutionStatus == UNO_FEASIBLE_FJ_POINT || solutionStatus == UNO_FEASIBLE_SMALL_STEP)
            {
                status = E_NLPSolutionStatus::Feasible;
                env->output->outputDebug("        Feasible solution found with Uno.");
            }
            else if(solutionStatus == UNO_INFEASIBLE_STATIONARY_POINT || solutionStatus == UNO_INFEASIBLE_SMALL_STEP)
            {
                status = E_NLPSolutionStatus::Infeasible;
                env->output->outputDebug("        No solution found to problem with Uno: Locally infeasible.");
            }
            else if(solutionStatus == UNO_DIVERGING_ITERATE || solutionStatus == UNO_UNBOUNDED_OBJECTIVE)
            {
                status = E_NLPSolutionStatus::Unbounded;
                env->output->outputDebug("        No solution found to problem with Uno: Diverging iterates.");
            }
            else
            {
                status = E_NLPSolutionStatus::Error;
                env->output->outputDebug("        No solution found to problem with Uno.");
            }
        }

        /* The result is held by the solver rather than the model, so it is copied out before anything else can be
           solved. Uno converts the objective back into the sense of the model, so no sign correction is needed. */
        if(status != E_NLPSolutionStatus::Error)
        {
            uno_get_primal_solution(unoSolver, variableSolution.data());
            objectiveValue = uno_get_solution_objective(unoSolver);
        }
    }
    catch(std::exception& e)
    {
        /* Uno throws when a requested subproblem solver was not compiled into it, and uno_optimize does not catch
           it, so the exception surfaces here. */
        env->output->outputError("        Error when solving problem with Uno!", e.what());
        status = E_NLPSolutionStatus::Error;
    }
    catch(...)
    {
        env->output->outputError("        Unspecified error when solving problem with Uno!");
        status = E_NLPSolutionStatus::Error;
    }

    env->output->outputDebug("        Finished solution of Uno problem.");

    return (status);
}

void NLPSolverUno::setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues)
{
    startingPointVariableIndexes = variableIndexes;
    startingPointVariableValues = variableValues;

    int startingPointSize = startingPointVariableIndexes.size();

    if(startingPointSize == 0)
        return;

    env->output->outputDebug("        Adding starting points to Uno.");

    for(int k = 0; k < startingPointSize; k++)
    {
        int currVarIndex = startingPointVariableIndexes.at(k);
        auto currPt = startingPointVariableValues.at(k);

        auto currLB = sourceProblem->getVariableLowerBound(currVarIndex);
        auto currUB = sourceProblem->getVariableUpperBound(currVarIndex);

        if(currPt > currUB)
        {
            env->output->outputDebug("         Starting point value for variable " + std::to_string(currVarIndex)
                + " is larger than ub: " + Utilities::toString(currPt) + " > " + Utilities::toString(currUB)
                + "; resetting to ub.");

            if(currUB == 1 && currPt > 1 && currPt < 1.00001)
                currPt = 1;
            else
                currPt = currUB;
        }

        if(currPt < currLB)
        {
            env->output->outputDebug("         Starting point value for variable " + std::to_string(currVarIndex)
                + " is smaller than lb: " + Utilities::toString(currPt) + " < " + Utilities::toString(currLB)
                + "; resetting to lb.");

            if(currLB == 0 && currPt < 0 && currPt > -0.00001)
                currPt = 0;
            else
                currPt = currLB;
        }

        startingPointVariableValues.at(k) = currPt;
    }

    env->output->outputDebug("        All starting points set.");
}

void NLPSolverUno::clearStartingPoint()
{
    startingPointVariableIndexes.clear();
    startingPointVariableValues.clear();
    setInitialSettings();
}

void NLPSolverUno::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    fixedVariableIndexes = variableIndexes;
    fixedVariableValues = variableValues;

    int size = fixedVariableIndexes.size();

    if(size == 0)
        return;

    if(lowerBoundsBeforeFix.size() > 0 || upperBoundsBeforeFix.size() > 0)
    {
        env->output->outputDebug("        Old variable fixes remain for Uno solver, resetting!");
        lowerBoundsBeforeFix.clear();
        upperBoundsBeforeFix.clear();
    }

    env->output->outputDebug("        Defining fixed variables in Uno.");

    for(int k = 0; k < size; k++)
    {
        int currVarIndex = fixedVariableIndexes.at(k);
        double currPt = fixedVariableValues.at(k);

        double currLB = sourceProblem->getVariableLowerBound(currVarIndex);
        double currUB = sourceProblem->getVariableUpperBound(currVarIndex);

        lowerBoundsBeforeFix.push_back(currLB);
        upperBoundsBeforeFix.push_back(currUB);

        currPt = std::round(currPt);

        // Fix for negative zero
        if(currPt >= (0.0 - std::numeric_limits<double>::epsilon())
            && currPt <= 0.0 + std::numeric_limits<double>::epsilon())
        {
            currPt = 0.0;
        }

        if(currPt > currUB)
        {
            env->output->outputDebug("         Fixed value for variable " + std::to_string(currVarIndex)
                + " is larger than ub: " + Utilities::toString(currPt) + " > " + std::to_string(currUB));

            if(currUB == 1 && currPt > 1 && currPt < 1.00001)
                currPt = 1;
            else
                continue;
        }
        else if(currPt < currLB)
        {
            env->output->outputDebug("         Fixed value for variable " + std::to_string(currVarIndex)
                + " is smaller than lb: " + Utilities::toString(currPt) + " < " + std::to_string(currLB));

            if(currLB == 0 && currPt < 0 && currPt > -0.00001)
                currPt = 0;
            else
                continue;
        }

        if(currPt >= currLB && currPt <= currUB)
        {
            lowerBounds.at(currVarIndex) = currPt;
            upperBounds.at(currVarIndex) = currPt;
        }
        else
        {
            env->output->outputDebug("          Cannot fix variable value for variable with index "
                + std::to_string(currVarIndex) + ": not within bounds (" + Utilities::toString(currLB) + " < "
                + Utilities::toString(currPt) + " < " + Utilities::toString(currUB));
        }
    }

    env->output->outputDebug("        All fixed variables defined.");
}

void NLPSolverUno::unfixVariables()
{
    env->output->outputDebug("        Starting reset of fixed variables in Uno.");

    for(size_t k = 0; k < fixedVariableIndexes.size(); k++)
    {
        int currVarIndex = fixedVariableIndexes.at(k);

        lowerBounds[currVarIndex] = lowerBoundsBeforeFix.at(k);
        upperBounds[currVarIndex] = upperBoundsBeforeFix.at(k);
    }

    fixedVariableIndexes.clear();
    fixedVariableValues.clear();
    lowerBoundsBeforeFix.clear();
    upperBoundsBeforeFix.clear();

    setInitialSettings();

    env->output->outputDebug("        Reset of fixed variables in Uno completed.");
}

VectorDouble NLPSolverUno::getVariableLowerBounds() { return (lowerBounds); }

VectorDouble NLPSolverUno::getVariableUpperBounds() { return (upperBounds); }

void NLPSolverUno::updateVariableLowerBound(int variableIndex, double bound) { lowerBounds[variableIndex] = bound; }

void NLPSolverUno::updateVariableUpperBound(int variableIndex, double bound) { upperBounds[variableIndex] = bound; }

VectorDouble NLPSolverUno::getSolution() { return (variableSolution); }

double NLPSolverUno::getSolution(int i) { return (variableSolution[i]); }

double NLPSolverUno::getObjectiveValue() { return (objectiveValue); }

void NLPSolverUno::saveOptionsToFile([[maybe_unused]] std::string fileName) { }

void NLPSolverUno::saveProblemToFile([[maybe_unused]] std::string fileName) { }

std::string NLPSolverUno::getSolverDescription() { return (solverDescription); }

} // namespace SHOT
