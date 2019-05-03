/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverIpoptBase.h"

#include "../Output.h"
#include "../Settings.h"
#include "../Utilities.h"

namespace SHOT
{

using namespace Ipopt;

IpoptProblem::IpoptProblem(EnvironmentPtr envPtr, NLPSolverIpoptBase* solver, ProblemPtr problem)
    : env(envPtr), ipoptSolver(solver), sourceProblem(problem)
{
}

bool IpoptProblem::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n = sourceProblem->properties.numberOfVariables;
    m = sourceProblem->properties.numberOfNumericConstraints;

    nnz_jac_g = 0;

    for(auto& E : *sourceProblem->getConstraintsJacobianSparsityPattern())
    {
        nnz_jac_g += E.second.size();
    }

    nnz_h_lag = sourceProblem->getLagrangianHessianSparsityPattern()->size();

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

    return (true);
}

bool IpoptProblem::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u)
{
    for(int i = 0; i < n; i++)
    {
        x_l[i] = ipoptSolver->lowerBounds[i];
        x_u[i] = ipoptSolver->upperBounds[i];
    }

    // Ipopt interprets any number greater than nlp_upper_bound_inf as
    // infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
    // is 1e19 and can be changed through ipopt options.
    // e.g. g_u[0] = 2e19;

    for(int i = 0; i < m; i++)
    {
        auto constraint = std::dynamic_pointer_cast<NumericConstraint>(sourceProblem->getConstraint(i));

        g_l[i] = constraint->valueLHS;
        g_u[i] = constraint->valueRHS;
    }

    return (true);
}

// returns the initial point for the problem
bool IpoptProblem::get_starting_point(
    Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U, Index m, bool init_lambda, Number* lambda)
{
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    std::vector<bool> isInitialized(n, false);

    for(int k = 0; k < ipoptSolver->startingPointVariableIndexes.size(); k++)
    {
        int variableIndex = ipoptSolver->startingPointVariableIndexes[k];

        double variableValue = ipoptSolver->startingPointVariableValues[k];

        double variableLB = sourceProblem->getVariableLowerBound(variableIndex);
        double variableUB = sourceProblem->getVariableUpperBound(variableIndex);

        if(variableUB == SHOT_DBL_MAX)
        {
            if(variableValue < variableLB)
            {
                env->output->outputCritical("Initial value " + std::to_string(variableValue)
                    + " for variable with index " + std::to_string(variableIndex) + " is less than the lower bound "
                    + std::to_string(variableLB));
                continue;
            }
        }
        else if(variableLB == SHOT_DBL_MIN)
        {
            if(variableValue > variableUB)
            {
                env->output->outputCritical("Initial value " + std::to_string(variableValue)
                    + " for variable with index " + std::to_string(variableIndex) + " is larger than the upper bound "
                    + std::to_string(variableUB));
                continue;
            }
        }
        else
        {
            if(variableValue < variableLB || variableValue > variableUB)
            {
                env->output->outputCritical("Initial value " + std::to_string(variableValue)
                    + " for variable with index " + std::to_string(variableIndex) + " is not within variable bounds ["
                    + std::to_string(variableLB) + "," + std::to_string(variableUB));
                continue;
            }
        }

        x[variableIndex] = variableValue;
        isInitialized[variableIndex] = true;
    }

    double defaultInitValue = 1.7171; // TODO: why?

    for(int k = 0; k < n; k++)
    {
        if(isInitialized[k])
            continue;

        double variableLB = sourceProblem->getVariableLowerBound(k);
        double variableUB = sourceProblem->getVariableUpperBound(k);

        if(variableUB == SHOT_DBL_MAX)
        {
            if(defaultInitValue > variableLB)

                x[k] = variableLB;
            else
                x[k] = defaultInitValue;
        }
        else if(variableLB == SHOT_DBL_MIN)
        {
            if(defaultInitValue > variableUB)
                x[k] = variableUB;
            else
                x[k] = defaultInitValue;
        }
        else if(variableLB <= defaultInitValue && defaultInitValue <= variableUB)
            x[k] = variableUB;
        else if(variableLB > defaultInitValue)
            x[k] = variableLB;
        else
            x[k] = variableUB;
    }

    return (true);
}

// Returns the value of the objective function
bool IpoptProblem::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    VectorDouble vectorPoint(n);

    for(int i = 0; i < n; i++)
        vectorPoint[i] = x[i];

    obj_value = sourceProblem->objectiveFunction->calculateValue(vectorPoint);

    return (true);
}

// Returns the gradient of the objective function
bool IpoptProblem::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
    VectorDouble vectorPoint(n);

    for(int i = 0; i < n; i++)
        vectorPoint[i] = x[i];

    for(int i = 0; i < n; i++)
        grad_f[i] = 0.0;

    for(auto& G : sourceProblem->objectiveFunction->calculateGradient(vectorPoint, false))
        grad_f[G.first->index] = G.second;

    return (true);
}

// Return the value of the constraints
bool IpoptProblem::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
    VectorDouble vectorPoint(n);

    for(int i = 0; i < n; i++)
        vectorPoint[i] = x[i];

    for(int i = 0; i < m; i++)
        g[i] = 0.0;

    for(int i = 0; i < m; i++)
        g[i] = sourceProblem->numericConstraints[i]->calculateFunctionValue(vectorPoint);

    return (true);
}

// Return the structure or values of the jacobian
bool IpoptProblem::eval_jac_g(
    Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index* jCol, Number* values)
{
    // The structure
    if(values == nullptr)
    {
        int counter = 0;

        jacobianCounterPlacement.clear();

        for(auto& C : sourceProblem->numericConstraints)
        {
            auto jacobian = C->getGradientSparsityPattern();

            for(auto& G : *jacobian)
            {
                iRow[counter] = C->index;
                jCol[counter] = G->index;

                jacobianCounterPlacement.emplace(std::make_pair(C->index, G->index), counter);
                counter++;
            }
        }

        return (true);
    }

    // The values

    VectorDouble vectorPoint(n);

    for(int i = 0; i < n; i++)
        vectorPoint[i] = x[i];

    for(int i = 0; i < nele_jac; i++)
        values[i] = 0.0;

    for(auto& C : sourceProblem->numericConstraints)
    {
        auto jacobian = C->calculateGradient(vectorPoint, false);

        for(auto& G : jacobian)
        {
            int location = jacobianCounterPlacement[std::make_pair(C->index, G.first->index)];

            values[location] += G.second;
        }
    }

    return (true);
}

// Return the structure or values of the Hessian of the Langragian
bool IpoptProblem::eval_h(Index n, const Number* x, bool new_x, Number obj_factor, Index m, const Number* lambda,
    bool new_lambda, Index nele_hess, Index* iRow, Index* jCol, Number* values)
{
    // The structure
    if(values == nullptr)
    {
        int counter = 0;
        lagrangianHessianCounterPlacement.clear();

        for(auto& E : *sourceProblem->getLagrangianHessianSparsityPattern())
        {
            assert(E.first->index <= E.second->index);

            iRow[counter] = E.first->index;
            jCol[counter] = E.second->index;

            lagrangianHessianCounterPlacement.emplace(std::make_pair(E.first->index, E.second->index), counter);

            counter++;
        }

        return (true);
    }

    // The values

    int counter = 0;

    VectorDouble vectorPoint(n);

    for(int i = 0; i < n; i++)
        vectorPoint[i] = x[i];

    for(int i = 0; i < nele_hess; i++)
        values[i] = 0.0;

    if(obj_factor != 0.0)
    {
        for(auto& E : sourceProblem->objectiveFunction->calculateHessian(vectorPoint, false))
        {
            int location
                = lagrangianHessianCounterPlacement[std::make_pair(E.first.first->index, E.first.second->index)];

            values[location] = obj_factor * E.second;
        }
    }

    for(auto& C : sourceProblem->numericConstraints)
    {
        if(C->properties.classification == E_ConstraintClassification::Linear)
            continue;

        if(lambda[C->index] == 0.0)
            continue;

        for(auto& E : C->calculateHessian(vectorPoint, false))
        {
            int location
                = lagrangianHessianCounterPlacement[std::make_pair(E.first.first->index, E.first.second->index)];

            values[location] += lambda[C->index] * E.second;
        }
    }

    return (true);
}

bool IpoptProblem::get_scaling_parameters(Number& obj_scaling, bool& use_x_scaling, Index n, Number* x_scaling,
    bool& use_g_scaling, Index m, Number* g_scaling)
{
    obj_scaling = sourceProblem->objectiveFunction->properties.isMinimize ? 1.0 : -1.0;

    use_x_scaling = false;
    use_g_scaling = false;

    return (true);
}

void IpoptProblem::finalize_solution(SolverReturn status, Index n, const Number* x, const Number* z_L,
    const Number* z_U, Index m, const Number* g, const Number* lambda, Number obj_value, const IpoptData* ip_data,
    IpoptCalculatedQuantities* ip_cq)
{
    int numberOfVariables = sourceProblem->properties.numberOfVariables;

    switch(status)
    {
    case SUCCESS:
        solutionDescription
            = "Algorithm terminated normally at a locally optimal point satisfying the convergence tolerances.";

        solutionStatus = E_NLPSolutionStatus::Optimal;
        variableSolution = VectorDouble(numberOfVariables);

        for(int i = 0; i < numberOfVariables; i++)
            variableSolution[i] = x[i];

        objectiveValue = obj_value;
        hasSolution = true;

        break;

    case MAXITER_EXCEEDED:
        solutionDescription = "Maximum number of iterations exceeded.";

        solutionStatus = E_NLPSolutionStatus::IterationLimit;

        if(x != nullptr)
        {
            hasSolution = true;
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case STOP_AT_TINY_STEP:
        solutionDescription = "Algorithm proceeds with very little progress.";

        solutionStatus = E_NLPSolutionStatus::IterationLimit;

        if(x != nullptr)
        {
            hasSolution = true;
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case STOP_AT_ACCEPTABLE_POINT:
        solutionDescription = "Algorithm stopped at a point that was converged, not to desired tolerances, but to "
                              "acceptable tolerances.";

        solutionStatus = E_NLPSolutionStatus::Feasible;

        if(x != nullptr)
        {
            hasSolution = true;
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case LOCAL_INFEASIBILITY:
        solutionDescription = "Algorithm converged to a point of local infeasibility. Problem may be infeasible.";

        solutionStatus = E_NLPSolutionStatus::Infeasible;

        break;

    case USER_REQUESTED_STOP:
        solutionDescription = "The user requested a premature termination of the optimization.";

        solutionStatus = E_NLPSolutionStatus::Error;

        if(x != nullptr)
        {
            hasSolution = true;
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case DIVERGING_ITERATES:
        solutionDescription = "It seems that the iterates diverge.";

        solutionStatus = E_NLPSolutionStatus::Unbounded;

        if(x != nullptr)
        {
            hasSolution = true;
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case RESTORATION_FAILURE:
        solutionDescription = "Restoration phase failed, algorithm doesn't know how to proceed.";

        solutionStatus = E_NLPSolutionStatus::Error;

        if(x != nullptr)
        {
            hasSolution = true;
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case ERROR_IN_STEP_COMPUTATION:
        solutionDescription = "An unrecoverable error occurred while Ipopt tried to compute the search direction.";

        solutionStatus = E_NLPSolutionStatus::Error;

        if(x != nullptr)
        {
            hasSolution = true;
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case INVALID_NUMBER_DETECTED:
        solutionDescription = "Algorithm received an invalid number (such as NaN or Inf) from the NLP.";

        solutionStatus = E_NLPSolutionStatus::Error;

        if(x != nullptr)
        {
            hasSolution = true;
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case INTERNAL_ERROR:
        solutionDescription = "An unknown internal error occurred.";

        solutionStatus = E_NLPSolutionStatus::Error;

        if(x != nullptr)
        {
            hasSolution = true;
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    default:
        solutionDescription = "Unknown solution status.";

        solutionStatus = E_NLPSolutionStatus::Error;
    }

    env->output->outputDebug("Ipopt terminated with status: " + solutionDescription);
}

E_NLPSolutionStatus NLPSolverIpoptBase::solveProblemInstance()
{
    env->output->outputDebug(" Starting solution of Ipopt problem.");

    E_NLPSolutionStatus status;

    try
    {
        updateSettings();

        ipoptProblem = std::make_shared<IpoptProblem>(env, this, sourceProblem);

        ipoptApplication = std::make_unique<Ipopt::IpoptApplication>();

        setInitialSettings();

        Ipopt::ApplicationReturnStatus ipoptStatus = ipoptApplication->Initialize();

        if(ipoptStatus != Ipopt::Solve_Succeeded)
        {
            env->output->outputError(" Error when initializing Ipopt.");
            return (E_NLPSolutionStatus::Error);
        }

        ipoptStatus = ipoptApplication->OptimizeTNLP(ipoptProblem.get());

        switch(ipoptStatus)
        {
        case Ipopt::ApplicationReturnStatus::Solve_Succeeded:
            status = E_NLPSolutionStatus::Optimal;
            env->output->outputDebug(" Global solution found with Ipopt.");
            break;

        case Ipopt::ApplicationReturnStatus::Feasible_Point_Found:
        case Ipopt::ApplicationReturnStatus::Solved_To_Acceptable_Level:
            status = E_NLPSolutionStatus::Feasible;
            env->output->outputDebug(" Feasible solution found with Ipopt.");
            break;

        case Ipopt::ApplicationReturnStatus::Infeasible_Problem_Detected:
            status = E_NLPSolutionStatus::Infeasible;
            env->output->outputDebug(" No solution found to problem with Ipopt: Infeasible problem detected.");
            break;

        case Ipopt::ApplicationReturnStatus::Maximum_Iterations_Exceeded:
            status = E_NLPSolutionStatus::IterationLimit;
            env->output->outputDebug(" No solution found to problem with Ipopt: Iteration limit exceeded.");
            break;

        case Ipopt::ApplicationReturnStatus::Maximum_CpuTime_Exceeded:
            status = E_NLPSolutionStatus::TimeLimit;
            env->output->outputDebug(" No solution found to problem with Ipopt: Time limit exceeded.");
            break;

        default:
            status = E_NLPSolutionStatus::Error;
            env->output->outputError(" Error when solving NLP problem with Ipopt.");
            break;
        }
    }
    catch(std::exception& e)
    {
        env->output->outputError(" Error when solving relaxed problem with Ipopt!", e.what());
        status = E_NLPSolutionStatus::Error;
    }
    catch(...)
    {
        env->output->outputError(" Unspecified error when solving relaxed problem with Ipopt!");
        status = E_NLPSolutionStatus::Error;
    }

    env->output->outputDebug(" Finished solution of Ipopt problem.");

    return (status);
}

double NLPSolverIpoptBase::getSolution(int i) { return (ipoptProblem->variableSolution[i]); }

double NLPSolverIpoptBase::getObjectiveValue() { return (ipoptProblem->objectiveValue); }

void NLPSolverIpoptBase::setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues)
{
    startingPointVariableIndexes = variableIndexes;
    startingPointVariableValues = variableValues;

    int startingPointSize = startingPointVariableIndexes.size();

    if(startingPointSize == 0)
        return;

    env->output->outputDebug(" Adding starting points to Ipopt.");

    for(int k = 0; k < startingPointSize; k++)
    {
        int currVarIndex = startingPointVariableIndexes.at(k);
        auto currPt = startingPointVariableValues.at(k);

        auto currLB = sourceProblem->getVariableLowerBound(currVarIndex);
        auto currUB = sourceProblem->getVariableUpperBound(currVarIndex);

        if(currPt > currUB)
        {
            env->output->outputDebug("  Starting point value for variable " + std::to_string(currVarIndex)
                + " is larger than ub: " + Utilities::toString(currPt) + " > " + Utilities::toString(currUB)
                + "; resetting to ub.");
            if(currUB == 1 && currPt > 1 && currPt < 1.00001)
            {
                currPt = 1;
            }
            else
            {
                currPt = currUB;
            }
        }

        if(currPt < currLB)
        {
            env->output->outputDebug("  Starting point value for variable " + std::to_string(currVarIndex)
                + " is smaller than lb: " + Utilities::toString(currPt) + " < " + Utilities::toString(currLB)
                + "; resetting to lb.");

            if(currLB == 0 && currPt < 0 && currPt > -0.00001)
            {
                currPt = 0;
            }
            else
            {
                currPt = currLB;
            }
        }

        startingPointVariableValues.at(k) = currPt;

        env->output->outputTrace("  Starting point value for " + std::to_string(currVarIndex) + " set: "
            + Utilities::toString(currLB) + " < " + Utilities::toString(currPt) + " < " + Utilities::toString(currUB));
    }

    env->output->outputDebug(" All starting points set.");
}

void NLPSolverIpoptBase::clearStartingPoint()
{
    startingPointVariableIndexes.clear();
    startingPointVariableValues.clear();
    setInitialSettings();
    setSolverSpecificInitialSettings();
}

VectorDouble NLPSolverIpoptBase::getVariableLowerBounds() { return (lowerBounds); }

VectorDouble NLPSolverIpoptBase::getVariableUpperBounds() { return (upperBounds); }

VectorDouble NLPSolverIpoptBase::getSolution() { return (ipoptProblem->variableSolution); }

void NLPSolverIpoptBase::setInitialSettings()
{
    std::string subsolver = "";

    // Sets the linear solver used
    switch(static_cast<ES_IpoptSolver>(env->settings->getSetting<int>("Ipopt.LinearSolver", "Subsolver")))
    {
    case(ES_IpoptSolver::ma27):
        subsolver = "ma27";
        break;

    case(ES_IpoptSolver::ma57):
        subsolver = "ma57";
        break;

    case(ES_IpoptSolver::ma86):
        subsolver = "ma86";
        break;

    case(ES_IpoptSolver::ma97):
        subsolver = "ma97";
        break;

    case(ES_IpoptSolver::mumps):
        subsolver = "mumps";
        break;

    default:
        subsolver = "mumps";
    }

    ipoptApplication->Options()->SetStringValue("fixed_variable_treatment", "make_parameter");
    ipoptApplication->Options()->SetStringValue("linear_solver", subsolver);

    switch(static_cast<E_LogLevel>(env->settings->getSetting<int>("Console.LogLevel", "Output")))
    {
    case E_LogLevel::Off:
    case E_LogLevel::Critical:
    case E_LogLevel::Error:
    case E_LogLevel::Warning:
    case E_LogLevel::Info:
        ipoptApplication->Options()->SetIntegerValue("print_level", 0);
        break;
    case E_LogLevel::Debug:
        ipoptApplication->Options()->SetIntegerValue("print_level", 8);
        break;
    case E_LogLevel::Trace:
        ipoptApplication->Options()->SetIntegerValue("print_level", 10);
        break;
    default:
        break;
    }

    // Suppress copyright message
    if(env->settings->getSetting<int>("Console.LogLevel", "Output") > (int)E_LogLevel::Debug)
    {
        ipoptApplication->Options()->SetStringValue("sb", "yes");
    }

    // ipoptApplication->Options()->SetStringValue("derivative_test", "second-order");
    // ipoptApplication->Options()->SetStringValue("derivative_test_print_all", "yes");

    setSolverSpecificInitialSettings();
}

void NLPSolverIpoptBase::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    fixedVariableIndexes = variableIndexes;
    fixedVariableValues = variableValues;

    int size = fixedVariableIndexes.size();

    if(size == 0)
        return;

    if(lowerBoundsBeforeFix.size() > 0 || upperBoundsBeforeFix.size() > 0)
    {
        env->output->outputDebug(" Old variable fixes remain for Ipopt solver, resetting!");
        lowerBoundsBeforeFix.clear();
        upperBoundsBeforeFix.clear();
    }

    env->output->outputDebug(" Defining fixed variables in Ipopt.");

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
            env->output->outputWarning("  Fixed value for variable " + std::to_string(currVarIndex)
                + " is larger than ub: " + Utilities::toString(currPt) + " > " + std::to_string(currUB));
            if(currUB == 1 && currPt > 1 && currPt < 1.00001)
            {
                currPt = 1;
            }
            else
            {
                continue;
            }
        }
        else if(currPt < currLB)
        {
            env->output->outputWarning("  Fixed value for variable " + std::to_string(currVarIndex)
                + " is smaller than lb: " + Utilities::toString(currPt) + " < " + std::to_string(currLB));

            if(currLB == 0 && currPt < 0 && currPt > -0.00001)
            {
                currPt = 0;
            }
            else
            {
                continue;
            }
        }

        env->output->outputTrace("  Setting fixed value for variable " + std::to_string(currVarIndex) + ": "
            + Utilities::toString(currLB) + " <= " + Utilities::toString(currPt)
            + " <= " + Utilities::toString(currUB));

        if(currPt >= sourceProblem->getVariableLowerBound(currVarIndex)
            && currPt <= sourceProblem->getVariableUpperBound(currVarIndex))
        {
            lowerBounds.at(currVarIndex) = currPt;
            upperBounds.at(currVarIndex) = currPt;
        }
        else
        {
            env->output->outputWarning("     Cannot fix variable value for variable with index "
                + std::to_string(currVarIndex) + ": not within bounds ("
                + Utilities::toString(sourceProblem->getVariableLowerBound(currVarIndex)) + " < "
                + Utilities::toString(currPt) + " < "
                + Utilities::toString(sourceProblem->getVariableUpperBound(currVarIndex)));
        }
    }

    env->output->outputDebug(" All fixed variables defined.");
}

void NLPSolverIpoptBase::unfixVariables()
{
    env->output->outputDebug(" Starting reset of fixed variables in Ipopt.");

    for(int k = 0; k < fixedVariableIndexes.size(); k++)
    {
        int currVarIndex = fixedVariableIndexes.at(k);
        double newLB = lowerBoundsBeforeFix.at(k);
        double newUB = upperBoundsBeforeFix.at(k);

        lowerBounds[currVarIndex] = newLB;
        upperBounds[currVarIndex] = newUB;

        env->output->outputDebug("  Resetting initial bounds for variable " + std::to_string(currVarIndex)
            + " lb = " + Utilities::toString(newLB) + " ub = " + Utilities::toString(newUB));
    }

    fixedVariableIndexes.clear();
    fixedVariableValues.clear();
    lowerBoundsBeforeFix.clear();
    upperBoundsBeforeFix.clear();

    setInitialSettings(); // Must initialize it again since the class contains fixed variable bounds and starting
                          // points

    env->output->outputDebug(" Reset of fixed variables in Ipopt completed.");
}

void NLPSolverIpoptBase::updateVariableLowerBound(int variableIndex, double bound)
{
    lowerBounds[variableIndex] = bound;
}

void NLPSolverIpoptBase::updateVariableUpperBound(int variableIndex, double bound)
{
    upperBounds[variableIndex] = bound;
}

void NLPSolverIpoptBase::updateSettings() {}

void NLPSolverIpoptBase::saveOptionsToFile(std::string fileName) {}

void NLPSolverIpoptBase::saveProblemToFile(std::string fileName) {}
} // namespace SHOT
