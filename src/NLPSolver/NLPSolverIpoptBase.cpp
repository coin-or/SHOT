/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverIpoptBase.h"

#include <cstdio>

#include "../Output.h"
#include "../Settings.h"
#include "../Utilities.h"

namespace SHOT
{

using namespace Ipopt;

void IpoptJournal::PrintImpl(Ipopt::EJournalCategory category, Ipopt::EJournalLevel level, const char* str)
{
    auto lines = Utilities::splitStringByCharacter(str, '\n');

    switch(level)
    {
    case J_NONE:
        return;
    case J_INSUPPRESSIBLE:
    case J_ERROR:
    case J_STRONGWARNING:
    case J_WARNING:
    case J_SUMMARY:
    case J_ITERSUMMARY:
    case J_DETAILED:

        for(auto const& line : lines)
            env->output->outputInfo(fmt::format("      | {} ", line));

        break;

    case J_MOREDETAILED:

        for(auto const& line : lines)
            env->output->outputDebug(fmt::format("      | {} ", line));

        break;

    default:
        for(auto const& line : lines)
            env->output->outputTrace(fmt::format("      | {} ", line));

        break;
    }
}

void IpoptJournal::PrintfImpl(
    Ipopt::EJournalCategory category, Ipopt::EJournalLevel level, const char* pformat, va_list ap)
{
    if(level == Ipopt::EJournalLevel::J_NONE)
        return;

    // append to output buffer
    int rc = std::vsnprintf(outBuf + outBufPos, sizeof(outBuf) - outBufPos, pformat, ap);

    if(rc < 0) // ignore error
        return;

    outBufPos += rc;

    // if output buffer terminates with newline or is almost full, then print
    if((outBufPos > 0 && outBuf[outBufPos - 1] == '\n') || outBufPos > (int)sizeof(outBuf) - 100)
    {
        PrintImpl(category, level, outBuf);
        outBufPos = 0;
    }
}

void IpoptJournal::FlushBufferImpl()
{
    if(outBufPos > 0)
    {
        // just make up a category and level, as not used in PrintImpl anyway
        PrintImpl(J_LAST_CATEGORY, J_ALL, outBuf);
        outBufPos = 0;
    }

    env->output->flush();
}

IpoptProblem::IpoptProblem(EnvironmentPtr envPtr, ProblemPtr problem) : env(envPtr), sourceProblem(problem) { }

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
        x_l[i] = lowerBounds[i];
        x_u[i] = upperBounds[i];
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

// gets the linearity of the variables
bool IpoptProblem::get_variables_linearity(Ipopt::Index n, LinearityType* var_types)
{
    assert(n == sourceProblem->properties.numberOfVariables);

    for(int i = 0; i < n; ++i)
    {
        if(sourceProblem->allVariables[i]->properties.isNonlinear)
            var_types[i] = NON_LINEAR;
        else
            var_types[i] = LINEAR;
    }

    return true;
}

// gets the linearity of the constraints
bool IpoptProblem::get_constraints_linearity(Ipopt::Index m, LinearityType* const_types)
{
    assert(m == sourceProblem->properties.numberOfNumericConstraints);

    for(int i = 0; i < m; ++i)
    {
        if(sourceProblem->numericConstraints[i]->properties.classification > E_ConstraintClassification::Linear)
            const_types[i] = NON_LINEAR;
        else
            const_types[i] = LINEAR;
    }

    return true;
}

Ipopt::Index IpoptProblem::get_number_of_nonlinear_variables()
{
    return (sourceProblem->properties.numberOfNonlinearVariables);
}

bool IpoptProblem::get_list_of_nonlinear_variables(
    [[maybe_unused]] Ipopt::Index num_nonlin_vars, Ipopt::Index* pos_nonlin_vars)
{
    VectorInteger nonlinearVariables;
    int count = 0;

    for(int i = 0; i < sourceProblem->properties.numberOfNonlinearVariables; i++)
    {
        pos_nonlin_vars[i] = sourceProblem->nonlinearVariables[i]->index;
        count++;
    }

    assert(count == num_nonlin_vars);

    return true;
}

// returns the initial point for the problem
bool IpoptProblem::get_starting_point(Index n, [[maybe_unused]] bool init_x, [[maybe_unused]] Number* x,
    [[maybe_unused]] bool init_z, [[maybe_unused]] Number* z_L, [[maybe_unused]] Number* z_U, [[maybe_unused]] Index m,
    [[maybe_unused]] bool init_lambda, [[maybe_unused]] Number* lambda)
{
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    std::vector<bool> isInitialized(n, false);

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
        {
            variableValue = -0.99 * divergingIterativesTolerance;
            env->output->outputTrace(
                fmt::format("         Starting point value for variable with index {} is below diverging "
                            "iterates tolerance {}. Setting value to {}.",
                    variableIndex, divergingIterativesTolerance, variableValue));
        }
        else if(variableValue > divergingIterativesTolerance)
        {
            variableValue = 0.99 * divergingIterativesTolerance;

            env->output->outputTrace(
                fmt::format("         Starting point value for variable with index {} is above diverging "
                            "iterates tolerance {}. Setting value to {}.",
                    variableIndex, divergingIterativesTolerance, variableValue));
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
bool IpoptProblem::eval_f(Index n, const Number* x, [[maybe_unused]] bool new_x, Number& obj_value)
{
    VectorDouble vectorPoint(n);

    for(int i = 0; i < n; i++)
        vectorPoint[i] = x[i];

    obj_value = sourceProblem->objectiveFunction->calculateValue(vectorPoint);

    return (true);
}

// Returns the gradient of the objective function
bool IpoptProblem::eval_grad_f(Index n, const Number* x, [[maybe_unused]] bool new_x, Number* grad_f)
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
bool IpoptProblem::eval_g(Index n, const Number* x, [[maybe_unused]] bool new_x, Index m, Number* g)
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
bool IpoptProblem::eval_jac_g(Index n, const Number* x, [[maybe_unused]] bool new_x, [[maybe_unused]] Index m,
    Index nele_jac, Index* iRow, Index* jCol, Number* values)
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

            assert(counter <= nele_jac);
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

            assert(location < nele_jac);
            assert(location >= 0);
        }
    }

    return (true);
}

// Return the structure or values of the Hessian of the Langragian
bool IpoptProblem::eval_h(Index n, const Number* x, [[maybe_unused]] bool new_x, Number obj_factor,
    [[maybe_unused]] Index m, const Number* lambda, [[maybe_unused]] bool new_lambda, Index nele_hess, Index* iRow,
    Index* jCol, Number* values)
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

            assert(location < nele_hess);
            assert(location >= 0);

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

            assert(location < nele_hess);
            assert(location >= 0);

            values[location] += lambda[C->index] * E.second;
        }
    }

    return (true);
}

/*
bool IpoptProblem::get_scaling_parameters(Number& obj_scaling, [[maybe_unused]] bool& use_x_scaling,
    [[maybe_unused]] Index n, [[maybe_unused]] Number* x_scaling, [[maybe_unused]] bool& use_g_scaling,
    [[maybe_unused]] Index m, [[maybe_unused]] Number* g_scaling)
{
    obj_scaling = sourceProblem->objectiveFunction->properties.isMinimize ? 1.0 : -1.0;

    use_x_scaling = false;
    use_g_scaling = false;

    return (true);
}*/

void IpoptProblem::finalize_solution(SolverReturn status, [[maybe_unused]] Index n, const Number* x,
    [[maybe_unused]] const Number* z_L, [[maybe_unused]] const Number* z_U, [[maybe_unused]] Index m,
    [[maybe_unused]] const Number* g, [[maybe_unused]] const Number* lambda, Number obj_value,
    [[maybe_unused]] const IpoptData* ip_data, [[maybe_unused]] IpoptCalculatedQuantities* ip_cq)
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

    env->output->outputDebug("        Ipopt terminated with status: " + solutionDescription);
}

E_NLPSolutionStatus NLPSolverIpoptBase::solveProblemInstance()
{
    env->output->outputDebug("        Starting solution of Ipopt problem.");

    E_NLPSolutionStatus status;
    ipoptProblem->variableSolution.clear();

    try
    {
        Ipopt::ApplicationReturnStatus ipoptStatus;

        if(!hasBeenSolved)
        {
            ipoptStatus = ipoptApplication->OptimizeTNLP(ipoptProblem);
        }
        else
        {
            ipoptStatus = ipoptApplication->ReOptimizeTNLP(ipoptProblem);
            hasBeenSolved = true;
        }

        switch(ipoptStatus)
        {
        case Ipopt::ApplicationReturnStatus::Solve_Succeeded:
            status = E_NLPSolutionStatus::Optimal;
            env->output->outputDebug("        Global solution found with Ipopt.");
            break;

        case Ipopt::ApplicationReturnStatus::Feasible_Point_Found:
        case Ipopt::ApplicationReturnStatus::Solved_To_Acceptable_Level:
            status = E_NLPSolutionStatus::Feasible;
            env->output->outputDebug("        Feasible solution found with Ipopt.");
            break;

        case Ipopt::ApplicationReturnStatus::Infeasible_Problem_Detected:
            status = E_NLPSolutionStatus::Infeasible;
            env->output->outputDebug("        No solution found to problem with Ipopt: Infeasible problem detected.");
            break;

        case Ipopt::ApplicationReturnStatus::Maximum_Iterations_Exceeded:
            status = E_NLPSolutionStatus::IterationLimit;
            env->output->outputDebug("        No solution found to problem with Ipopt: Iteration limit exceeded.");
            break;

        case Ipopt::ApplicationReturnStatus::Maximum_CpuTime_Exceeded:
            status = E_NLPSolutionStatus::TimeLimit;
            env->output->outputDebug("        No solution found to problem with Ipopt: Time limit exceeded.");
            break;

        case Ipopt::ApplicationReturnStatus::Diverging_Iterates:
            status = E_NLPSolutionStatus::Unbounded;
            env->output->outputDebug("        No solution found to problem with Ipopt: Diverging iterates.");
            break;

        default:
            status = E_NLPSolutionStatus::Error;
            env->output->outputError("        Error when solving NLP problem with Ipopt.");
            break;
        }
    }

    catch(std::exception& e)
    {
        env->output->outputError("        Error when solving problem with Ipopt!", e.what());
        status = E_NLPSolutionStatus::Error;
    }
    catch(...)
    {
        env->output->outputError("        Unspecified error when solving problem with Ipopt!");
        status = E_NLPSolutionStatus::Error;
    }

    env->output->outputDebug("        Finished solution of Ipopt problem.");

    return (status);
}

double NLPSolverIpoptBase::getSolution(int i) { return (ipoptProblem->variableSolution[i]); }

double NLPSolverIpoptBase::getObjectiveValue() { return (ipoptProblem->objectiveValue); }

void NLPSolverIpoptBase::setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues)
{
    ipoptProblem->startingPointVariableIndexes = variableIndexes;
    ipoptProblem->startingPointVariableValues = variableValues;

    int startingPointSize = ipoptProblem->startingPointVariableIndexes.size();

    if(startingPointSize == 0)
        return;

    env->output->outputDebug("        Adding starting points to Ipopt.");

    for(int k = 0; k < startingPointSize; k++)
    {
        int currVarIndex = ipoptProblem->startingPointVariableIndexes.at(k);
        auto currPt = ipoptProblem->startingPointVariableValues.at(k);

        auto currLB = sourceProblem->getVariableLowerBound(currVarIndex);
        auto currUB = sourceProblem->getVariableUpperBound(currVarIndex);

        if(currPt > currUB)
        {
            env->output->outputDebug("         Starting point value for variable " + std::to_string(currVarIndex)
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
            env->output->outputDebug("         Starting point value for variable " + std::to_string(currVarIndex)
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

        ipoptProblem->startingPointVariableValues.at(k) = currPt;

        env->output->outputTrace("         Starting point value for " + std::to_string(currVarIndex) + " set: "
            + Utilities::toString(currLB) + " < " + Utilities::toString(currPt) + " < " + Utilities::toString(currUB));
    }

    env->output->outputDebug("        All starting points set.");
}

void NLPSolverIpoptBase::clearStartingPoint()
{
    ipoptProblem->startingPointVariableIndexes.clear();
    ipoptProblem->startingPointVariableValues.clear();
    setInitialSettings();
    setSolverSpecificInitialSettings();
}

VectorDouble NLPSolverIpoptBase::getVariableLowerBounds() { return (ipoptProblem->lowerBounds); }

VectorDouble NLPSolverIpoptBase::getVariableUpperBounds() { return (ipoptProblem->upperBounds); }

VectorDouble NLPSolverIpoptBase::getSolution() { return (ipoptProblem->variableSolution); }

void NLPSolverIpoptBase::setInitialSettings()
{
    std::string subsolver = "";

    // Sets the linear solver used
    switch(static_cast<ES_IpoptSolver>(env->settings->getSetting<int>("Ipopt.LinearSolver", "Subsolver")))
    {
    case(ES_IpoptSolver::ma27):
        ipoptApplication->Options()->SetStringValue("linear_solver", "ma27");
        ipoptApplication->Options()->SetStringValue("linear_system_scaling", "mc19", true, true);
        break;

    case(ES_IpoptSolver::ma57):
        ipoptApplication->Options()->SetStringValue("linear_solver", "ma57");
        ipoptApplication->Options()->SetStringValue("linear_system_scaling", "mc19", true, true);
        break;

    case(ES_IpoptSolver::ma86):
        ipoptApplication->Options()->SetStringValue("linear_solver", "ma86");
        ipoptApplication->Options()->SetStringValue("linear_system_scaling", "mc19", true, true);
        break;

    case(ES_IpoptSolver::ma97):
        ipoptApplication->Options()->SetStringValue("linear_solver", "ma97");
        ipoptApplication->Options()->SetStringValue("linear_system_scaling", "mc19", true, true);
        break;

    case(ES_IpoptSolver::mumps):
        ipoptApplication->Options()->SetStringValue("linear_solver", "mumps");
        break;

    case(ES_IpoptSolver::IpoptDefault):
    default:
        break;
    }

    // ipoptApplication->Options()->SetStringValue("fixed_variable_treatment", "make_parameter");
    // ipoptApplication->Options()->SetStringValue("hessian_approximation", "limited-memory");

    if(!env->settings->getSetting<bool>("Console.PrimalSolver.Show", "Output"))
    {
        ipoptApplication->Options()->SetIntegerValue("print_level", J_NONE);
        ipoptApplication->Options()->SetStringValue("sb", "yes");
    }

    // ipoptApplication->Options()->SetStringValue("derivative_test", "second-order");
    // ipoptApplication->Options()->SetStringValue("derivative_test_print_all", "no");

    // These are default settings for Ipopt in Bonmin, so should work here as well
    ipoptApplication->Options()->SetNumericValue("bound_relax_factor", 1e-8, true, true);
    ipoptApplication->Options()->SetStringValue("mu_strategy", "adaptive", true, true);
    // ipoptApplication->Options()->SetStringValue("ma86_order", "auto", true, true);
    ipoptApplication->Options()->SetStringValue("mu_oracle", "probing", true, true);
    ipoptApplication->Options()->SetStringValue("expect_infeasible_problem", "yes", true, true);
    ipoptApplication->Options()->SetStringValue("warm_start_init_point", "no", true, true);
    ipoptApplication->Options()->SetNumericValue("gamma_phi", 1e-8, true, true);
    ipoptApplication->Options()->SetNumericValue("gamma_theta", 1e-4, true, true);
    ipoptApplication->Options()->SetNumericValue("required_infeasibility_reduction", 0.1, true, true);
    // ipoptApplication->Options()->SetStringValue("nlp_scaling_method", "none", true, true);
    ipoptApplication->Options()->SetNumericValue(
        "obj_scaling_factor", sourceProblem->objectiveFunction->properties.isMinimize ? 1.0 : -1.0, true, true);

    // if we have linear constraint and a quadratic objective, then the hessian of the Lagrangian is constant, and
    // Ipopt can make use of this
    if(sourceProblem->properties.isMIQPProblem)
        ipoptApplication->Options()->SetStringValue("hessian_constant", "yes", true, true);

    setSolverSpecificInitialSettings();
}

void NLPSolverIpoptBase::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    ipoptProblem->fixedVariableIndexes = variableIndexes;
    ipoptProblem->fixedVariableValues = variableValues;

    int size = ipoptProblem->fixedVariableIndexes.size();

    if(size == 0)
        return;

    if(lowerBoundsBeforeFix.size() > 0 || upperBoundsBeforeFix.size() > 0)
    {
        env->output->outputDebug("        Old variable fixes remain for Ipopt solver, resetting!");
        lowerBoundsBeforeFix.clear();
        upperBoundsBeforeFix.clear();
    }

    env->output->outputDebug("        Defining fixed variables in Ipopt.");

    for(int k = 0; k < size; k++)
    {
        int currVarIndex = ipoptProblem->fixedVariableIndexes.at(k);
        double currPt = ipoptProblem->fixedVariableValues.at(k);

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

        env->output->outputTrace("         Setting fixed value for variable " + std::to_string(currVarIndex) + ": "
            + Utilities::toString(currLB) + " <= " + Utilities::toString(currPt)
            + " <= " + Utilities::toString(currUB));

        if(currPt >= sourceProblem->getVariableLowerBound(currVarIndex)
            && currPt <= sourceProblem->getVariableUpperBound(currVarIndex))
        {
            ipoptProblem->lowerBounds.at(currVarIndex) = currPt;
            ipoptProblem->upperBounds.at(currVarIndex) = currPt;
        }
        else
        {
            env->output->outputDebug("          Cannot fix variable value for variable with index "
                + std::to_string(currVarIndex) + ": not within bounds ("
                + Utilities::toString(sourceProblem->getVariableLowerBound(currVarIndex)) + " < "
                + Utilities::toString(currPt) + " < "
                + Utilities::toString(sourceProblem->getVariableUpperBound(currVarIndex)));
        }
    }

    env->output->outputDebug("        All fixed variables defined.");
}

void NLPSolverIpoptBase::unfixVariables()
{
    env->output->outputDebug("        Starting reset of fixed variables in Ipopt.");

    for(size_t k = 0; k < ipoptProblem->fixedVariableIndexes.size(); k++)
    {
        int currVarIndex = ipoptProblem->fixedVariableIndexes.at(k);
        double newLB = lowerBoundsBeforeFix.at(k);
        double newUB = upperBoundsBeforeFix.at(k);

        ipoptProblem->lowerBounds[currVarIndex] = newLB;
        ipoptProblem->upperBounds[currVarIndex] = newUB;

        env->output->outputTrace("         Resetting initial bounds for variable " + std::to_string(currVarIndex)
            + " lb = " + Utilities::toString(newLB) + " ub = " + Utilities::toString(newUB));
    }

    ipoptProblem->fixedVariableIndexes.clear();
    ipoptProblem->fixedVariableValues.clear();
    lowerBoundsBeforeFix.clear();
    upperBoundsBeforeFix.clear();

    setInitialSettings(); // Must initialize it again since the class contains fixed variable bounds and starting
                          // points

    env->output->outputDebug("        Reset of fixed variables in Ipopt completed.");
}

void NLPSolverIpoptBase::updateVariableLowerBound(int variableIndex, double bound)
{
    ipoptProblem->lowerBounds[variableIndex] = bound;
}

void NLPSolverIpoptBase::updateVariableUpperBound(int variableIndex, double bound)
{
    ipoptProblem->upperBounds[variableIndex] = bound;
}

void NLPSolverIpoptBase::updateSettings() { }

void NLPSolverIpoptBase::saveOptionsToFile([[maybe_unused]] std::string fileName) { }

void NLPSolverIpoptBase::saveProblemToFile([[maybe_unused]] std::string fileName) { }

std::string NLPSolverIpoptBase::getSolverDescription()
{
    std::string linearSolver = "";

    switch(static_cast<ES_IpoptSolver>(env->settings->getSetting<int>("Ipopt.LinearSolver", "Subsolver")))
    {
    case(ES_IpoptSolver::ma27):
        linearSolver = "HSL MA27";
        break;

    case(ES_IpoptSolver::ma57):
        linearSolver = "HSL MA57";
        break;

    case(ES_IpoptSolver::ma86):
        linearSolver = "HSL MA86";
        break;

    case(ES_IpoptSolver::ma97):
        linearSolver = "HSL MA97";
        break;

    case(ES_IpoptSolver::mumps):
        linearSolver = "MUMPS";
        break;
    default:
        linearSolver = "default linear solver";
    }

    std::string description = fmt::format("Ipopt {} (with {})", IPOPT_VERSION, linearSolver);

    return (description);
};
} // namespace SHOT
