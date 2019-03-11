/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverIpoptBase.h"

namespace SHOT
{

using namespace Ipopt;

IpoptProblem::IpoptProblem(
    EnvironmentPtr envPtr, ProblemPtr originalProblem, std::shared_ptr<NLPSolverIpoptBase> solver)
    : env(envPtr), sourceProblem(originalProblem), solver(solver)
{
}

// returns the size of the problem
bool IpoptProblem::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n = sourceProblem->properties.numberOfVariables;
    m = sourceProblem->properties.numberOfNumericConstraints;

    nnz_jac_g = sourceProblem->objectiveFunction->getGradientSparsityPattern()->size();
    nnz_h_lag = sourceProblem->objectiveFunction->getHessianSparsityPattern()->size();

    for(auto& C : sourceProblem->numericConstraints)
    {
        nnz_jac_g += C->getGradientSparsityPattern()->size();
        nnz_h_lag += C->getHessianSparsityPattern()->size();
    }

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

    return (true);
}

bool IpoptProblem::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u)
{
    for(int i = 0; i < n; i++)
    {
        x_l[i] = sourceProblem->getVariableLowerBound(i);
        x_u[i] = sourceProblem->getVariableUpperBound(i);
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
    // Here, we assume we only have starting values for x, if you code
    // your own NLP, you can provide starting values for the dual variables
    // if you wish
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    assert(solver->startingPointVariableIndexes.size() == solver->startingPointVariableValues.size());

    int numberOfVariables = sourceProblem->properties.numberOfVariables;

    std::vector<bool> isInitialized(numberOfVariables, false);

    for(int k = 0; k < solver->startingPointVariableIndexes.size(); k++)
    {
        int variableIndex = solver->startingPointVariableIndexes[k];

        if(variableIndex > numberOfVariables)
        {
            env->output->outputError("Variable index " + std::to_string(variableIndex)
                + " is too large when setting initial value in Ipopt.");
            return (false);
        }

        int variableValue = solver->startingPointVariableValues[k];

        double variableLB = sourceProblem->getVariableLowerBound(variableIndex);
        double variableUB = sourceProblem->getVariableUpperBound(variableIndex);

        if(variableUB == SHOT_DBL_MAX)
        {
            if(variableValue < variableLB)
            {
                env->output->outputTrace("Initial value " + std::to_string(variableValue) + " for variable with index "
                    + std::to_string(variableIndex) + " is less than the lower bound " + std::to_string(variableLB));
                continue;
            }
        }
        else if(variableLB == SHOT_DBL_MIN)
        {
            if(variableValue > variableUB)
            {
                env->output->outputTrace("Initial value " + std::to_string(variableValue) + " for variable with index "
                    + std::to_string(variableIndex) + " is larger than the upper bound " + std::to_string(variableUB));
                continue;
            }
        }
        else
        {
            if(variableValue < variableLB || variableValue > variableUB)
            {
                env->output->outputTrace("Initial value " + std::to_string(variableValue) + " for variable with index "
                    + std::to_string(variableIndex) + " is not within variable bounds [" + std::to_string(variableLB)
                    + "," + std::to_string(variableUB));
                continue;
            }
        }

        x[variableIndex] = variableValue;
        isInitialized[variableIndex] = true;
    }

    double defaultInitValue = 1.7171; // TODO: why?

    for(int k = 0; k < numberOfVariables; k++)
    {
        double variableLB = sourceProblem->getVariableLowerBound(k);
        double variableUB = sourceProblem->getVariableUpperBound(k);

        if(variableUB == SHOT_DBL_MAX)
        {
            if(defaultInitValue > variableLB)
            {
                x[k] = variableLB;
            }
            else
            {
                x[k] = defaultInitValue;
            }
        }
        else if(variableLB == SHOT_DBL_MIN)
        {
            if(defaultInitValue > variableUB)
            {
                x[k] = variableUB;
            }
            else
            {
                x[k] = defaultInitValue;
            }
        }
        else if(variableLB <= defaultInitValue && defaultInitValue <= variableUB)
        {
            x[k] = variableUB;
        }
        else if(variableLB > defaultInitValue)
        {
            x[k] = variableLB;
        }
        else
        {
            x[k] = variableUB;
        }
    }

    return (true);
}

// returns the value of the objective function
bool IpoptProblem::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    if(new_x)
    {
        int numberOfVariables = sourceProblem->properties.numberOfVariables;
        VectorDouble vectorPoint(numberOfVariables);

        for(int i = 0; i < numberOfVariables; i++)
        {
            vectorPoint[i] = x[i];
        }

        cachedObjectiveValue = sourceProblem->objectiveFunction->calculateValue(vectorPoint);
    }

    obj_value = cachedObjectiveValue;

    return (true);
}

bool IpoptProblem::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
    if(new_x)
    {
        int numberOfVariables = sourceProblem->properties.numberOfVariables;
        VectorDouble vectorPoint(numberOfVariables);

        for(int i = 0; i < numberOfVariables; i++)
        {
            vectorPoint[i] = x[i];
        }

        auto sparseGradient = sourceProblem->objectiveFunction->calculateGradient(vectorPoint, true);

        cachedObjectiveGradient = VectorDouble(numberOfVariables, 0.0);

        for(auto& G : sparseGradient)
            cachedObjectiveGradient[G.first->index] = G.second;
    }

    grad_f = &cachedObjectiveGradient[0];

    return (true);
}

// return the value of the constraints: g(x)
bool IpoptProblem::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
    if(new_x)
    {
        int numberOfVariables = sourceProblem->properties.numberOfVariables;
        int numberOfConstraints = sourceProblem->properties.numberOfNumericConstraints;

        VectorDouble vectorPoint(numberOfVariables);

        for(int i = 0; i < numberOfVariables; i++)
        {
            vectorPoint[i] = x[i];
        }

        cachedConstraintValues = VectorDouble(numberOfConstraints);

        for(int i = 0; i < numberOfConstraints; i++)
        {
            cachedConstraintValues[i] = sourceProblem->numericConstraints[i]->calculateFunctionValue(vectorPoint);
        }
    }

    g = &cachedConstraintValues[0];
}

// return the structure or values of the jacobian
bool IpoptProblem::eval_jac_g(
    Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index* jCol, Number* values)
{
    int counter = 0;

    if(values == NULL)
    {
        for(auto& C : sourceProblem->numericConstraints)
        {
            for(auto& E : *C->getGradientSparsityPattern())
            {
                iRow[counter] = C->index;
                jCol[counter] = E->index;
                counter++;
            }
        }

        return (true);
    }

    int numberOfVariables = sourceProblem->properties.numberOfVariables;

    VectorDouble vectorPoint(numberOfVariables);

    for(int i = 0; i < numberOfVariables; i++)
    {
        vectorPoint[i] = x[i];
    }

    for(auto& C : sourceProblem->numericConstraints)
    {
        auto jacobian = C->calculateGradient(vectorPoint, false);

        for(auto& E : jacobian)
        {
            if(iRow[counter] == C->index && jCol[counter] == E.first->index)
            {
                values[counter] = E.second;
            }
            else // In case some elements have value zero
            {
                values[counter] = 0.0;
            }

            counter++;
        }
    }

    assert(counter == nele_jac);

    return true;
}

// return the structure or values of the Hessian of the Langragian
bool IpoptProblem::eval_h(Index n, const Number* x, bool new_x, Number obj_factor, Index m, const Number* lambda,
    bool new_lambda, Index nele_hess, Index* iRow, Index* jCol, Number* values)
{
    if(values == NULL)
    {
        int counter = 0;

        for(auto& E : *sourceProblem->objectiveFunction->getHessianSparsityPattern())
        {
            iRow[counter] = E.first->index;
            jCol[counter] = E.second->index;
            counter++;
        }

        for(auto& C : sourceProblem->numericConstraints)
        {
            for(auto& E : *C->getHessianSparsityPattern())
            {
                iRow[counter] = E.first->index;
                jCol[counter] = E.second->index;
                counter++;
            }
        }

        return (true);
    }

    int numberOfVariables = sourceProblem->properties.numberOfVariables;
    int counter = 0;

    VectorDouble vectorPoint(numberOfVariables);

    for(int i = 0; i < numberOfVariables; i++)
    {
        vectorPoint[i] = x[i];
    }

    for(auto& E : sourceProblem->objectiveFunction->calculateHessian(vectorPoint, false))
    {
        if(iRow[counter] == E.first.first->index && jCol[counter] == E.first.second->index)
        {
            values[counter] = obj_factor * E.second;
        }
        else
        {
            values[counter] = 0.0;
        }

        counter++;
    }

    int lambdaCounter = 0;

    for(auto& C : sourceProblem->numericConstraints)
    {
        auto hessian = C->calculateHessian(vectorPoint, false);

        for(auto& E : hessian)
        {
            if(iRow[counter] == E.first.first->index && jCol[counter] == E.first.second->index)
            {
                values[counter] = E.second * lambda[lambdaCounter];
            }
            else // In case some elements have value zero
            {
                values[counter] = 0.0;
            }

            counter++;
            lambdaCounter++;
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

        break;

    case MAXITER_EXCEEDED:
        solutionDescription = "Maximum number of iterations exceeded.";

        solutionStatus = E_NLPSolutionStatus::IterationLimit;

        if(x != NULL)
        {
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case STOP_AT_TINY_STEP:
        solutionDescription = "Algorithm proceeds with very little progress.";

        solutionStatus = E_NLPSolutionStatus::IterationLimit;

        if(x != NULL)
        {
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

        if(x != NULL)
        {
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

        if(x != NULL)
        {
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case DIVERGING_ITERATES:
        solutionDescription = "It seems that the iterates diverge.";

        solutionStatus = E_NLPSolutionStatus::Unbounded;

        if(x != NULL)
        {
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case RESTORATION_FAILURE:
        solutionDescription = "Restoration phase failed, algorithm doesn't know how to proceed.";

        solutionStatus = E_NLPSolutionStatus::Error;

        if(x != NULL)
        {
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case ERROR_IN_STEP_COMPUTATION:
        solutionDescription = "An unrecoverable error occurred while Ipopt tried to compute the search direction.";

        solutionStatus = E_NLPSolutionStatus::Error;

        if(x != NULL)
        {
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case INVALID_NUMBER_DETECTED:
        solutionDescription = "Algorithm received an invalid number (such as NaN or Inf) from the NLP.";

        solutionStatus = E_NLPSolutionStatus::Error;

        if(x != NULL)
        {
            variableSolution = VectorDouble(numberOfVariables);

            for(int i = 0; i < numberOfVariables; i++)
                variableSolution[i] = x[i];

            objectiveValue = obj_value;
        }

        break;

    case INTERNAL_ERROR:
        solutionDescription = "An unknown internal error occurred.";

        solutionStatus = E_NLPSolutionStatus::Error;

        if(x != NULL)
        {
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

    env->output->outputCritical("Ipopt terminated with status: " + solutionDescription);
}

E_NLPSolutionStatus NLPSolverIpoptBase::solveProblemInstance()
{
    env->output->outputDebug(" Starting solution of Ipopt problem.");

    auto timeLimit = env->settings->getDoubleSetting("TimeLimit", "Termination") - env->timing->getElapsedTime("Total");

    E_NLPSolutionStatus status;

    IpoptNLPSolver = std::make_unique<IpoptSolver>();

    try
    {

        setIntegers(false);

        IpoptNLPSolver->osinstance = osInstance.get();
        updateSettings();

        std::string solStatus;

        try
        {
            IpoptNLPSolver->solve();
            // std::cout << "\e[A"; // Fix for removing unwanted output
            // std::cout << "\e[A";
            solStatus = IpoptNLPSolver->osresult->getSolutionStatusType(0);
        }
        catch(ErrorClass e)
        {
            env->output->outputError(" Error when solving NLP problem with Ipopt", e.errormsg);
            solStatus == "other";
        }

        if(solStatus == "globallyOptimal")
        {
            env->output->outputDebug(" Global solution found to relaxed problem with Ipopt.");
            status = E_NLPSolutionStatus::Optimal;
        }

        if(solStatus == "locallyOptimal")
        {
            env->output->outputDebug(" Local solution found to relaxed problem with Ipopt.");
            status = E_NLPSolutionStatus::Optimal;
        }

        if(solStatus == "optimal")
        {
            env->output->outputDebug(" Optimal solution found to relaxed problem with Ipopt.");
            status = E_NLPSolutionStatus::Optimal;
        }

        if(solStatus == "bestSoFar")
        {
            env->output->outputDebug(" Feasible solution found to relaxed problem with Ipopt.");
            status = E_NLPSolutionStatus::Feasible;
        }

        if(solStatus == "stoppedByLimit")
        {
            env->output->outputDebug(" No solution found to problem with Ipopt: Time or iteration limit exceeded.");
            status = E_NLPSolutionStatus::IterationLimit;
        }

        if(solStatus == "infeasible")
        {
            env->output->outputDebug(" No solution found to problem with Ipopt: Infeasible problem detected.");
            status = E_NLPSolutionStatus::Infeasible;
        }

        if(solStatus == "unsure")
        {
            env->output->outputDebug(" No solution found to problem with Ipopt, solution code: unsure.");
            status = E_NLPSolutionStatus::Infeasible;
        }

        if(solStatus == "other")
        {
            env->output->outputDebug(" No solution found to problem with Ipopt, solution code: other.");
            status = E_NLPSolutionStatus::Infeasible;
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

    setIntegers(true);
    return (status);
}

double NLPSolverIpoptBase::getSolution(int i)
{
    double value = IpoptNLPSolver->osresult->getVarValue(0, i);
    return (value);
}

double NLPSolverIpoptBase::getObjectiveValue()
{
    double value = IpoptNLPSolver->osresult->getObjValue(0, 0);
    return (value);
}

void NLPSolverIpoptBase::setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues)
{
    startingPointVariableIndexes = variableIndexes;
    startingPointVariableValues = variableValues;

    int startingPointSize = startingPointVariableIndexes.size();

    if(startingPointSize == 0)
        return;

    auto lbs = osInstance->getVariableLowerBounds();
    auto ubs = osInstance->getVariableUpperBounds();

    env->output->outputDebug(" Adding starting points to Ipopt.");

    for(int k = 0; k < startingPointSize; k++)
    {
        int currVarIndex = startingPointVariableIndexes.at(k);
        auto currPt = startingPointVariableValues.at(k);

        auto currLB = lbs[currVarIndex];
        auto currUB = ubs[currVarIndex];

        if(currPt > currUB)
        {
            env->output->outputDebug("  Starting point value for variable " + std::to_string(currVarIndex)
                + " is larger than ub: " + UtilityFunctions::toString(currPt) + " > "
                + UtilityFunctions::toString(currUB) + "; resetting to ub.");
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
                + " is smaller than lb: " + UtilityFunctions::toString(currPt) + " < "
                + UtilityFunctions::toString(currLB) + "; resetting to lb.");

            if(currLB == 0 && currPt < 0 && currPt > -0.00001)
            {
                currPt = 0;
            }
            else
            {
                currPt = currLB;
            }
        }

        osOption->setAnotherInitVarValue(currVarIndex, currPt);

        env->output->outputTrace("  Starting point value for " + std::to_string(currVarIndex)
            + " set: " + UtilityFunctions::toString(currLB) + " < " + UtilityFunctions::toString(currPt) + " < "
            + UtilityFunctions::toString(currUB));
    }

    osOption->setAnotherSolverOption("warm_start_init_point", "yes", "ipopt", "", "string", "");

    env->output->outputDebug(" All starting points set.");
}

void NLPSolverIpoptBase::clearStartingPoint()
{
    startingPointVariableIndexes.clear();
    startingPointVariableValues.clear();
    setInitialSettings();
    setSolverSpecificInitialSettings();
}

VectorDouble NLPSolverIpoptBase::getVariableLowerBounds()
{
    double* tmpArray = osInstance->getVariableLowerBounds();

    VectorDouble tmpVector(tmpArray, tmpArray + osInstance->getVariableNumber());

    return (tmpVector);
}

VectorDouble NLPSolverIpoptBase::getVariableUpperBounds()
{
    double* tmpArray = osInstance->getVariableUpperBounds();

    VectorDouble tmpVector(tmpArray, tmpArray + osInstance->getVariableNumber());

    return (tmpVector);
}

VectorDouble NLPSolverIpoptBase::getSolution()
{
    int numVar = osInstance->getVariableNumber();
    VectorDouble tmpPoint(numVar);

    for(int i = 0; i < numVar; i++)
    {
        tmpPoint.at(i) = NLPSolverIpoptBase::getSolution(i);
    }

    return (tmpPoint);
}

void NLPSolverIpoptBase::setInitialSettings()
{
    osOption = std::make_unique<OSOption>();

    std::string IpoptSolver = "";

    // Sets the linear solver used
    switch(static_cast<ES_IpoptSolver>(env->settings->getIntSetting("Ipopt.LinearSolver", "Subsolver")))
    {
    case(ES_IpoptSolver::ma27):
        IpoptSolver = "ma27";
        break;

    case(ES_IpoptSolver::ma57):
        IpoptSolver = "ma57";
        break;

    case(ES_IpoptSolver::ma86):
        IpoptSolver = "ma86";
        break;

    case(ES_IpoptSolver::ma97):
        IpoptSolver = "ma97";
        break;

    case(ES_IpoptSolver::mumps):
        IpoptSolver = "mumps";
        break;
    default:
        IpoptSolver = "mumps";
    }

    osOption->setAnotherSolverOption("linear_solver", IpoptSolver, "ipopt", "", "string", "");

    switch(static_cast<E_LogLevel>(env->settings->getIntSetting("Console.LogLevel", "Output")))
    {
    case E_LogLevel::Off:
    case E_LogLevel::Critical:
    case E_LogLevel::Error:
    case E_LogLevel::Warning:
    case E_LogLevel::Info:
        osOption->setAnotherSolverOption("print_level", "0", "ipopt", "", "integer", "");
        break;
    case E_LogLevel::Debug:
        osOption->setAnotherSolverOption("print_level", "8", "ipopt", "", "integer", "");
        break;
    case E_LogLevel::Trace:
        osOption->setAnotherSolverOption("print_level", "10", "ipopt", "", "integer", "");
        break;
    default:
        break;
    }

    // Suppress copyright message
    if(env->settings->getIntSetting("Console.LogLevel", "Output") > (int)E_LogLevel::Debug)
    {
        osOption->setAnotherSolverOption("sb", "yes", "ipopt", "", "string", "");
    }

    // osOption->setAnotherSolverOption("fixed_variable_treatment", "make_constraint", "ipopt", "", "string", "");

    /*
         // Misc options
         osOption->setAnotherSolverOption("acceptable_iter", "15", "ipopt", "", "integer", "");
         osOption->setAnotherSolverOption("acceptable_obj_change_tol", "1E20", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("acceptable_compl_inf_tol", "0.01", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("acceptable_compl_viol_tol", "0.01", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("acceptable_dual_inf_tol", "100000000000000", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("barrier_tol_factor", "10", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("expect_infeasible_problem", "no", "ipopt", "", "string", "");
         osOption->setAnotherSolverOption("mu_init", "0.1", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("mu_max", "100000", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("mu_min", "1E-11", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("resto_failure_feasibility_threshold", "0", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("resto_penalty_parameter", "1000", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("resto_proximity_weight", "1", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("gamma_theta", "1E-5", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("required_infeasibility_reduction", "0.5", "ipopt", "", "double", "");

         osOption->setAnotherSolverOption("constr_viol_tol", "0.01", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("dual_inf_tol", "1", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("compl_inf_tol", "0.0001", "ipopt", "", "double", "");
         osOption->setAnotherSolverOption("mu_strategy", "adaptive", "ipopt", "", "string", "");
         osOption->setAnotherSolverOption("mu_oracle", "quality-function", "ipopt", "", "string", "");
         osOption->setAnotherSolverOption("check_derivatives_for_naninf", "yes", "ipopt", "", "string", "");

         */

    setSolverSpecificInitialSettings();
}

void NLPSolverIpoptBase::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{

    fixedVariableIndexes = variableIndexes;
    fixedVariableValues = variableValues;

    int size = fixedVariableIndexes.size();

    if(size == 0)
        return;

    auto lbs = osInstance->getVariableLowerBounds();
    auto ubs = osInstance->getVariableUpperBounds();

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
        auto currPt = fixedVariableValues.at(k);

        auto currLB = lbs[currVarIndex];
        auto currUB = ubs[currVarIndex];

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
                + " is larger than ub: " + UtilityFunctions::toString(currPt) + " > " + std::to_string(currUB));
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
                + " is smaller than lb: " + UtilityFunctions::toString(currPt) + " < " + std::to_string(currLB));

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
            + UtilityFunctions::toString(currLB) + " <= " + UtilityFunctions::toString(currPt)
            + " <= " + UtilityFunctions::toString(currUB));

        if(currPt >= osInstance->instanceData->variables->var[currVarIndex]->lb
            && currPt <= osInstance->instanceData->variables->var[currVarIndex]->ub)
        {
            osInstance->instanceData->variables->var[currVarIndex]->lb = currPt;
            osInstance->instanceData->variables->var[currVarIndex]->ub = currPt;
            osInstance->bVariablesModified = true;
        }
        else
        {
            env->output->outputWarning("     Cannot fix variable value for variable with index "
                + std::to_string(currVarIndex) + ": not within bounds ("
                + UtilityFunctions::toString(osInstance->instanceData->variables->var[currVarIndex]->lb) + " < "
                + UtilityFunctions::toString(currPt) + " < "
                + UtilityFunctions::toString(osInstance->instanceData->variables->var[currVarIndex]->ub));
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

        osInstance->instanceData->variables->var[currVarIndex]->lb = newLB;
        osInstance->instanceData->variables->var[currVarIndex]->ub = newUB;
        osInstance->bVariablesModified = true;

        env->output->outputDebug("  Resetting initial bounds for variable " + std::to_string(currVarIndex)
            + " lb = " + UtilityFunctions::toString(newLB) + " ub = " + UtilityFunctions::toString(newUB));
    }

    fixedVariableIndexes.clear();
    fixedVariableValues.clear();
    lowerBoundsBeforeFix.clear();
    upperBoundsBeforeFix.clear();

    setInitialSettings(); // Must initialize it again since the class contains fixed variable bounds and starting
                          // points

    env->output->outputDebug(" Reset of fixed variables in Ipopt completed.");
}

void NLPSolverIpoptBase::setIntegers(bool useDiscrete)
{
    for(int i = 0; i < osInstance->getVariableNumber(); i++)
    {
        if(useDiscrete)
        {
            osInstance->instanceData->variables->var[i]->type = originalVariableType[i];
        }
        else
        {
            osInstance->instanceData->variables->var[i]->type = 'C';
        }
    }

    osInstance->bVariablesModified = true;
}

void NLPSolverIpoptBase::updateVariableLowerBound(int variableIndex, double bound)
{
    osInstance->instanceData->variables->var[variableIndex]->lb = bound;
    osInstance->bVariablesModified = true;
}

void NLPSolverIpoptBase::updateVariableUpperBound(int variableIndex, double bound)
{
    osInstance->instanceData->variables->var[variableIndex]->ub = bound;
    osInstance->bVariablesModified = true;
}

void NLPSolverIpoptBase::updateSettings()
{
    IpoptNLPSolver->osoption = osOption.get();
    IpoptNLPSolver->osol = osolwriter->writeOSoL(osOption.get());
}

void NLPSolverIpoptBase::saveOptionsToFile(std::string fileName)
{
    osolwriter->m_bWhiteSpace = false;

    std::stringstream ss;
    ss << osolwriter->writeOSoL(osOption.get());

    UtilityFunctions::writeStringToFile(fileName, ss.str());
}

void NLPSolverIpoptBase::saveProblemToFile(std::string fileName)
{
    std::string problem = osInstance->printModel();

    UtilityFunctions::writeStringToFile(fileName, problem);
}
} // namespace SHOT
