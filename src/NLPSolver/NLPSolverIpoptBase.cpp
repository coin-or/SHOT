/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverIpoptBase.h"

E_NLPSolutionStatus NLPSolverIpoptBase::solveProblemInstance()
{
    env->output->outputInfo("     Starting solution of Ipopt problem.");

    auto timeLimit = env->settings->getDoubleSetting("TimeLimit", "Termination") - env->process->getElapsedTime("Total");

    E_NLPSolutionStatus status;

    if (IpoptNLPSolver != NULL)
    {
        delete IpoptNLPSolver;
        IpoptNLPSolver = NULL;
    }

    IpoptNLPSolver = new IpoptSolver();

    try
    {
        IpoptNLPSolver->osinstance = NLPProblem->getProblemInstance();
        updateSettings();

        std::string solStatus;

        try
        {
            IpoptNLPSolver->solve();
            std::cout << "\e[A"; // Fix for removing unwanted output
            std::cout << "\e[A";
            solStatus = IpoptNLPSolver->osresult->getSolutionStatusType(0);
        }
        catch (ErrorClass e)
        {
            env->output->outputError("     Error when solving NLP problem with Ipopt", e.errormsg);
            solStatus == "other";
        }

        if (solStatus == "globallyOptimal")
        {
            env->output->outputWarning("     Global solution found to relaxed problem with Ipopt.");
            status = E_NLPSolutionStatus::Optimal;
        }

        if (solStatus == "locallyOptimal")
        {
            env->output->outputWarning("     Local solution found to relaxed problem with Ipopt.");
            status = E_NLPSolutionStatus::Optimal;
        }

        if (solStatus == "optimal")
        {
            env->output->outputWarning("     Optimal solution found to relaxed problem with Ipopt.");
            status = E_NLPSolutionStatus::Optimal;
        }

        if (solStatus == "bestSoFar")
        {
            env->output->outputWarning("     Feasible solution found to relaxed problem with Ipopt.");
            status = E_NLPSolutionStatus::Feasible;
        }

        if (solStatus == "stoppedByLimit")
        {
            env->output->outputWarning(
                "     No solution found to problem with Ipopt: Time or iteration limit exceeded.");
            status = E_NLPSolutionStatus::IterationLimit;
        }

        if (solStatus == "infeasible")
        {
            env->output->outputWarning(
                "     No solution found to problem with Ipopt: Infeasible problem detected.");
            status = E_NLPSolutionStatus::Infeasible;
        }

        if (solStatus == "unsure")
        {
            env->output->outputError(
                "     No solution found to problem with Ipopt, solution code: unsure.");
            status = E_NLPSolutionStatus::Infeasible;
        }

        if (solStatus == "other")
        {
            env->output->outputWarning(
                "     No solution found to problem with Ipopt, solution code: other.");
            status = E_NLPSolutionStatus::Infeasible;
        }
    }
    catch (std::exception &e)
    {
        if (IpoptNLPSolver != NULL)
        {
            delete IpoptNLPSolver;
            IpoptNLPSolver = NULL;
        }

        env->output->outputError("     Error when solving relaxed problem with Ipopt!", e.what());
        status = E_NLPSolutionStatus::Error;
    }
    catch (...)
    {
        if (IpoptNLPSolver != NULL)
        {
            delete IpoptNLPSolver;
            IpoptNLPSolver = NULL;
        }

        env->output->outputError("     Unspecified error when solving relaxed problem with Ipopt!");
        status = E_NLPSolutionStatus::Error;
    }

    env->output->outputInfo("     Finished solution of Ipopt problem.");

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

void NLPSolverIpoptBase::setStartingPoint(std::vector<int> variableIndexes, std::vector<double> variableValues)
{
    startingPointVariableIndexes = variableIndexes;
    startingPointVariableValues = variableValues;

    int startingPointSize = startingPointVariableIndexes.size();

    if (startingPointSize == 0)
        return;

    auto lbs = NLPProblem->getVariableLowerBounds();
    auto ubs = NLPProblem->getVariableUpperBounds();

    env->output->outputInfo("     Adding starting points to Ipopt.");

    for (int k = 0; k < startingPointSize; k++)
    {
        int currVarIndex = startingPointVariableIndexes.at(k);
        auto currPt = startingPointVariableValues.at(k);

        auto currLB = lbs.at(currVarIndex);
        auto currUB = ubs.at(currVarIndex);

        // Nonlinear objective function variable are not used in the NLP problem for Ipopt
        if (NLPProblem->isObjectiveFunctionNonlinear() && currVarIndex == NLPProblem->getNonlinearObjectiveVariableIdx())
        {
            continue;
        }

        if (currPt > currUB)
        {
            env->output->outputWarning(
                "       Starting point value for variable " + std::to_string(currVarIndex) + " is larger than ub: " + UtilityFunctions::toString(currPt) + " > " + UtilityFunctions::toString(currUB) + "; resetting to ub.");
            if (currUB == 1 && currPt > 1 && currPt < 1.00001)
            {
                currPt = 1;
            }
            else
            {
                currPt = currUB;
            }
        }

        if (currPt < currLB)
        {
            env->output->outputWarning(
                "       Starting point value for variable " + std::to_string(currVarIndex) + " is smaller than lb: " + UtilityFunctions::toString(currPt) + " < " + UtilityFunctions::toString(currLB) + "; resetting to lb.");

            if (currLB == 0 && currPt < 0 && currPt > -0.00001)
            {
                currPt = 0;
            }
            else
            {
                currPt = currLB;
            }
        }

        osOption->setAnotherInitVarValue(currVarIndex, currPt);

        env->output->outputInfo(
            "       Starting point value for " + std::to_string(currVarIndex) + " set: " + UtilityFunctions::toString(currLB) + " < " + UtilityFunctions::toString(currPt) + " < " + UtilityFunctions::toString(currUB));
    }

    osOption->setAnotherSolverOption("warm_start_init_point", "yes", "ipopt", "", "string", "");

    env->output->outputInfo("     All starting points set.");
}

void NLPSolverIpoptBase::clearStartingPoint()
{
    startingPointVariableIndexes.clear();
    startingPointVariableValues.clear();
    setInitialSettings();
    setSolverSpecificInitialSettings();
}

bool NLPSolverIpoptBase::isObjectiveFunctionNonlinear()
{
    return (NLPProblem->isObjectiveFunctionNonlinear());
}

int NLPSolverIpoptBase::getObjectiveFunctionVariableIndex()
{
    return (NLPProblem->getNonlinearObjectiveVariableIdx());
}

std::vector<double> NLPSolverIpoptBase::getCurrentVariableLowerBounds()
{
    return (NLPProblem->getVariableLowerBounds());
}

std::vector<double> NLPSolverIpoptBase::getCurrentVariableUpperBounds()
{
    return (NLPProblem->getVariableUpperBounds());
}

NLPSolverIpoptBase::NLPSolverIpoptBase()
{
    IpoptNLPSolver = NULL;
}

NLPSolverIpoptBase::~NLPSolverIpoptBase()
{
    if (IpoptNLPSolver != NULL)
    {
        delete IpoptNLPSolver;
        IpoptNLPSolver = NULL;
    }

    delete osOption;
}

std::vector<double> NLPSolverIpoptBase::getSolution()
{
    int numVar = NLPProblem->getNumberOfVariables();
    std::vector<double> tmpPoint(numVar);

    for (int i = 0; i < numVar; i++)
    {
        tmpPoint.at(i) = NLPSolverIpoptBase::getSolution(i);
    }

    return (tmpPoint);
}

void NLPSolverIpoptBase::setInitialSettings()
{
    osOption = new OSOption();

    std::string IpoptSolver = "";

    // Sets the linear solver used
    switch (static_cast<ES_IpoptSolver>(env->settings->getIntSetting("Ipopt.LinearSolver", "Subsolver")))
    {
    case (ES_IpoptSolver::ma27):
        IpoptSolver = "ma27";
        break;

    case (ES_IpoptSolver::ma57):
        IpoptSolver = "ma57";
        break;

    case (ES_IpoptSolver::ma86):
        IpoptSolver = "ma86";
        break;

    case (ES_IpoptSolver::ma97):
        IpoptSolver = "ma97";
        break;

    case (ES_IpoptSolver::mumps):
        IpoptSolver = "mumps";
        break;
    default:
        IpoptSolver = "mumps";
    }

    osOption->setAnotherSolverOption("linear_solver", IpoptSolver, "ipopt", "", "string", "");

    switch (env->settings->getIntSetting("Console.LogLevel", "Output") + 1)
    {
    case ENUM_OUTPUT_LEVEL_error:
        osOption->setAnotherSolverOption("print_level", "0", "ipopt", "", "integer", "");
        break;
    case ENUM_OUTPUT_LEVEL_summary:
        osOption->setAnotherSolverOption("print_level", "2", "ipopt", "", "integer", "");
        break;
    case ENUM_OUTPUT_LEVEL_warning:
        osOption->setAnotherSolverOption("print_level", "2", "ipopt", "", "integer", "");
        break;
    case ENUM_OUTPUT_LEVEL_info:
        osOption->setAnotherSolverOption("print_level", "5", "ipopt", "", "integer", "");
        break;
    case ENUM_OUTPUT_LEVEL_debug:
        osOption->setAnotherSolverOption("print_level", "8", "ipopt", "", "integer", "");
        break;
    case ENUM_OUTPUT_LEVEL_trace:
        osOption->setAnotherSolverOption("print_level", "10", "ipopt", "", "integer", "");
        break;
    case ENUM_OUTPUT_LEVEL_detailed_trace:
        osOption->setAnotherSolverOption("print_level", "12", "ipopt", "", "integer", "");
        break;
    default:
        break;
    }

    // Suppress copyright message
    if (env->settings->getIntSetting("Console.LogLevel", "Output") < 3)
    {
        osOption->setAnotherSolverOption("sb", "yes", "ipopt", "", "string", "");
    }

    //osOption->setAnotherSolverOption("fixed_variable_treatment", "make_constraint", "ipopt", "", "string", "");

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

void NLPSolverIpoptBase::fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues)
{

    fixedVariableIndexes = variableIndexes;
    fixedVariableValues = variableValues;

    int size = fixedVariableIndexes.size();

    if (size == 0)
        return;

    auto lbs = NLPProblem->getVariableLowerBounds();
    auto ubs = NLPProblem->getVariableUpperBounds();

    if (lowerBoundsBeforeFix.size() > 0 || upperBoundsBeforeFix.size() > 0)
    {
        env->output->outputWarning("     Old variable fixes remain for Ipopt solver, resetting!");
        lowerBoundsBeforeFix.clear();
        upperBoundsBeforeFix.clear();
    }

    env->output->outputInfo("     Defining fixed variables in Ipopt.");

    for (int k = 0; k < size; k++)
    {
        int currVarIndex = fixedVariableIndexes.at(k);
        auto currPt = fixedVariableValues.at(k);

        auto currLB = lbs.at(currVarIndex);
        auto currUB = ubs.at(currVarIndex);

        lowerBoundsBeforeFix.push_back(currLB);
        upperBoundsBeforeFix.push_back(currUB);

        currPt = UtilityFunctions::round(currPt);

        // Fix for negative zero
        if (currPt >= (0.0 - std::numeric_limits<double>::epsilon()) && currPt <= 0.0 + std::numeric_limits<double>::epsilon())
        {
            currPt = 0.0;
        }

        if (currPt > currUB)
        {
            env->output->outputWarning(
                "       Fixed value for variable " + std::to_string(currVarIndex) + " is larger than ub: " + UtilityFunctions::toString(currPt) + " > " + std::to_string(currUB));
            if (currUB == 1 && currPt > 1 && currPt < 1.00001)
            {
                currPt = 1;
            }
            else
            {
                continue;
            }
        }
        else if (currPt < currLB)
        {
            env->output->outputWarning(
                "       Fixed value for variable " + std::to_string(currVarIndex) + " is smaller than lb: " + UtilityFunctions::toString(currPt) + " < " + std::to_string(currLB));

            if (currLB == 0 && currPt < 0 && currPt > -0.00001)
            {
                currPt = 0;
            }
            else
            {
                continue;
            }
        }

        env->output->outputInfo(
            "       Setting fixed value for variable " + std::to_string(currVarIndex) + ": " + UtilityFunctions::toString(currLB) + " <= " + UtilityFunctions::toString(currPt) + " <= " + UtilityFunctions::toString(currUB));

        NLPProblem->fixVariable(currVarIndex, currPt);
    }
    env->output->outputInfo("     All fixed variables defined.");
}

void NLPSolverIpoptBase::unfixVariables()
{
    env->output->outputInfo("     Starting reset of fixed variables in Ipopt.");

    for (int k = 0; k < fixedVariableIndexes.size(); k++)
    {
        int currVarIndex = fixedVariableIndexes.at(k);
        double newLB = lowerBoundsBeforeFix.at(k);
        double newUB = upperBoundsBeforeFix.at(k);

        NLPProblem->setVariableLowerBound(currVarIndex, newLB);
        NLPProblem->setVariableUpperBound(currVarIndex, newUB);
        env->output->outputInfo(
            "       Resetting initial bounds for variable " + std::to_string(currVarIndex) + " lb = " + UtilityFunctions::toString(newLB) + " ub = " + UtilityFunctions::toString(newUB));
    }

    fixedVariableIndexes.clear();
    fixedVariableValues.clear();
    lowerBoundsBeforeFix.clear();
    upperBoundsBeforeFix.clear();

    setInitialSettings(); // Must initialize it again since the class contains fixed variable bounds and starting points

    env->output->outputInfo("     Reset of fixed variables in Ipopt completed.");
}

void NLPSolverIpoptBase::updateSettings()
{
    IpoptNLPSolver->osoption = osOption;
    IpoptNLPSolver->osol = osolwriter->writeOSoL(osOption);
}

void NLPSolverIpoptBase::saveOptionsToFile(std::string fileName)
{
    osolwriter->m_bWhiteSpace = false;

    std::stringstream ss;
    ss << osolwriter->writeOSoL(osOption);

    UtilityFunctions::writeStringToFile(fileName, ss.str());
}
