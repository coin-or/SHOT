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

    setInitialSettings(); // Must initialize it again since the class contains fixed variable bounds and starting points

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