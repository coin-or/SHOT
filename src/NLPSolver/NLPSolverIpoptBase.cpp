#include "NLPSolverIpoptBase.h"
#include "../OptProblems/OptProblem.h"

E_NLPSolutionStatus NLPSolverIpoptBase::solveProblemInstance()
{
	//

	ProcessInfo::getInstance().outputInfo("     Starting solution of Ipopt problem.");

	auto timeLimit = Settings::getInstance().getDoubleSetting("TimeLimit", "Termination")
			- ProcessInfo::getInstance().getElapsedTime("Total");

	E_NLPSolutionStatus status;

	NLPSolver = new NLPIpoptSolver();

	try
	{
		NLPSolver->osinstance = NLPProblem->getProblemInstance();
		updateSettings();

		std::string solStatus;

		try
		{
			NLPSolver->solve();
			std::cout << "\e[A"; // Fix for removing unwanted output
			std::cout << "\e[A";
			solStatus = NLPSolver->osresult->getSolutionStatusType(0);
		}
		catch (ErrorClass e)
		{
			ProcessInfo::getInstance().outputError("     Error when solving NLP problem with Ipopt", e.errormsg);
			solStatus == "other";
		}

		if (solStatus == "globallyOptimal")
		{
			ProcessInfo::getInstance().outputWarning("     Global solution found to relaxed problem with Ipopt.");
			status = E_NLPSolutionStatus::Optimal;
		}

		if (solStatus == "locallyOptimal")
		{
			ProcessInfo::getInstance().outputWarning("     Local solution found to relaxed problem with Ipopt.");
			status = E_NLPSolutionStatus::Optimal;
		}

		if (solStatus == "optimal")
		{
			ProcessInfo::getInstance().outputWarning("     Optimal solution found to relaxed problem with Ipopt.");
			status = E_NLPSolutionStatus::Optimal;
		}

		if (solStatus == "bestSoFar")
		{
			ProcessInfo::getInstance().outputWarning("     Feasible solution found to relaxed problem with Ipopt.");
			status = E_NLPSolutionStatus::Feasible;
		}

		if (solStatus == "stoppedByLimit")
		{
			ProcessInfo::getInstance().outputWarning(
					"     No solution found to problem with Ipopt: Time or iteration limit exceeded.");
			status = E_NLPSolutionStatus::IterationLimit;
		}

		if (solStatus == "infeasible")
		{
			ProcessInfo::getInstance().outputWarning(
					"     No solution found to problem with Ipopt: Infeasible problem detected.");
			status = E_NLPSolutionStatus::Infeasible;
		}

		if (solStatus == "unsure")
		{
			ProcessInfo::getInstance().outputError(
					"     No solution found to problem with Ipopt, solution code: unsure.");
			status = E_NLPSolutionStatus::Infeasible;
		}

		if (solStatus == "other")
		{
			ProcessInfo::getInstance().outputWarning(
					"     No solution found to problem with Ipopt, solution code: other.");
			status = E_NLPSolutionStatus::Infeasible;
		}
	}
	catch (std::exception &e)
	{
		ProcessInfo::getInstance().outputError("     Error when solving relaxed problem with Ipopt!", e.what());
		status = E_NLPSolutionStatus::Error;
	}
	catch (...)
	{
		ProcessInfo::getInstance().outputError("     Unspecified error when solving relaxed problem with Ipopt!");
		status = E_NLPSolutionStatus::Error;
	}

	ProcessInfo::getInstance().outputInfo("     Finished solution of Ipopt problem.");

	return (status);
}

double NLPSolverIpoptBase::getSolution(int i)
{
	double value = NLPSolver->osresult->getVarValue(0, i);
	return (value);
}

double NLPSolverIpoptBase::getObjectiveValue()
{
	double value = NLPSolver->osresult->getObjValue(0, 0);
	return (value);
}

void NLPSolverIpoptBase::setStartingPoint(std::vector<int> variableIndexes, std::vector<double> variableValues)
{
	startingPointVariableIndexes = variableIndexes;
	startingPointVariableValues = variableValues;

	int startingPointSize = startingPointVariableIndexes.size();

	if (startingPointSize == 0) return;

	auto lbs = NLPProblem->getVariableLowerBounds();
	auto ubs = NLPProblem->getVariableUpperBounds();

	ProcessInfo::getInstance().outputInfo("     Adding starting points to Ipopt.");

	for (int k = 0; k < startingPointSize; k++)
	{
		int currVarIndex = startingPointVariableIndexes.at(k);
		auto currPt = startingPointVariableValues.at(k);

		auto currLB = lbs.at(currVarIndex);
		auto currUB = ubs.at(currVarIndex);

		// Nonlinear objective function variable are not used in the NLP problem for Ipopt
		if (NLPProblem->isObjectiveFunctionNonlinear()
				&& currVarIndex == NLPProblem->getNonlinearObjectiveVariableIdx())
		{
			continue;
		}

		if (currPt > currUB)
		{
			ProcessInfo::getInstance().outputWarning(
					"       Starting point value for variable " + to_string(currVarIndex) + " is larger than ub: "
							+ UtilityFunctions::toString(currPt) + " > " + UtilityFunctions::toString(currUB)
							+ "; resetting to ub.");
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
			ProcessInfo::getInstance().outputWarning(
					"       Starting point value for variable " + to_string(currVarIndex) + " is smaller than lb: "
							+ UtilityFunctions::toString(currPt) + " < " + UtilityFunctions::toString(currLB)
							+ "; resetting to lb.");

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

		ProcessInfo::getInstance().outputInfo(
				"       Starting point value for " + to_string(currVarIndex) + " set: "
						+ UtilityFunctions::toString(currLB) + " < " + UtilityFunctions::toString(currPt) + " < "
						+ UtilityFunctions::toString(currUB));
	}

	osOption->setAnotherSolverOption("warm_start_init_point", "yes", "ipopt", "", "string", "");

	ProcessInfo::getInstance().outputInfo("     All starting points set.");
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
	//

}

NLPSolverIpoptBase::~NLPSolverIpoptBase()
{
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
// Sets the linear solver used

	osOption = new OSOption();

	std::string IpoptSolver = "";

	if (Settings::getInstance().getIntSetting("Ipopt.LinearSolver", "Subsolver") == static_cast<int>(ES_IpoptSolver::ma27))
	{
		IpoptSolver = "ma27";
	}
	else if (Settings::getInstance().getIntSetting("Ipopt.LinearSolver", "Subsolver") == static_cast<int>(ES_IpoptSolver::ma57))
	{
		IpoptSolver = "ma57";
	}
	else if (Settings::getInstance().getIntSetting("Ipopt.LinearSolver", "Subsolver") == static_cast<int>(ES_IpoptSolver::ma86))
	{
		IpoptSolver = "ma86";
	}
	else if (Settings::getInstance().getIntSetting("Ipopt.LinearSolver", "Subsolver") == static_cast<int>(ES_IpoptSolver::ma97))
	{
		IpoptSolver = "ma97";
	}
	else if (Settings::getInstance().getIntSetting("Ipopt.LinearSolver", "Subsolver") == static_cast<int>(ES_IpoptSolver::mumps))
	{
		IpoptSolver = "mumps";
	}
	else
	{
		IpoptSolver = "ma57";
	}

	osOption->setAnotherSolverOption("linear_solver", IpoptSolver, "ipopt", "", "string", "");

	switch (Settings::getInstance().getIntSetting("Console.LogLevel", "Output") + 1)
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
	if (Settings::getInstance().getIntSetting("Console.LogLevel", "Output") < 3)
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

	if (size == 0) return;

	auto lbs = NLPProblem->getVariableLowerBounds();
	auto ubs = NLPProblem->getVariableUpperBounds();

	if (lowerBoundsBeforeFix.size() > 0 || upperBoundsBeforeFix.size() > 0)
	{
		ProcessInfo::getInstance().outputWarning("     Old variable fixes remain for Ipopt solver, resetting!");
		lowerBoundsBeforeFix.clear();
		upperBoundsBeforeFix.clear();
	}

	ProcessInfo::getInstance().outputInfo("     Defining fixed variables in Ipopt.");

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
		if (currPt >= (0.0 - std::numeric_limits<double>::epsilon())
				&& currPt <= 0.0 + std::numeric_limits<double>::epsilon())
		{
			currPt = 0.0;
		}

		if (currPt > currUB)
		{
			ProcessInfo::getInstance().outputWarning(
					"       Fixed value for variable " + to_string(currVarIndex) + " is larger than ub: "
							+ UtilityFunctions::toString(currPt) + " > " + to_string(currUB));
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
			ProcessInfo::getInstance().outputWarning(
					"       Fixed value for variable " + to_string(currVarIndex) + " is smaller than lb: "
							+ UtilityFunctions::toString(currPt) + " < " + to_string(currLB));

			if (currLB == 0 && currPt < 0 && currPt > -0.00001)
			{
				currPt = 0;
			}
			else
			{
				continue;
			}
		}

		ProcessInfo::getInstance().outputInfo(
				"       Setting fixed value for variable " + to_string(currVarIndex) + ": "
						+ UtilityFunctions::toString(currLB) + " <= " + UtilityFunctions::toString(currPt) + " <= "
						+ UtilityFunctions::toString(currUB));

		NLPProblem->fixVariable(currVarIndex, currPt);
	}
	ProcessInfo::getInstance().outputInfo("     All fixed variables defined.");
}

void NLPSolverIpoptBase::unfixVariables()
{
	ProcessInfo::getInstance().outputInfo("     Starting reset of fixed variables in Ipopt.");

	for (int k = 0; k < fixedVariableIndexes.size(); k++)
	{
		int currVarIndex = fixedVariableIndexes.at(k);
		double newLB = lowerBoundsBeforeFix.at(k);
		double newUB = upperBoundsBeforeFix.at(k);

		NLPProblem->setVariableLowerBound(currVarIndex, newLB);
		NLPProblem->setVariableUpperBound(currVarIndex, newUB);
		ProcessInfo::getInstance().outputInfo(
				"       Resetting initial bounds for variable " + to_string(currVarIndex) + " lb = "
						+ UtilityFunctions::toString(newLB) + " ub = " + UtilityFunctions::toString(newUB));

	}

	fixedVariableIndexes.clear();
	fixedVariableValues.clear();
	lowerBoundsBeforeFix.clear();
	upperBoundsBeforeFix.clear();

	setInitialSettings(); // Must initialize it again since the class contains fixed variable bounds and starting points

	ProcessInfo::getInstance().outputInfo("     Reset of fixed variables in Ipopt completed.");
}

void NLPSolverIpoptBase::updateSettings()
{
	NLPSolver->osoption = osOption;
	NLPSolver->osol = osolwriter->writeOSoL(osOption);
}

void NLPSolverIpoptBase::saveOptionsToFile(std::string fileName)
{
	fileUtil = new FileUtil();

	osolwriter->m_bWhiteSpace = false;

	stringstream ss;
	ss << osolwriter->writeOSoL(osOption);

	fileUtil->writeFileFromString(fileName, ss.str());
	delete fileUtil, osolwriter;
}
