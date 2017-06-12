#include "NLPSolverIPOptBase.h"
#include "../OptProblems/OptProblem.h"

E_NLPSolutionStatus NLPSolverIPOptBase::solveProblemInstance()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->outputInfo("     Starting solution of Ipopt problem.");

	auto timeLimit = settings->getDoubleSetting("TimeLimit", "Algorithm") - processInfo->getElapsedTime("Total");

	E_NLPSolutionStatus status;

	NLPSolver = new NLPIpoptSolver();

	try
	{
		NLPSolver->osinstance = NLPProblem->getProblemInstance();
		updateSettings();

		NLPSolver->solve();

		std::cout << "\e[A"; // Fix for removing unwanted output
		std::cout << "\e[A";

		std::string solStatus = NLPSolver->osresult->getSolutionStatusType(0);

		if (solStatus == "globallyOptimal")
		{
			processInfo->outputWarning("     Global solution found to relaxed problem with Ipopt.");
			status = E_NLPSolutionStatus::Optimal;
		}

		if (solStatus == "locallyOptimal")
		{
			processInfo->outputWarning("     Local solution found to relaxed problem with Ipopt.");
			status = E_NLPSolutionStatus::Optimal;
		}

		if (solStatus == "optimal")
		{
			processInfo->outputWarning("     Optimal solution found to relaxed problem with Ipopt.");
			status = E_NLPSolutionStatus::Optimal;
		}

		if (solStatus == "bestSoFar")
		{
			processInfo->outputWarning("     Feasible solution found to relaxed problem with Ipopt.");
			status = E_NLPSolutionStatus::Feasible;
		}

		if (solStatus == "stoppedByLimit")
		{
			processInfo->outputWarning(
					"     No solution found to problem with Ipopt: Time or iteration limit exceeded.");
			status = E_NLPSolutionStatus::IterationLimit;
		}

		if (solStatus == "infeasible")
		{
			processInfo->outputWarning("     No solution found to problem with Ipopt: Infeasible problem detected.");
			status = E_NLPSolutionStatus::Infeasible;
		}

		if (solStatus == "unsure")
		{
			processInfo->outputError("     No solution found to problem with Ipopt, solution code: unsure.");
			status = E_NLPSolutionStatus::Infeasible;
		}

		if (solStatus == "other")
		{
			processInfo->outputWarning("     No solution found to problem with Ipopt, solution code: other.");
			status = E_NLPSolutionStatus::Infeasible;
		}
	}
	catch (std::exception &e)
	{
		processInfo->outputError("     Error when solving relaxed problem with Ipopt!", e.what());
		status = E_NLPSolutionStatus::Error;
	}
	catch (...)
	{
		processInfo->outputError("     Unspecified error when solving relaxed problem with Ipopt!");
		status = E_NLPSolutionStatus::Error;
	}

	processInfo->outputInfo("     Finished solution of Ipopt problem.");

	return (status);
}

double NLPSolverIPOptBase::getSolution(int i)
{
	double value = NLPSolver->osresult->getVarValue(0, i);
	return (value);
}

double NLPSolverIPOptBase::getObjectiveValue()
{
	double value = NLPSolver->osresult->getObjValue(0, 0);
	return (value);
}

void NLPSolverIPOptBase::setStartingPoint(std::vector<int> variableIndexes, std::vector<double> variableValues)
{
	startingPointVariableIndexes = variableIndexes;
	startingPointVariableValues = variableValues;

	int startingPointSize = startingPointVariableIndexes.size();

	if (startingPointSize == 0) return;

	auto lbs = NLPProblem->getVariableLowerBounds();
	auto ubs = NLPProblem->getVariableUpperBounds();

	processInfo->outputInfo("     Adding starting points to Ipopt.");

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
			processInfo->outputWarning(
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
			processInfo->outputWarning(
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

		processInfo->outputInfo(
				"       Starting point value for " + to_string(currVarIndex) + " set: "
						+ UtilityFunctions::toString(currLB) + " < " + UtilityFunctions::toString(currPt) + " < "
						+ UtilityFunctions::toString(currUB));
	}

	osOption->setAnotherSolverOption("warm_start_init_point", "yes", "ipopt", "", "string", "");

	processInfo->outputInfo("     All starting points set.");
}

void NLPSolverIPOptBase::clearStartingPoint()
{
	startingPointVariableIndexes.clear();
	startingPointVariableValues.clear();
	setInitialSettings();
	setSolverSpecificInitialSettings();
}

bool NLPSolverIPOptBase::isObjectiveFunctionNonlinear()
{
	return (NLPProblem->isObjectiveFunctionNonlinear());
}

int NLPSolverIPOptBase::getObjectiveFunctionVariableIndex()
{
	return (NLPProblem->getNonlinearObjectiveVariableIdx());
}

std::vector<double> NLPSolverIPOptBase::getCurrentVariableLowerBounds()
{
	return (NLPProblem->getVariableLowerBounds());
}

std::vector<double> NLPSolverIPOptBase::getCurrentVariableUpperBounds()
{
	return (NLPProblem->getVariableUpperBounds());
}

NLPSolverIPOptBase::NLPSolverIPOptBase()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

NLPSolverIPOptBase::~NLPSolverIPOptBase()
{
}

std::vector<double> NLPSolverIPOptBase::getSolution()
{
	int numVar = NLPProblem->getNumberOfVariables();
	std::vector<double> tmpPoint(numVar);

	for (int i = 0; i < numVar; i++)
	{
		tmpPoint.at(i) = NLPSolverIPOptBase::getSolution(i);
	}

	return (tmpPoint);
}

void NLPSolverIPOptBase::setInitialSettings()
{
// Sets the linear solver used

	osOption = new OSOption();

	std::string IPOptSolver = "";

	if (settings->getIntSetting("IpoptSolver", "Ipopt") == static_cast<int>(ES_IPOptSolver::ma27))
	{
		IPOptSolver = "ma27";
	}
	else if (settings->getIntSetting("IpoptSolver", "Ipopt") == static_cast<int>(ES_IPOptSolver::ma57))
	{
		IPOptSolver = "ma57";
	}
	else if (settings->getIntSetting("IpoptSolver", "Ipopt") == static_cast<int>(ES_IPOptSolver::ma86))
	{
		IPOptSolver = "ma86";
	}
	else if (settings->getIntSetting("IpoptSolver", "Ipopt") == static_cast<int>(ES_IPOptSolver::ma97))
	{
		IPOptSolver = "ma97";
	}
	else if (settings->getIntSetting("IpoptSolver", "Ipopt") == static_cast<int>(ES_IPOptSolver::mumps))
	{
		IPOptSolver = "mumps";
	}
	else
	{
		IPOptSolver = "ma57";
	}

	osOption->setAnotherSolverOption("linear_solver", IPOptSolver, "ipopt", "", "string", "");

	switch (settings->getIntSetting("LogLevelConsole", "SHOTSolver") + 1)
	{
		case ENUM_OUTPUT_LEVEL_error:
			osOption->setAnotherSolverOption("print_level", "0", "ipopt", "", "integer", "");
			break;
		case ENUM_OUTPUT_LEVEL_summary:
			osOption->setAnotherSolverOption("print_level", "1", "ipopt", "", "integer", "");
			break;
		case ENUM_OUTPUT_LEVEL_warning:
			osOption->setAnotherSolverOption("print_level", "1", "ipopt", "", "integer", "");
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
	if (settings->getIntSetting("LogLevelConsole", "SHOTSolver") < 3)
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

void NLPSolverIPOptBase::fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues)
{

	fixedVariableIndexes = variableIndexes;
	fixedVariableValues = variableValues;

	int size = fixedVariableIndexes.size();

	if (size == 0) return;

	auto lbs = NLPProblem->getVariableLowerBounds();
	auto ubs = NLPProblem->getVariableUpperBounds();

	if (lowerBoundsBeforeFix.size() > 0 || upperBoundsBeforeFix.size() > 0)
	{
		processInfo->outputWarning("     Old variable fixes remain for Ipopt solver, resetting!");
		lowerBoundsBeforeFix.clear();
		upperBoundsBeforeFix.clear();
	}

	processInfo->outputInfo("     Defining fixed variables in Ipopt.");

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
			processInfo->outputWarning(
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
			processInfo->outputWarning(
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

		processInfo->outputInfo(
				"       Setting fixed value for variable " + to_string(currVarIndex) + ": "
						+ UtilityFunctions::toString(currLB) + " <= " + UtilityFunctions::toString(currPt) + " <= "
						+ UtilityFunctions::toString(currUB));

		NLPProblem->fixVariable(currVarIndex, currPt);
	}
	processInfo->outputInfo("     All fixed variables defined.");
}

void NLPSolverIPOptBase::unfixVariables()
{
	processInfo->outputInfo("     Starting reset of fixed variables in Ipopt.");

	for (int k = 0; k < fixedVariableIndexes.size(); k++)
	{
		int currVarIndex = fixedVariableIndexes.at(k);
		double newLB = lowerBoundsBeforeFix.at(k);
		double newUB = upperBoundsBeforeFix.at(k);

		NLPProblem->setVariableLowerBound(currVarIndex, newLB);
		NLPProblem->setVariableUpperBound(currVarIndex, newUB);
		processInfo->outputInfo(
				"       Resetting initial bounds for variable " + to_string(currVarIndex) + " lb = "
						+ UtilityFunctions::toString(newLB) + " ub = " + UtilityFunctions::toString(newUB));

	}

	fixedVariableIndexes.clear();
	fixedVariableValues.clear();
	lowerBoundsBeforeFix.clear();
	upperBoundsBeforeFix.clear();

	setInitialSettings(); // Must initialize it again since the class contains fixed variable bounds and starting points

	processInfo->outputInfo("     Reset of fixed variables in Ipopt completed.");
}

void NLPSolverIPOptBase::updateSettings()
{
	NLPSolver->osoption = osOption;
	NLPSolver->osol = osolwriter->writeOSoL(osOption);
}

void NLPSolverIPOptBase::saveOptionsToFile(std::string fileName)
{
	fileUtil = new FileUtil();

	osolwriter->m_bWhiteSpace = false;

	stringstream ss;
	ss << osolwriter->writeOSoL(osOption);

	fileUtil->writeFileFromString(fileName, ss.str());
	delete fileUtil, osolwriter;
}
