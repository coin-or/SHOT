#include "NLPSolverIPOptRelaxed.h"

NLPSolverIPOptRelaxed::NLPSolverIPOptRelaxed()
{

	NLPSolver = new NLPIpoptSolver();
	osOption = new OSOption();
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	processInfo->startTimer("InteriorPointRelaxed");

	std::string IPOptSolver = "";

	if (settings->getIntSetting("IPOptSolver", "NLP") == static_cast<int>(ES_IPOptSolver::ma27))
	{
		IPOptSolver = "ma27";
	}
	else if (settings->getIntSetting("IPOptSolver", "NLP") == static_cast<int>(ES_IPOptSolver::ma57))
	{
		IPOptSolver = "ma57";
	}
	else if (settings->getIntSetting("IPOptSolver", "NLP") == static_cast<int>(ES_IPOptSolver::ma86))
	{
		IPOptSolver = "ma86";
	}
	else if (settings->getIntSetting("IPOptSolver", "NLP") == static_cast<int>(ES_IPOptSolver::ma97))
	{
		IPOptSolver = "ma97";
	}
	else if (settings->getIntSetting("IPOptSolver", "NLP") == static_cast<int>(ES_IPOptSolver::mumps))
	{
		IPOptSolver = "mumps";
	}
	else
	{
		IPOptSolver = "mumps";
	}

	auto timeLimit = settings->getDoubleSetting("TimeLimit", "Algorithm") - processInfo->getElapsedTime("Total");
	auto constrTol = settings->getDoubleSetting("InteriorPointFeasEps", "NLP");

	//osOption->setAnotherSolverOption("tol", "1E-1", "ipopt", "", "double", "");
	osOption->setAnotherSolverOption("max_iter", "10000", "ipopt", "", "integer", "");
	switch (settings->getIntSetting("LogLevelConsole", "SHOTSolver") + 1)
	{
		case ENUM_OUTPUT_LEVEL_error:
			osOption->setAnotherSolverOption("print_level", "0", "ipopt", "", "integer", "");
			break;
		case ENUM_OUTPUT_LEVEL_summary:
			osOption->setAnotherSolverOption("print_level", "0", "ipopt", "", "integer", "");
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

	osOption->setAnotherSolverOption("sb", "yes", "ipopt", "", "string", "");
	osOption->setAnotherSolverOption("linear_solver", IPOptSolver, "ipopt", "", "string", "");
	osOption->setAnotherSolverOption("max_cpu_time", std::to_string(timeLimit), "ipopt", "", "double", "");
	//osOption->setAnotherSolverOption("constr_viol_tol", "-0.1", "ipopt", "", "double", "");

	/*osOption->setAnotherSolverOption("barrier_tol_factor", "10", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("expect_infeasible_problem", "no", "ipopt", "", "string", "");
	 osOption->setAnotherSolverOption("mu_init", "0.1", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("mu_max", "100000", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("mu_min", "1E-11", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("resto_failure_feasibility_threshold", "0", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("resto_penalty_parameter", "1000", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("resto_proximity_weight", "1", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("gamma_theta", "1E-5", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("required_infeasibility_reduction", "0.5", "ipopt", "", "double", "");

	 osOption->setAnotherSolverOption("constr_viol_tol", "0.0000000001", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("dual_inf_tol", "1", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("compl_inf_tol", "0.0001", "ipopt", "", "double", "");
	 osOption->setAnotherSolverOption("mu_strategy", "adaptive", "ipopt", "", "string", "");
	 osOption->setAnotherSolverOption("mu_oracle", "quality-function", "ipopt", "", "string", "");
	 osOption->setAnotherSolverOption("check_derivatives_for_naninf", "yes", "ipopt", "", "string", "");*/

	//ipoptSolver = new NLPIpoptSolver();
	//NLPSolver->nlp = ipoptSolver;
	OSoLWriter *osolwriter = NULL;
	osolwriter = new OSoLWriter();

	//(options = osolreader->readOSoL(osol);

	//NLPSolver->osoption = osOption;
	NLPSolver->osol = osolwriter->writeOSoL(osOption);
	isPointValueCached = false;

	processInfo->stopTimer("InteriorPointRelaxed");
}

NLPSolverIPOptRelaxed::~NLPSolverIPOptRelaxed()
{
	delete osOption;
	delete NLPProblem;
	//delete NLPSolver;
}

bool NLPSolverIPOptRelaxed::createProblem(OSInstance * origInstance)
{
	processInfo->startTimer("InteriorPointRelaxed");
	NLPProblem = new OptProblemNLPRelaxed();
	NLPProblem->reformulate(origInstance);
	NLPSolver->osinstance = NLPProblem->getProblemInstance();
	processInfo->outputDebug("NLP problem created");

	processInfo->stopTimer("InteriorPointRelaxed");
	return true;
}

bool NLPSolverIPOptRelaxed::solveProblem()
{
	processInfo->startTimer("InteriorPointRelaxed");
	try
	{
		NLPSolver->solve();
		processInfo->numNLPProbsSolved++;
		std::cout << "\e[A";
		std::cout << "\e[A";
	}
	catch (std::exception e)
	{
		processInfo->outputError("Error when solving relaxed problem with Ipopt!");
		processInfo->stopTimer("InteriorPointRelaxed");
		return false;
	}
	catch (...)
	{
		processInfo->outputError("Error when solving relaxed problem with Ipopt!");
		processInfo->stopTimer("InteriorPointRelaxed");
		return false;
	}

	std::string solStatus = NLPSolver->osresult->getSolutionStatusType(0);

	if (solStatus == "infeasible" || NLPSolver->osresult->getSolutionNumber() == 0)
	{
		processInfo->outputError("No solution found to relaxed problem with Ipopt. Solution status: " + solStatus);

		processInfo->stopTimer("InteriorPointRelaxed");
		return false;
	}

	processInfo->outputInfo("Solution found to relaxed problem with Ipopt. Solution status:" + solStatus);

	int numVar = NLPProblem->getNumberOfVariables();
	std::vector<double> tmpPoint(numVar);

	for (int i = 0; i < numVar; i++)
	{
		tmpPoint.at(i) = NLPSolver->osresult->getVarValue(0, i);
	}

	auto objval = NLPSolver->osresult->getObjValue(0, 0);

	if (NLPProblem->getNumberOfNonlinearConstraints() > 0)
	{
		auto maxDev = NLPProblem->getMostDeviatingConstraint(tmpPoint).value;
		if (maxDev > settings->getDoubleSetting("InteriorPointFeasEps", "NLP"))
		{
			processInfo->outputError(
					"NLP point from relaxed problem not valid!\nObjective value is" + to_string(objval) + " > "
							+ to_string(settings->getDoubleSetting("InteriorPointFeasEps", "NLP")));

			processInfo->stopTimer("InteriorPointRelaxed");
			return false;
		}

		processInfo->outputInfo(
				"NLP point from relaxed problem valid!\nObjective value is" + to_string(objval) + " < "
						+ to_string(settings->getDoubleSetting("InteriorPointFeasEps", "NLP")));
	}
	else
	{
		processInfo->outputInfo("NLP problem linearly constrained. Assuming point is valid.");
	}

	auto tmpIP = new InteriorPoint();
	tmpIP->NLPSolver = static_cast<int>(ES_NLPSolver::IPOptRelaxed);

	if (NLPProblem->isObjectiveFunctionNonlinear()
			&& processInfo->originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
	{
		tmpPoint.push_back(NLPSolver->osresult->getObjValue(0, 0));
	}

	auto maxDev = processInfo->originalProblem->getMostDeviatingConstraint(tmpPoint);

	tmpIP->point = tmpPoint;
	tmpIP->maxDevatingConstraint = maxDev;

	processInfo->interiorPts.push_back(*tmpIP);

	if (settings->getBoolSetting("Debug", "SHOTSolver"))
	{
		auto tmpVars = NLPProblem->getVariableNames();
		tmpVars.push_back("mu");
		std::string filename = settings->getStringSetting("DebugPath", "SHOTSolver") + "/nlppoint_relaxed.txt";
		UtilityFunctions::saveVariablePointVectorToFile(tmpPoint, tmpVars, filename);
	}

	processInfo->stopTimer("InteriorPointRelaxed");
	return true;
}

void NLPSolverIPOptRelaxed::saveProblemModelToFile(std::string fileName)
{
	NLPProblem->saveProblemModelToFile(fileName);
}
