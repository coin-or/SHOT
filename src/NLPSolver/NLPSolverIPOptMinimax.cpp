#include "NLPSolverIPOptMinimax.h"

//#include "OSnl2OS.h"

NLPSolverIPOptMinimax::NLPSolverIPOptMinimax()
{

	NLPSolver = new NLPIpoptSolver();
	osOption = new OSOption();
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	processInfo->startTimer("InteriorPointMinimax");

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
	else if (settings->getIntSetting("IPOptSolver", "NLP") == static_cast<int>(ES_IPOptSolver::multiple))
	{
		IPOptSolver = "ma27";
	}
	else
	{
		IPOptSolver = "ma57";
	}

	auto timeLimit = settings->getDoubleSetting("TimeLimit", "Algorithm") - processInfo->getElapsedTime("Total");
	auto constrTol = settings->getDoubleSetting("InteriorPointFeasEps", "NLP");
	//timeLimit = 1.0;

	osOption->setAnotherSolverOption("tol", "1E-8", "ipopt", "", "double", "");
	osOption->setAnotherSolverOption("max_iter", "10000", "ipopt", "", "integer", "");

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

	osOption->setAnotherSolverOption("sb", "yes", "ipopt", "", "string", "");
	osOption->setAnotherSolverOption("linear_solver", IPOptSolver, "ipopt", "", "string", "");
	osOption->setAnotherSolverOption("max_cpu_time", std::to_string(timeLimit), "ipopt", "", "double", "");
	osOption->setAnotherSolverOption("constr_viol_tol", std::to_string(constrTol), "ipopt", "", "double", "");

	/*
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

	//NLPSolver->NLPSolver = new IpoptSolver();
	//NLPSolver->NLPSolver->osoption = osOption;
	//ipoptSolver = new NLPIpoptSolver();
	//NLPSolver->nlp = ipoptSolver;
	OSoLWriter *osolwriter = NULL;
	osolwriter = new OSoLWriter();

	//(options = osolreader->readOSoL(osol);

	NLPSolver->osoption = osOption;
	NLPSolver->osol = osolwriter->writeOSoL(osOption);
	//NLPSolver->setSolverOptions();
	isPointValueCached = false;

	processInfo->stopTimer("InteriorPointMinimax");
}

NLPSolverIPOptMinimax::~NLPSolverIPOptMinimax()
{
	delete osOption;
	delete NLPProblem;
	//delete NLPSolver;
}

bool NLPSolverIPOptMinimax::createProblem(OSInstance * origInstance)
{
	processInfo->startTimer("InteriorPointMinimax");
	NLPProblem = new OptProblemNLPMinimax();
	NLPProblem->reformulate(origInstance);
	NLPSolver->osinstance = NLPProblem->getProblemInstance();

	processInfo->outputDebug("NLP problem created");

	processInfo->stopTimer("InteriorPointMinimax");
	return true;
}

bool NLPSolverIPOptMinimax::solveProblem()
{
	processInfo->startTimer("InteriorPointMinimax");
	bool useMultiple = (settings->getIntSetting("IPOptSolver", "NLP") == static_cast<int>(ES_IPOptSolver::multiple));

	OSoLWriter *osolwriter = NULL;
	osolwriter = new OSoLWriter();

	std::vector<std::vector<double>> tmpPoints;

	//NLPSolver->bSetSolverOptions = false;

	for (int k = 0; k < 5; k++)
	{
		std::string IPOptSolver = "";

		NLPSolver->osoption = osOption;

		if (useMultiple)
		{
			if (k == 0)
			{
				IPOptSolver = "ma27";
			}
			else if (k == 1)
			{
				IPOptSolver = "ma57";
			}
			else if (k == 2)
			{
				IPOptSolver = "ma86";
			}
			else if (k == 3)
			{
				IPOptSolver = "ma97";
			}
			else if (k == 4)
			{
				IPOptSolver = "mumps";
			}
			else
			{
				IPOptSolver = "ma57";
			}

			auto timeLimit = settings->getDoubleSetting("TimeLimit", "Algorithm")
					- processInfo->getElapsedTime("Total");
			auto constrTol = settings->getDoubleSetting("InteriorPointFeasEps", "NLP");
			//timeLimit = 1.0;

			osOption->setAnotherSolverOption("tol", "1E-8", "ipopt", "", "double", "");
			osOption->setAnotherSolverOption("max_iter", "10000", "ipopt", "", "integer", "");
			osOption->setAnotherSolverOption("sb", "yes", "ipopt", "", "string", "");
			//osOption->setAnotherSolverOption("linear_solver", IPOptSolver, "ipopt", "", "string", "");
			osOption->setAnotherSolverOption("max_cpu_time", std::to_string(timeLimit), "ipopt", "", "double", "");
			osOption->setAnotherSolverOption("constr_viol_tol", std::to_string(constrTol), "ipopt", "", "double", "");

			osOption->setAnotherSolverOption("linear_solver", IPOptSolver, "ipopt", "", "string", "");

			//NLPSolver->bSetSolverOptions = false;

			// To force change of options.
			NLPSolver->bSetSolverOptions = false;
		}

		//NLPSolver->osol = osolwriter->writeOSoL(osOption);
		NLPSolver->bSetSolverOptions = false;
		try
		{
			NLPSolver->solve();
			processInfo->numNLPProbsSolved++;
			std::cout << "\e[A";
			std::cout << "\e[A";
		}
		catch (...)
		{
			processInfo->outputError("Error when solving IPOpt minimax problem!");
			continue;
			//return false;
		}

		std::string solStatus = NLPSolver->osresult->getSolutionStatusType(0);

		if (solStatus == "infeasible" || NLPSolver->osresult->getSolutionNumber() == 0)
		{
			processInfo->outputError("No solution found to minimax problem with Ipopt. Solution status: " + solStatus);

			processInfo->stopTimer("InteriorPointMinimax");
			return false;
		}

		processInfo->outputInfo("Solution found to minimax problem with Ipopt. Solution status:" + solStatus);

		int numVar = NLPProblem->getNumberOfVariables();
		std::vector<double> tmpPoint(numVar);

		for (int i = 0; i < numVar; i++)
		{
			tmpPoint.at(i) = NLPSolver->osresult->getVarValue(0, i);
		}

		auto objval = NLPSolver->osresult->getObjValue(0, 0);

		if (objval > settings->getDoubleSetting("InteriorPointFeasEps", "NLP"))
		{
			processInfo->outputError(
					"NLP point from minimax problem not valid!\nObjective value is" + to_string(objval) + " > "
							+ to_string(settings->getDoubleSetting("InteriorPointFeasEps", "NLP")));

			processInfo->stopTimer("InteriorPointMinimax");
			return false;
		}

		processInfo->outputInfo(
				"NLP point from minimax problem valid!\nObjective value is" + to_string(objval) + " < "
						+ to_string(settings->getDoubleSetting("InteriorPointFeasEps", "NLP")));

		tmpPoint.pop_back();

		if (processInfo->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
		{
			tmpPoint.pop_back();
		}

		tmpPoints.push_back(tmpPoint);

		if (settings->getBoolSetting("Debug", "SHOTSolver"))
		{
			auto tmpVars = NLPProblem->getVariableNames();
			std::string filename = settings->getStringSetting("DebugPath", "SHOTSolver") + "/nlppoint_minimax"
					+ std::to_string(k) + ".txt";
			UtilityFunctions::saveVariablePointVectorToFile(tmpPoint, tmpVars, filename);
		}

		if (!useMultiple) break;
	}

	if (tmpPoints.size() == 0)
	{
		processInfo->stopTimer("InteriorPointMinimax");
		return false;
	}

	auto centerPoint = UtilityFunctions::calculateCenterPoint(tmpPoints);

	std::vector<std::vector<double>> selPts;

	auto tmpNorms = UtilityFunctions::L2Norms(tmpPoints, centerPoint);
	IndexValuePair valpair;

	for (int i = 0; i < tmpNorms.size(); i++)
	{
		tmpNorms.at(i) = round(1000.0 * tmpNorms.at(i)) / 1000.0;
	}

	std::vector<double> usedNorms;

	for (int i = 0; i < tmpNorms.size(); i++)
	{
		if (std::find(usedNorms.begin(), usedNorms.end(), tmpNorms.at(i)) == usedNorms.end())
		{
			selPts.push_back(tmpPoints.at(i));
			usedNorms.push_back(tmpNorms.at(i));
		}
	}

	/*auto biggest = std::max_element(std::begin(tmpNorms), std::end(tmpNorms));
	 valpair.idx = std::distance(std::begin(tmpNorms), biggest);
	 valpair.value = *biggest;*/

	//selPts.push_back(tmpPoints.at(valpair.idx));
	for (int i = 0; i < selPts.size(); i++)
	{

		//auto centerPoint = tmpPoints.at(0);
		auto tmpIP = new InteriorPoint();
		tmpIP->NLPSolver = static_cast<int>(ES_NLPSolver::IPOptMiniMax);
		tmpIP->point = selPts.at(i);

		auto tmpPoint(tmpIP->point);
		//auto tmpPoint2(tmpPoints.at(0));
		//tmpPoint2.pop_back();

		//UtilityFunctions::displayVector(tmpPoint);
		//UtilityFunctions::displayVector(tmpPoint2);
		auto maxDev = processInfo->originalProblem->getMostDeviatingConstraint(tmpPoint);

		tmpPoint.pop_back();
		//auto maxDev2 = NLPProblem->getMostDeviatingConstraint(tmpPoint2).value;
		//std::cout << "Error " << maxDev2 << std::endl;

		tmpIP->maxDevatingConstraint = maxDev;
		processInfo->interiorPts.push_back(*tmpIP);

		if (settings->getBoolSetting("Debug", "SHOTSolver"))
		{
			auto tmpVars = NLPProblem->getVariableNames();
			tmpVars.push_back("mu");
			std::string filename = settings->getStringSetting("DebugPath", "SHOTSolver") + "/nlppoint_minimax"
					+ std::to_string(i) + ".txt";
			UtilityFunctions::saveVariablePointVectorToFile(tmpPoint, tmpVars, filename);
		}
	}

	processInfo->outputInfo("Number of NLP points: " + to_string(processInfo->interiorPts.size()));

	processInfo->stopTimer("InteriorPointMinimax");
	return (true);
}

void NLPSolverIPOptMinimax::saveProblemModelToFile(std::string fileName)
{
	NLPProblem->saveProblemModelToFile(fileName);
}
