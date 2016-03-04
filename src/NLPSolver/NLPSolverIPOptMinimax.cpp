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
	osOption->setAnotherSolverOption("print_level", "5", "ipopt", "", "integer", "");
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
	//NLPSolver->buildSolverInstance();
	processInfo->logger.message(2) << "NLP problem created" << CoinMessageEol;

	/*nl2os = new OSnl2OS();
	 nl2os->readNl("clay0203h.nl");
	 //nl2os->setOsol(oscommandline->osol);
	 nl2os->createOSObjects();
	 NLPSolver->osinstance = nl2os->osinstance;

	 NLPSolver->buildSolverInstance();

	 for (int i = 0; i < NLPSolver->osinstance->getVariableNumber(); i++)
	 {
	 NLPSolver->osinstance->instanceData->variables->var[i]->type = 'C';
	 }


	 NLPSolver->solve();*/

	// convert to the OS native format
	/*OSnl2OS *nl2osil = NULL;
	 nl2osil = new OSnl2OS();
	 nl2osil->readNl("osi\clay0203h.nl");
	 // create the first in-memory OSInstance
	 nl2osil->createOSObjects();
	 NLPSolver->osinstance = nl2osil->osinstance;
	 */
	//NLPProblem->printProblem();
	/*OSiLWriter *osilWriter = new OSiLWriter();

	 std::string osil = osilWriter->writeOSiL(NLPProblem->getProblemInstance());

	 OSiLReader *osilreader = new OSiLReader();
	 OSInstance *instance = NULL;

	 instance = osilreader->readOSiL(osil);

	 instance->bUseExpTreeForFunEval = true;*/

	//std::cout << NLPProblem->problemInstance->printModel();
	/*OSiLWriter *osilwriter = NULL;
	 osilwriter = new OSiLWriter();

	 std::string test = osilwriter->writeOSiL(NLPProblem->getProblemInstance());

	 FileUtil * f = new FileUtil();
	 */
	//f->writeFileFromString("test.osil", test);*/
	/*instance->bVariablesModified = true;

	 instance->getJacobianSparsityPattern();

	 //std::cout << instance->printModel();

	 NLPSolver->osinstance = instance;
	 NLPSolver->buildSolverInstance();*/

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
			osOption->setAnotherSolverOption("print_level", "5", "ipopt", "", "integer", "");
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
		}
		catch (...)
		{
			processInfo->logger.message(1) << "Error when solving IPOpt minimax problem!" << CoinMessageEol;
			continue;
			//return false;
		}

		std::string solStatus = NLPSolver->osresult->getSolutionStatusType(0);

		if (solStatus == "infeasible" || NLPSolver->osresult->getSolutionNumber() == 0)
		{
			processInfo->logger.message(1) << "No solution found to IPOpt minimax problem. Solution status: "
					<< solStatus << CoinMessageEol;

			processInfo->stopTimer("InteriorPointMinimax");
			return false;
		}

		processInfo->logger.message(1) << "Solution found to IPOpt minimax problem. Solution status: " << solStatus
				<< CoinMessageEol;

		int numVar = NLPProblem->getNumberOfVariables();
		std::vector<double> tmpPoint(numVar);

		for (int i = 0; i < numVar; i++)
		{
			tmpPoint.at(i) = NLPSolver->osresult->getVarValue(0, i);
		}

		auto objval = NLPSolver->osresult->getObjValue(0, 0);

		if (objval > settings->getDoubleSetting("InteriorPointFeasEps", "NLP"))
		{
			processInfo->logger.message(2) << "NLP point from IPOpt minimax problem not valid!" << CoinMessageEol;
			processInfo->logger.message(2) << "Objective value is " << objval << " > "
					<< settings->getDoubleSetting("InteriorPointFeasEps", "NLP") << CoinMessageEol;

			processInfo->stopTimer("InteriorPointMinimax");
			return false;
		}

		processInfo->logger.message(2) << "NLP point from IPOpt minimax problem valid." << CoinMessageEol;
		processInfo->logger.message(2) << "Objective value is " << objval << " < "
				<< settings->getDoubleSetting("InteriorPointFeasEps", "NLP") << CoinMessageEol;

		/*
		 if (NLPProblem->getNumberOfNonlinearConstraints() > 0)
		 {
		 //auto maxDev = NLPProblem->getMostDeviatingConstraint(tmpPoint).value;


		 if (objval > settings->getDoubleSetting("InteriorPointFeasEps", "NLP"))
		 {
		 processInfo->logger.message(2) << "NLP point from IPOpt minimax problem not valid!" << CoinMessageEol;
		 processInfo->logger.message(2) << "	Objective value is " << objval << " > " << settings->getDoubleSetting("InteriorPointFeasEps", "NLP") << CoinMessageEol;
		 return false;
		 }

		 processInfo->logger.message(2) << "NLP point from IPOpt minimax problem valid." << CoinMessageEol;
		 processInfo->logger.message(2) << " Objective value is " << objval << " < " << settings->getDoubleSetting("InteriorPointFeasEps", "NLP") << CoinMessageEol;
		 }
		 else
		 {
		 processInfo->logger.message(2) << "NLP problem linearly constrained. Assuming point is valid." << CoinMessageEol;
		 }*/

		//auto tmpIP = new InteriorPoint();
		//tmpIP->NLPSolver = static_cast<int>(ES_NLPSolver::IPOptMiniMax);
		tmpPoint.pop_back();

		if (processInfo->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
		{
			tmpPoint.pop_back();
		}

		//tmpIP->point = tmpPoint;
		//processInfo->interiorPts.push_back(*tmpIP);

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

	//if (settings->getBoolSetting("Debug", "SHOTSolver"))

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
		std::cout << "Norm " << i << ": " << tmpNorms.at(i) << std::endl;
	}

	std::vector<double> usedNorms;

	for (int i = 0; i < tmpNorms.size(); i++)
	{
		if (std::find(usedNorms.begin(), usedNorms.end(), tmpNorms.at(i)) == usedNorms.end())
		{
			selPts.push_back(tmpPoints.at(i));
			usedNorms.push_back(tmpNorms.at(i));
			std::cout << "Pt with norm: " << tmpNorms.at(i) << " added." << std::endl;
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
		tmpPoint.pop_back();
		//tmpPoint2.pop_back();

		//UtilityFunctions::displayVector(tmpPoint);
		//UtilityFunctions::displayVector(tmpPoint2);
		std::cout << "hejjjj" << std::endl;
		auto maxDev = NLPProblem->getMostDeviatingConstraint(tmpPoint).value;

		//auto maxDev2 = NLPProblem->getMostDeviatingConstraint(tmpPoint2).value;

		std::cout << "Error " << maxDev << std::endl;
		//std::cout << "Error " << maxDev2 << std::endl;

		processInfo->interiorPts.push_back(*tmpIP);
	}

	std::cout << "Number of NLP points: " << processInfo->interiorPts.size() << std::endl;

	processInfo->stopTimer("InteriorPointMinimax");
	return true;
}

void NLPSolverIPOptMinimax::saveProblemModelToFile(std::string fileName)
{
	NLPProblem->saveProblemModelToFile(fileName);
}
