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
	osOption->setAnotherSolverOption("print_level", "5", "ipopt", "", "integer", "");
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
	//OSiLWriter *osilwriter = NULL;

	//osilwriter = new OSiLWriter();

	//std::string test = osilwriter->writeOSiL(NLPProblem->getProblemInstance());

	//NLPProblem->exportProblemToOsil("exportNLP.osil");
	/*
	 FileUtil *fileUtil = new FileUtil();
	 OSiLReader *osilreader = new OSiLReader();
	 OSInstance *instance = NULL;


	 std::string osil = fileUtil->getFileAsChar(origInstance->instanceHeader->source.c_str());

	 instance = osilreader->readOSiL(osil);


	 for (int i = 0; i < instance->getVariableNumber(); i++)
	 {
	 instance->instanceData->variables->var[i]->type = 'C';
	 }

	 instance->getJacobianSparsityPattern();

	 NLPSolver->osinstance = instance;	*/
	//NLPSolver->buildSolverInstance();
	processInfo->logger.message(2) << "NLP problem created" << CoinMessageEol;
	/*NLPProblem->printProblem();
	 NLPProblem->exportProblemToOsil("exportNLP.osil");
	 std::cout << NLPProblem->problemInstance->printModel();*/

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
	}
	catch (std::exception e)
	{
		processInfo->logger.message(1) << "Error when solving IPOpt relaxed problem!" << CoinMessageEol;

		processInfo->stopTimer("InteriorPointRelaxed");
		return false;
	}
	catch (...)
	{
		processInfo->logger.message(1) << "Error when solving IPOpt relaxed problem!" << CoinMessageEol;

		processInfo->stopTimer("InteriorPointRelaxed");
		return false;
	}

	std::string solStatus = NLPSolver->osresult->getSolutionStatusType(0);

	if (solStatus == "infeasible" || NLPSolver->osresult->getSolutionNumber() == 0)
	{
		processInfo->logger.message(1) << "No solution found to IPOpt minimax problem. Solution status: " << solStatus
				<< CoinMessageEol;

		processInfo->stopTimer("InteriorPointRelaxed");
		return false;
	}

	processInfo->logger.message(3) << "Solution found to IPOpt relaxed problem. Solution status: " << solStatus
			<< CoinMessageEol;

	int numVar = NLPProblem->getNumberOfVariables();
	std::vector<double> tmpPoint(numVar);

	for (int i = 0; i < numVar; i++)
	{
		tmpPoint.at(i) = NLPSolver->osresult->getVarValue(0, i);
	}

	if (NLPProblem->getNumberOfNonlinearConstraints() > 0)
	{
		auto maxDev = NLPProblem->getMostDeviatingConstraint(tmpPoint).value;
		if (maxDev > settings->getDoubleSetting("InteriorPointFeasEps", "NLP"))
		{
			processInfo->logger.message(2) << "NLP point from IPOpt relaxed problem not valid!" << CoinMessageEol;
			processInfo->logger.message(2) << " Maximum constraint value is " << maxDev << " > "
					<< settings->getDoubleSetting("InteriorPointFeasEps", "NLP") << CoinMessageEol;

			processInfo->stopTimer("InteriorPointRelaxed");
			return false;
		}

		processInfo->logger.message(2) << "NLP point from IPOpt relaxed problem valid." << CoinMessageEol;
		processInfo->logger.message(2) << " Maximum constraint value is " << maxDev << " < "
				<< settings->getDoubleSetting("InteriorPointFeasEps", "NLP") << CoinMessageEol;
	}
	else
	{
		processInfo->logger.message(2) << "NLP problem linearly constrained. Assuming point is valid."
				<< CoinMessageEol;
	}

	auto tmpIP = new InteriorPoint();
	tmpIP->NLPSolver = static_cast<int>(ES_NLPSolver::IPOptRelaxed);

	if (NLPProblem->isObjectiveFunctionNonlinear()
			&& processInfo->originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
	{
		tmpPoint.push_back(NLPSolver->osresult->getObjValue(0, 0));
	}

	tmpIP->point = tmpPoint;

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

/*
 bool NLPSolverIPOptRelaxed::solveProblem()
 {
 std::string solStatus;
 double optSolValue;

 NLPSolver->solve();
 // the argument is the solution index
 solStatus = NLPSolver->osresult->getSolutionStatusType(0);

 if (solStatus.find("ptimal") != std::string::npos || solStatus == "bestSoFar" || solStatus == "feasible"){
 std::cout << solStatus << std::endl;
 optSolValue = NLPSolver->osresult->optimization->solution[0]->objectives->values[0].obj[0]->value;
 }
 else{
 std::cout << "IPOpt could not solve problem. The solver status is: " << solStatus << std::endl;

 if (NLPProblem->isConstraintsFulfilledInPoint(getSolution()))
 return true;
 else
 {
 std::cout << "NLP solution is not valid!" << std::endl;
 std::cout << "Maximum constraint value in NLP point is " << NLPProblem->getMostDeviatingConstraint(getSolution()).value << std::endl;

 return false;
 }
 }

 if (NLPProblem->isConstraintsFulfilledInPoint(getSolution()))
 {

 std::cout << "Maximum constraint value in NLP point is " << NLPProblem->getMostDeviatingConstraint(getSolution()).value << std::endl;

 return true;
 }
 else
 {
 std::cout << "NLP solution is not valid!" << std::endl;
 std::cout << "Maximum constraint value in NLP point is " << NLPProblem->getMostDeviatingConstraint(getSolution()).value << std::endl;

 return false;
 }
 }

 std::vector<double> NLPSolverIPOptRelaxed::getSolution()
 {
 if (!isPointValueCached)
 {
 int numVar = NLPProblem->getNumberOfVariables();

 std::vector<double> xNLP(numVar);

 for (int i = 0; i < numVar; i++)
 {
 xNLP[i] = NLPSolver->osresult->getVarValue(0, i);
 }


 NLPpoint = xNLP;
 }

 return NLPpoint;
 }*/
