/*
 * PrimalSolutionStrategyFixedNLP.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: alundell
 */

#include <PrimalSolutionStrategyFixedNLP.h>

PrimalSolutionStrategyFixedNLP::PrimalSolutionStrategyFixedNLP()
{
	isOriginalRelaxedPointTested = false;
	NLPSolver = new NLPIpoptSolver();
	osOption = new OSOption();
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

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

	auto timeLimit = 1.0; //settings->getDoubleSetting("TimeLimit", "Algorithm") - processInfo->getElapsedTime(E_TimerTypes::Total);
	auto constrTol = 0.000001; //settings->getDoubleSetting("InteriorPointFeasEps", "NLP");

	osOption->setAnotherSolverOption("tol", "1E-6", "ipopt", "", "double", "");
	osOption->setAnotherSolverOption("max_iter", "10000", "ipopt", "", "integer", "");
	osOption->setAnotherSolverOption("print_level", "0", "ipopt", "", "integer", "");
	osOption->setAnotherSolverOption("sb", "yes", "ipopt", "", "string", "");
	osOption->setAnotherSolverOption("linear_solver", IPOptSolver, "ipopt", "", "string", "");
	osOption->setAnotherSolverOption("max_cpu_time", std::to_string(timeLimit), "ipopt", "", "double", "");
	//osOption->setAnotherSolverOption("constr_viol_tol", "1E-8", "ipopt", "", "double", "");

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

}

PrimalSolutionStrategyFixedNLP::~PrimalSolutionStrategyFixedNLP()
{
	// TODO Auto-generated destructor stub
}

bool PrimalSolutionStrategyFixedNLP::createProblem(OSInstance * origInstance)
{
	NLPProblem = new OptProblemNLPRelaxed();
	NLPProblem->reformulate(origInstance);
	//NLPSolver->osinstance = NLPProblem->getProblemInstance();
	processInfo->logger.message(2) << "NLP problem for primal bound created" << CoinMessageEol;

	// Run relaxed once and round, and test
	int numVar = processInfo->originalProblem->getNumberOfVariables();

	bool isSolved;
	/*
	 if (!isOriginalRelaxedPointTested)	// Tests to solve the integer relaxed problem
	 {
	 NLPSolver->osinstance = NLPProblem->getProblemInstance();
	 isSolved = solveProblem();

	 if (isSolved)
	 {
	 std::vector<double> tmpPoint(numVar);


	 int numNLPVars = NLPProblem->getNumberOfVariables();

	 for (int i = 0; i < numNLPVars; i++)
	 {
	 tmpPoint.at(i) = NLPSolver->osresult->getVarValue(0, i);
	 }

	 //checkPoint(tmpPoint,"NLP");
	 }

	 isOriginalRelaxedPointTested = true;
	 }
	 */

	return true;
}

bool PrimalSolutionStrategyFixedNLP::solveProblem()
{
	try
	{
		NLPSolver->solve();
	}
	catch (std::exception &e)
	{

	}
	catch (...)
	{
		processInfo->logger.message(1) << "Error when solving IPOpt relaxed problem!" << CoinMessageEol;
		return false;
	}

	std::string solStatus = NLPSolver->osresult->getSolutionStatusType(0);

	if (solStatus == "infeasible" || NLPSolver->osresult->getSolutionNumber() == 0)
	{
		processInfo->logger.message(1) << "No solution found to IPOpt relaxed problem. Solution status: " << solStatus
				<< CoinMessageEol;
		return false;
	}

	processInfo->logger.message(3) << "Solution found to IPOpt relaxed problem. Solution status: " << solStatus
			<< CoinMessageEol;

	/*
	 int numVar = NLPProblem->getNumberOfVariables();
	 std::vector<double> tmpPoint(numVar);

	 for (int i = 0; i < numVar; i++)
	 {
	 tmpPoint.at(i) = NLPSolver->osresult->getVarValue(0, i);
	 }

	 if (settings->getBoolSetting("Debug", "SHOTSolver"))
	 {
	 auto tmpVars = NLPProblem->getVariableNames();
	 tmpVars.push_back("mu");
	 std::string filename = settings->getStringSetting("DebugPath", "SHOTSolver") + "/nlppoint_relaxed.txt";
	 UtilityFunctions::saveVariablePointVectorToFile(tmpPoint, tmpVars, filename);
	 }*/

	return true;
}

void PrimalSolutionStrategyFixedNLP::saveProblemModelToFile(std::string fileName)
{
	NLPProblem->saveProblemModelToFile(fileName);
}

bool PrimalSolutionStrategyFixedNLP::runStrategy()
{
	auto currIter = processInfo->getCurrentIteration();

	if (!currIter->isMILP()) return false;
	// Run relaxed once and round, and test
	int numVar = processInfo->originalProblem->getNumberOfVariables();

	bool isSolved;

	// Fix variables
	auto varTypes = processInfo->originalProblem->getVariableTypes();

	auto solution = currIter->variableSolutions.at(0);

	for (int k = 0; k < numVar; k++)
	{
		if (varTypes.at(k) == 'I' || varTypes.at(k) == 'B')
		{
			NLPProblem->fixVariable(k, solution.at(k));
		}
	}

	NLPSolver->osinstance = NLPProblem->getProblemInstance();
	isSolved = solveProblem();

	if (isSolved)
	{
		std::vector<double> tmpPoint(numVar);

		int numNLPVars = NLPProblem->getNumberOfVariables();

		for (int i = 0; i < numNLPVars; i++)
		{
			tmpPoint.at(i) = NLPSolver->osresult->getVarValue(0, i);
		}

		PrimalSolution primalSol;

		primalSol.iterFound = currIter->iterationNumber;
		primalSol.objValue = currIter->objectiveValue;
		primalSol.point = tmpPoint;
		primalSol.sourceType = E_PrimalSolutionSource::NLPFixedIntegers;

		checkPoint(primalSol);

	}

//saveProblemModelToFile("NLPprimal.txt");

	processInfo->itersMILPWithoutNLPCall = 0;

	return true;
}
