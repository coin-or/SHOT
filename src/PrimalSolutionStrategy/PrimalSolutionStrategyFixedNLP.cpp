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

	discreteVariableIndexes = processInfo->originalProblem->getDiscreteVariableIndices();

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

	auto timeLimit = 1; //settings->getDoubleSetting("TimeLimit", "Algorithm") - processInfo->getElapsedTime("Total");
	//auto constrTol = 0.000001; //settings->getDoubleSetting("InteriorPointFeasEps", "NLP");

	osOption->setAnotherSolverOption("tol", "1E-8", "ipopt", "", "double", "");
	osOption->setAnotherSolverOption("max_iter", "10000", "ipopt", "", "integer", "");
	osOption->setAnotherSolverOption("print_level", "0", "ipopt", "", "integer", "");
	osOption->setAnotherSolverOption("sb", "yes", "ipopt", "", "string", "");
	osOption->setAnotherSolverOption("linear_solver", IPOptSolver, "ipopt", "", "string", "");
	osOption->setAnotherSolverOption("max_cpu_time", std::to_string(timeLimit), "ipopt", "", "double", "");
	//osOption->setAnotherSolverOption("constr_viol_tol", "1E-9", "ipopt", "", "double", "");

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
		std::cout << "\e[A";
		std::cout << "\e[A";

	}
	catch (std::exception &e)
	{
		return false;
	}
	catch (...)
	{
		processInfo->logger.message(1) << "Error when solving IPOpt relaxed problem!" << CoinMessageEol;
		return false;
	}

	if (NLPSolver->osresult->getSolutionStatusType(0) == "infeasible" || NLPSolver->osresult->getSolutionNumber() == 0)
	{
		return false;
	}

	return true;
}

void PrimalSolutionStrategyFixedNLP::saveProblemModelToFile(std::string fileName)
{
	NLPProblem->saveProblemModelToFile(fileName);
}

bool PrimalSolutionStrategyFixedNLP::runStrategy()
{
	auto currIter = processInfo->getCurrentIteration();

	int numVar = processInfo->originalProblem->getNumberOfVariables();

	bool isSolved;

	vector < SolutionPoint > solPts;

	// Fix variables
	auto varTypes = processInfo->originalProblem->getVariableTypes();
	if (currIter->solutionPoints.size() == 0) //No solution found in last iteration
	{

		if (processInfo->dualSolutions.size() > 0)
		{
			auto tmpPoint = processInfo->dualSolutions.back().point;

			for (int i = 0; i < testedPoints.size(); i++)
			{
				if (UtilityFunctions::isDifferentRoundedSelectedElements(tmpPoint, testedPoints.at(i),
						discreteVariableIndexes))
				{
					SolutionPoint tmpSP;
					tmpSP.iterFound = processInfo->dualSolutions.back().iterFound;
					//tmpSP.maxDeviation = processInfo->dualSolutions.back().;
					tmpSP.objectiveValue = processInfo->dualSolutions.back().objValue;
					tmpSP.point = tmpPoint;
					tmpSP.maxDeviation = processInfo->originalProblem->getMostDeviatingConstraint(tmpSP.point);

					solPts.push_back(tmpSP);
					testedPoints.push_back(tmpSP.point);
					break;
				}
			}

		}
	}
	else
	{
		if (settings->getIntSetting("NLPFixedSource", "PrimalBound")
				== static_cast<int>(ES_PrimalBoundNLPFixedPoint::Both)
				|| settings->getIntSetting("NLPFixedSource", "PrimalBound")
						== static_cast<int>(ES_PrimalBoundNLPFixedPoint::MILPSolution))
		{
			if (testedPoints.size() == 0)
			{
				solPts.push_back(currIter->solutionPoints.at(0));
				testedPoints.push_back(currIter->solutionPoints.at(0).point);
			}
			else
			{
				for (int i = 0; i < testedPoints.size(); i++)
				{
					if (UtilityFunctions::isDifferentRoundedSelectedElements(currIter->solutionPoints.at(0).point,
							testedPoints.at(i), discreteVariableIndexes))
					{
						solPts.push_back(currIter->solutionPoints.at(0));
						testedPoints.push_back(currIter->solutionPoints.at(0).point);
						break;
					}
					else
					{
					}
				}
			}
		}

		if (settings->getIntSetting("NLPFixedSource", "PrimalBound")
				== static_cast<int>(ES_PrimalBoundNLPFixedPoint::Both)
				|| settings->getIntSetting("NLPFixedSource", "PrimalBound")
						== static_cast<int>(ES_PrimalBoundNLPFixedPoint::SmallestDeviation))
		{
			if (testedPoints.size() == 0)
			{
				solPts.push_back(currIter->getSolutionPointWithSmallestDeviation());
				testedPoints.push_back(currIter->getSolutionPointWithSmallestDeviation().point);
			}
			else
			{
				auto solPtSmallestDev = currIter->getSolutionPointWithSmallestDeviation();

				for (int i = 0; i < testedPoints.size(); i++)
				{
					if (UtilityFunctions::isDifferentRoundedSelectedElements(solPtSmallestDev.point, testedPoints.at(i),
							discreteVariableIndexes))
					{
						solPts.push_back(solPtSmallestDev);
						testedPoints.push_back(solPtSmallestDev.point);
						break;
					}
				}
			}
		}
	}

	if (solPts.size() == 0)
	{
		processInfo->itersMILPWithoutNLPCall++;
		//processInfo->logger.message(1) << "    No NLP call performed" << CoinMessageEol;
		return (false);

	}

	for (int j = 0; j < solPts.size(); j++)
	{
		double timeStart = processInfo->getElapsedTime("Total");

		for (int k = 0; k < numVar; k++)
		{
			if (varTypes.at(k) == 'I' || varTypes.at(k) == 'B')
			{
				NLPProblem->fixVariable(k, solPts.at(j).point.at(k));
			}
		}

		NLPSolver->osinstance = NLPProblem->getProblemInstance();
		isSolved = solveProblem();

		double timeEnd = processInfo->getElapsedTime("Total");
		if (isSolved)
		{
			std::vector<double> tmpPoint(numVar);

			double tmpObj = NLPSolver->osresult->getObjValue(0, 0);

			int numNLPVars = NLPProblem->getNumberOfVariables();

			for (int i = 0; i < numNLPVars; i++)
			{
				tmpPoint.at(i) = NLPSolver->osresult->getVarValue(0, i);
			}

			if (processInfo->originalProblem->isObjectiveFunctionNonlinear())
			{
				tmpPoint.back() = tmpObj;
			}

			processInfo->addPrimalSolutionCandidate(tmpPoint, E_PrimalSolutionSource::NLPFixedIntegers,
					currIter->iterationNumber);

			/*UtilityFunctions::displayVector(tmpPoint);

			 UtilityFunctions::displayVector(processInfo->dualSolutions.back().point);
			 */
			auto error = processInfo->originalProblem->getMostDeviatingConstraint(tmpPoint).value;

			processInfo->logger.message(1) << "    NLP call objective: " << tmpObj << ", max viol.: " << error
					<< ", duration: " << timeEnd - timeStart << " s" << CoinMessageEol;
		}
		else
		{
			processInfo->logger.message(1) << "    Unsuccessful NLP call, duration:  " << timeEnd - timeStart << " s"
					<< CoinMessageEol;
		}

		processInfo->itersMILPWithoutNLPCall = 0;
	}

	return (true);
}
