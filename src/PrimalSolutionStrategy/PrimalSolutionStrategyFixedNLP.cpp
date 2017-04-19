/*
 * PrimalSolutionStrategyFixedNLP.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: alundell
 */

#include <PrimalSolutionStrategyFixedNLP.h>

PrimalSolutionStrategyFixedNLP::PrimalSolutionStrategyFixedNLP()
{

	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	originalNLPTime = settings->getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound");
	originalNLPIter = settings->getIntSetting("NLPFixedMaxIters", "PrimalBound");

	discreteVariableIndexes = processInfo->originalProblem->getDiscreteVariableIndices();

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

	originalLBs = NLPProblem->getVariableLowerBounds();
	originalUBs = NLPProblem->getVariableUpperBounds();

	/*NLPSolver = new NLPIpoptSolver();
	 NLPSolver->osinstance = NLPProblem->getProblemInstance();*/

	processInfo->outputDebug("NLP problem for primal heuristics created.");

	return true;
}

bool PrimalSolutionStrategyFixedNLP::solveProblem()
{
	try
	{
		NLPSolver->solve();
		processInfo->numNLPProbsSolved++;
		processInfo->numPrimalFixedNLPProbsSolved++;
		std::cout << "\e[A";
		std::cout << "\e[A";

	}
	catch (std::exception &e)
	{
		auto tmpLine = boost::format("%|-4s| %|-10s|") % processInfo->numPrimalFixedNLPProbsSolved
				% "NLPFIX: Error when solving Ipopt relaxed problem";

		processInfo->outputError(tmpLine.str(), e.what());
		return (false);
	}
	catch (...)
	{
		auto tmpLine = boost::format("%|-4s| %|-10s|") % processInfo->numPrimalFixedNLPProbsSolved
				% "NLPFIX: Error when solving Ipopt relaxed problem!";

		processInfo->outputError(tmpLine.str());
		return (false);
	}

	if (NLPSolver->osresult->getSolutionStatusType(0) == "infeasible" || NLPSolver->osresult->getSolutionNumber() == 0)
	{
		return (false);
	}

	return (true);
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

	vector < PrimalFixedNLPCandidate > testPts;

	// Fix variables
	auto varTypes = processInfo->originalProblem->getVariableTypes();

	if (processInfo->primalFixedNLPCandidates.size() == 0)
	{
		processInfo->itersMILPWithoutNLPCall++;
		return (false);
	}

	if (testedPoints.size() > 0)
	{
		for (int j = 0; j < processInfo->primalFixedNLPCandidates.size(); j++)
		{
			for (int i = 0; i < testedPoints.size(); i++)
			{
				if (UtilityFunctions::isDifferentRoundedSelectedElements(
						processInfo->primalFixedNLPCandidates.at(j).point, testedPoints.at(i), discreteVariableIndexes))
				{
					testPts.push_back(processInfo->primalFixedNLPCandidates.at(j));
					testedPoints.push_back(processInfo->primalFixedNLPCandidates.at(j).point);
					break;
				}
			}
		}
	}
	else
	{
		testPts.push_back(processInfo->primalFixedNLPCandidates.at(0));
		testedPoints.push_back(processInfo->primalFixedNLPCandidates.at(0).point);
	}

	if (testPts.size() == 0)
	{
		processInfo->itersMILPWithoutNLPCall++;
		return (false);
	}

	auto lbs = NLPProblem->getVariableLowerBounds();
	auto ubs = NLPProblem->getVariableUpperBounds();

	for (int j = 0; j < testPts.size(); j++)
	{
		double timeStart = processInfo->getElapsedTime("Total");
		std::vector<bool> fixedVars(numVar, false);

		NLPSolver = new NLPIpoptSolver();
		this->initializeOSOption(); // Must initialize it for each point since the class contains fixed variable bounds

		for (int k = 0; k < numVar; k++)
		{
			auto tmpSolPt = testPts.at(j).point.at(k);

			if (varTypes.at(k) == 'I' || varTypes.at(k) == 'B')
			{
				tmpSolPt = round(tmpSolPt);

				// Fix for negative zero
				if (tmpSolPt >= (0.0 - std::numeric_limits<double>::epsilon())
						&& tmpSolPt <= 0.0 + std::numeric_limits<double>::epsilon())
				{
					tmpSolPt = 0.0;
				}

				processInfo->outputInfo(
						"       Setting initial value for " + to_string(k) + ": "
								+ to_string(NLPProblem->getVariableLowerBounds().at(k)) + " < " + to_string(tmpSolPt)
								+ " < " + to_string(NLPProblem->getVariableUpperBounds().at(k)));

				NLPProblem->fixVariable(k, tmpSolPt);
				fixedVars.at(k) = true;

				if (settings->getBoolSetting("NLPFixedWarmstart", "PrimalBound"))
				{
					osOption->setAnotherInitVarValue(k, tmpSolPt);
				}
			}
			else if (settings->getBoolSetting("NLPFixedWarmstart", "PrimalBound"))
			{

				// Nonlinear objective function variable are not used in the NLP problem for Ipopt
				if (NLPProblem->isObjectiveFunctionNonlinear() && k == NLPProblem->getNonlinearObjectiveVariableIdx())
				{
					continue;
				}

				if (tmpSolPt > ubs.at(k))
				{
					processInfo->outputInfo(
							"       Variable " + to_string(k) + " is larger than ub: " + to_string(tmpSolPt) + " > "
									+ to_string(ubs.at(k)));
					if (ubs.at(k) == 1 && tmpSolPt > 1 && tmpSolPt < 1.00001)
					{
						tmpSolPt = 1;
					}
					else
					{
						tmpSolPt = ubs.at(k);
					}
				}

				if (tmpSolPt < lbs.at(k))
				{
					processInfo->outputInfo(
							"       Variable " + to_string(k) + " is smaller than lb: " + to_string(tmpSolPt) + " < "
									+ to_string(lbs.at(k)));

					if (lbs.at(k) == 0 && tmpSolPt < 0 && tmpSolPt > -0.00001)
					{
						tmpSolPt = 0;
					}
					else
					{
						tmpSolPt = lbs.at(k);
					}

				}

				osOption->setAnotherInitVarValue(k, tmpSolPt);

				processInfo->outputInfo(
						"       Setting initial value for " + to_string(k) + ": " + to_string(originalLBs.at(k)) + " < "
								+ to_string(testPts.at(j).point.at(k)) + " < " + to_string(originalUBs.at(k)));
			}

		}

		if (settings->getBoolSetting("Debug", "SHOTSolver"))
		{
			std::string filename = settings->getStringSetting("DebugPath", "SHOTSolver") + "/primalnlp"
					+ to_string(processInfo->getCurrentIteration()->iterationNumber) + "_" + to_string(j) + ".txt";
			NLPProblem->saveProblemModelToFile(filename);
		}

		// Updates the problem to Ipopt
		NLPSolver->osinstance = NLPProblem->getProblemInstance();
		NLPSolver->osoption = osOption;

		isSolved = solveProblem();

		double timeEnd = processInfo->getElapsedTime("Total");

		std::string sourceDesc;
		switch (testPts.at(j).sourceType)
		{
			case E_PrimalNLPSource::FirstSolution:
				sourceDesc = "SOLPT ";
				break;
			case E_PrimalNLPSource::FeasibleSolution:
				sourceDesc = "FEASPT";
				break;
			case E_PrimalNLPSource::UnFeasibleSolution:
				sourceDesc = "UNFEAS";
				break;
			case E_PrimalNLPSource::SmallestDeviationSolution:
				sourceDesc = "SMADEV";
				break;
			case E_PrimalNLPSource::FirstSolutionNewDualBound:
				sourceDesc = "NEWDB";
				break;
			default:
				break;
		}

		std::string solExpr = ((boost::format("(%.3f s)") % (timeEnd - timeStart)).str());

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

			auto mostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(tmpPoint);
			std::string tmpConstr;

			if (mostDevConstr.idx != -1)
			{
				tmpConstr = processInfo->originalProblem->getConstraintNames()[mostDevConstr.idx] + ": "
						+ ((boost::format("%.5f") % mostDevConstr.value).str());
			}
			else
			{
				tmpConstr =
						processInfo->originalProblem->getConstraintNames()[processInfo->originalProblem->getNonlinearObjectiveConstraintIdx()]
								+ ": " + ((boost::format("%.5f") % mostDevConstr.value).str());
			}

			std::string tmpObjVal = UtilityFunctions::toStringFormat(tmpObj, "%.3f");

			std::string tmpPrimal = "";

			if (processInfo->getPrimalBound() > tmpObj) tmpPrimal = tmpObjVal;

			auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|")
					% processInfo->numPrimalFixedNLPProbsSolved % ("NLP" + sourceDesc) % solExpr % "" % tmpObjVal
					% tmpPrimal % tmpConstr;

			processInfo->outputSummary(tmpLine.str());

			int iters = max(ceil(settings->getIntSetting("NLPFixedMaxIters", "PrimalBound") * 0.98), originalNLPIter);
			settings->updateSetting("NLPFixedMaxIters", "PrimalBound", iters);

			double interval = max(0.9 * settings->getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound"),
					originalNLPTime);
			settings->updateSetting("NLPFixedMaxElapsedTime", "PrimalBound", interval);

			processInfo->addPrimalSolutionCandidate(tmpPoint, E_PrimalSolutionSource::NLPFixedIntegers,
					currIter->iterationNumber);
		}
		else
		{
			int iters = ceil(settings->getIntSetting("NLPFixedMaxIters", "PrimalBound") * 1.02);
			settings->updateSetting("NLPFixedMaxIters", "PrimalBound", iters);
			double interval = 1.1 * settings->getDoubleSetting("NLPFixedMaxElapsedTime", "PrimalBound");
			settings->updateSetting("NLPFixedMaxElapsedTime", "PrimalBound", interval);

			auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|")
					% processInfo->numPrimalFixedNLPProbsSolved % ("NLP" + sourceDesc) % solExpr % "" % "infeasible"
					% "" % "";

			processInfo->outputSummary(tmpLine.str());

			processInfo->outputInfo(
					"     Duration:  " + to_string(timeEnd - timeStart) + " s. New interval: " + to_string(interval)
							+ " s or " + to_string(iters) + " iters.");

			if (settings->getBoolSetting("AddIntegerCuts", "Algorithm")
					&& processInfo->originalProblem->getNumberOfIntegerVariables() == 0)
			{
				//Add integer cut.

				auto binVars = processInfo->originalProblem->getBinaryVariableIndices();

				if (binVars.size() > 0)
				{
					std::vector<int> elements;

					for (int i = 0; i < binVars.size(); i++)
					{
						if (testPts.at(j).point.at(binVars.at(i)) > 0.99)
						{
							elements.push_back(binVars.at(i));
						}
					}

					processInfo->integerCutWaitingList.push_back(elements);
				}
			}
		}

		for (int k = 0; k < numVar; k++)
		{
			if (fixedVars.at(k))
			{
				NLPProblem->setVariableLowerBound(k, originalLBs.at(k));
				NLPProblem->setVariableUpperBound(k, originalUBs.at(k));
				processInfo->outputInfo(
						"       Resetting initial bounds for variable " + to_string(k) + " lb = "
								+ to_string(originalLBs.at(k)) + " ub = " + to_string(originalUBs.at(k)));
			}
		}

		processInfo->itersMILPWithoutNLPCall = 0;
		processInfo->solTimeLastNLPCall = processInfo->getElapsedTime("Total");
	}

	processInfo->primalFixedNLPCandidates.clear();

	return (true);
}

void PrimalSolutionStrategyFixedNLP::initializeOSOption()
{

	osOption = new OSOption();

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

	auto timeLimit = settings->getDoubleSetting("NLPTimeLimit", "PrimalBound");
	osOption->setAnotherSolverOption("max_cpu_time", std::to_string(timeLimit), "ipopt", "", "double", "");

//auto constrTol = 0.000001; //settings->getDoubleSetting("InteriorPointFeasEps", "NLP");
	osOption->setAnotherSolverOption("tol", "1E-8", "ipopt", "", "double", "");
	osOption->setAnotherSolverOption("warm_start_init_point", "yes", "ipopt", "", "string", "");
//osOption->setAnotherSolverOption("max_iter", "1000", "ipopt", "", "integer", "");

// Suppress Ipopt banner
	osOption->setAnotherSolverOption("sb", "yes", "ipopt", "", "string", "");

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

//osOption->setAnotherSolverOption("fixed_variable_treatment", "relax_bounds", "ipopt", "", "string", "");

//osOption->setAnotherSolverOption("expect_infeasible_problem", "no", "ipopt", "", "string", "");
//osOption->setAnotherSolverOption("constr_viol_tol", "1E-9", "ipopt", "", "double", "");

	/*osOption->setAnotherSolverOption("barrier_tol_factor", "10", "ipopt", "", "double", "");

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
//OSoLWriter *osolwriter = NULL;
//osolwriter = new OSoLWriter();
//(options = osolreader->readOSoL(osol);
//NLPSolver->osoption = osOption;
//NLPSolver->osol = osolwriter->writeOSoL(osOption);
}

/*void PrimalSolutionStrategyFixedNLP::setFixedPoint(std::vector<double> fixedPt)
 {
 this->fixPoint = fixedPt;
 }*/
