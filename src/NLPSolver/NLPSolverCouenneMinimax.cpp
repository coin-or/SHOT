#include "NLPSolverCouenneMinimax.h"

NLPSolverCouenneMinimax::NLPSolverCouenneMinimax()
{
	NLPSolver = new CouenneSolver();
	osOption = new OSOption();
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	processInfo->startTimer("InteriorPointMinimax");

	//NLPSolver->NLPSolver = new IpoptSolver();
	//NLPSolver->NLPSolver->osoption = osOption;

	//ipoptSolver = new NLPIpoptSolver();
	//NLPSolver->nlp = ipoptSolver;

	OSoLWriter *osolwriter = NULL;
	osolwriter = new OSoLWriter();

	//(options = osolreader->readOSoL(osol);

	//NLPSolver->osoption = osOption;
	NLPSolver->osol = osolwriter->writeOSoL(osOption);

	isPointValueCached = false;

	processInfo->stopTimer("InteriorPointMinimax");
}

NLPSolverCouenneMinimax::~NLPSolverCouenneMinimax()
{
	delete osOption;
	delete NLPProblem;
	//delete NLPSolver;
}

bool NLPSolverCouenneMinimax::createProblem(OSInstance * origInstance)
{

	processInfo->startTimer("InteriorPointMinimax");
	NLPProblem = new OptProblemNLPMinimax();
	NLPProblem->reformulate(origInstance);
	NLPSolver->osinstance = NLPProblem->getProblemInstance();
	processInfo->logger.message(2) << "NLP problem created" << CoinMessageEol;
	/*NLPProblem->printProblem();
	 NLPProblem->exportProblemToOsil("exportNLP.osil");
	 std::cout << NLPProblem->problemInstance->printModel();*/

	NLPSolver->buildSolverInstance();

	processInfo->stopTimer("InteriorPointMinimax");
	return true;
}

bool NLPSolverCouenneMinimax::solveProblem()
{

	processInfo->startTimer("InteriorPointMinimax");
	try
	{
		NLPSolver->solve();
	}
	catch (...)
	{
		processInfo->logger.message(1) << "Error when solving Couenne minimax problem!" << CoinMessageEol;

		processInfo->stopTimer("InteriorPointMinimax");
		return false;
	}

	std::string solStatus = NLPSolver->osresult->getSolutionStatusType(0);

	if (solStatus == "infeasible" || solStatus == "error" || NLPSolver->osresult->getSolutionNumber() == 0)
	{
		processInfo->logger.message(1) << "No solution found to Couenne minimax problem. Solution status: " << solStatus
				<< CoinMessageEol;

		processInfo->stopTimer("InteriorPointMinimax");
		return false;
	}

	processInfo->logger.message(1) << "Solution found to Couenne minimax problem. Solution status: " << solStatus
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
			processInfo->logger.message(2) << "NLP point from Couenne minimax problem not valid!" << CoinMessageEol;
			processInfo->logger.message(2) << " Maximum constraint value is " << maxDev << " > "
					<< settings->getDoubleSetting("InteriorPointFeasEps", "NLP") << CoinMessageEol;

			processInfo->stopTimer("InteriorPointMinimax");
			return false;
		}

		processInfo->logger.message(2) << "NLP point from Couenne minimax problem valid." << CoinMessageEol;
		processInfo->logger.message(2) << " Maximum constraint value is " << maxDev << " < "
				<< settings->getDoubleSetting("InteriorPointFeasEps", "NLP") << CoinMessageEol;
	}
	else
	{
		processInfo->logger.message(2) << "NLP problem linearly constrained. Assuming point is valid."
				<< CoinMessageEol;
	}

	auto tmpIP = new InteriorPoint();
	tmpIP->NLPSolver = static_cast<int>(ES_NLPSolver::CouenneMiniMax);

	tmpPoint.pop_back();

	if (processInfo->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
	{
		tmpPoint.pop_back();
	}

	tmpIP->point = tmpPoint;
	processInfo->interiorPts.push_back(*tmpIP);

	processInfo->stopTimer("InteriorPointMinimax");
	return true;
}

void NLPSolverCouenneMinimax::saveProblemModelToFile(std::string fileName)
{
	NLPProblem->saveProblemModelToFile(fileName);
}
