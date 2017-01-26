#include "NLPSolverCouenneMinimax.h"

NLPSolverCouenneMinimax::NLPSolverCouenneMinimax()
{
	NLPSolver = new CouenneSolver();
	osOption = new OSOption();
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	processInfo->startTimer("InteriorPointMinimax");

	OSoLWriter *osolwriter = NULL;
	osolwriter = new OSoLWriter();

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
	processInfo->outputDebug("NLP problem created");

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
		processInfo->outputError("Error when solving Couenne minimax problem!");

		processInfo->stopTimer("InteriorPointMinimax");
		return false;
	}

	std::string solStatus = NLPSolver->osresult->getSolutionStatusType(0);

	if (solStatus == "infeasible" || solStatus == "error" || NLPSolver->osresult->getSolutionNumber() == 0)
	{
		processInfo->outputError("No solution found to Couenne minimax problem. Solution status: " + solStatus);

		processInfo->stopTimer("InteriorPointMinimax");
		return false;
	}

	processInfo->outputInfo("Solution found to Couenne minimax problem. Solution status: " + solStatus);

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
			processInfo->outputWarning("NLP point from Couenne minimax problem not valid!");
			processInfo->outputWarning(
					" Maximum constraint value is " + to_string(maxDev) + " > "
							+ to_string(settings->getDoubleSetting("InteriorPointFeasEps", "NLP")));

			processInfo->stopTimer("InteriorPointMinimax");
			return false;
		}

		processInfo->outputInfo("NLP point from Couenne minimax problem valid.");
		processInfo->outputWarning(
				" Maximum constraint value is " + to_string(maxDev) + " < "
						+ to_string(settings->getDoubleSetting("InteriorPointFeasEps", "NLP")));
	}
	else
	{
		processInfo->outputInfo("NLP problem linearly constrained. Assuming point is valid.");
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
