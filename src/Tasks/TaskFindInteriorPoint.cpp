#include <TaskFindInteriorPoint.h>

TaskFindInteriorPoint::TaskFindInteriorPoint()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskFindInteriorPoint::~TaskFindInteriorPoint()
{
	// TODO Auto-generated destructor stub
}

void TaskFindInteriorPoint::run()
{

	processInfo->startTimer("InteriorPointTotal");

	processInfo->logger.message(3) << "Initializing NLP solver" << CoinMessageEol;
	auto solver = static_cast<ES_NLPSolver>(settings->getIntSetting("NLPSolver", "NLP"));

	if (solver == ES_NLPSolver::CuttingPlaneMiniMax)
	{
		NLPSolvers.emplace_back(new NLPSolverCuttingPlane());

		NLPSolvers[0]->createProblem(processInfo->originalProblem->getProblemInstance());

		processInfo->logger.message(2) << "Cutting plane minimax selected as NLP solver." << CoinMessageEol;
	}
	else if (solver == ES_NLPSolver::IPOptMiniMax)
	{
		NLPSolvers.emplace_back(new NLPSolverIPOptMinimax());

		NLPSolvers[0]->createProblem(processInfo->originalProblem->getProblemInstance());

		processInfo->logger.message(2) << "IPOpt minimax selected as NLP solver." << CoinMessageEol;
	}
	else if (solver == ES_NLPSolver::IPOptRelaxed)
	{
		NLPSolvers.emplace_back(new NLPSolverIPOptRelaxed());

		NLPSolvers[0]->createProblem(processInfo->originalProblem->getProblemInstance());

		processInfo->logger.message(2) << "IPOpt relaxed selected as NLP solver." << CoinMessageEol;

	}
	else if (solver == ES_NLPSolver::IPOptMiniMaxAndRelaxed)
	{
		NLPSolvers.emplace_back(new NLPSolverIPOptMinimax());

		NLPSolvers[0]->createProblem(processInfo->originalProblem->getProblemInstance());

		NLPSolvers.emplace_back(new NLPSolverIPOptRelaxed());

		NLPSolvers[1]->createProblem(processInfo->originalProblem->getProblemInstance());

		processInfo->logger.message(2) << "IPOpt minimax and relaxed selected as NLP solver." << CoinMessageEol;
	}
	else if (solver == ES_NLPSolver::CouenneMiniMax)
	{
		NLPSolvers.emplace_back(new NLPSolverCouenneMinimax());

		NLPSolvers[0]->createProblem(processInfo->originalProblem->getProblemInstance());

		processInfo->logger.message(2) << "Couenne minimax selected as NLP solver." << CoinMessageEol;

	}
	else
	{
		throw new TaskExceptionInteriorPoint("Error in NLP solver definition.");
	}

	if (settings->getBoolSetting("Debug", "SHOTSolver"))
	{
		for (int i = 0; i < NLPSolvers.size(); i++)
		{
			stringstream ss;
			ss << settings->getStringSetting("DebugPath", "SHOTSolver");
			ss << "/nlp";
			ss << i;
			ss << ".txt";

			NLPSolvers.at(i)->saveProblemModelToFile(ss.str());
		}
	}

	processInfo->logger.message(3) << "NLP solver initialized" << CoinMessageEol;

	processInfo->logger.message(3) << "Solving NLP problem" << CoinMessageEol;

	bool foundNLPPoint = false;

	for (int i = 0; i < NLPSolvers.size(); i++)
	{
		foundNLPPoint = NLPSolvers.at(i)->solveProblem() || foundNLPPoint;
	}

	if (!foundNLPPoint)
	{
		processInfo->stopTimer("InteriorPointTotal");
		throw TaskExceptionInteriorPoint("No interior point found");
	}

	processInfo->logger.message(3) << "Finished solving NLP problem" << CoinMessageEol;

	processInfo->numOriginalInteriorPoints = processInfo->interiorPts.size();

	processInfo->stopTimer("InteriorPointTotal");
}

std::string TaskFindInteriorPoint::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
