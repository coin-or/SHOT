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

	processInfo->outputDebug("Initializing NLP solver");
	auto solver = static_cast<ES_NLPSolver>(settings->getIntSetting("InteriorPointSolver", "InteriorPoint"));

	if (solver == ES_NLPSolver::CuttingPlaneMiniMax)
	{
		NLPSolvers.emplace_back(new NLPSolverCuttingPlaneMinimax());

		NLPSolvers[0]->setProblem(processInfo->originalProblem->getProblemInstance());

		processInfo->outputDebug("Cutting plane minimax selected as NLP solver.");
	}
	else if (solver == ES_NLPSolver::IPOptMiniMax)
	{
		NLPSolvers.emplace_back(new NLPSolverIPOptMinimax());

		NLPSolvers[0]->setProblem(processInfo->originalProblem->getProblemInstance());

		processInfo->outputDebug("IPOpt minimax selected as NLP solver.");
	}
	else if (solver == ES_NLPSolver::IPOptRelaxed)
	{
		NLPSolvers.emplace_back(new NLPSolverIPOptRelaxed());

		NLPSolvers[0]->setProblem(processInfo->originalProblem->getProblemInstance());

		processInfo->outputDebug("IPOpt relaxed selected as NLP solver.");

	}
	else if (solver == ES_NLPSolver::IPOptMiniMaxAndRelaxed)
	{
		NLPSolvers.emplace_back(new NLPSolverIPOptMinimax());

		NLPSolvers[0]->setProblem(processInfo->originalProblem->getProblemInstance());

		NLPSolvers.emplace_back(new NLPSolverIPOptRelaxed());

		NLPSolvers[1]->setProblem(processInfo->originalProblem->getProblemInstance());

		processInfo->outputDebug("IPOpt minimax and relaxed selected as NLP solver.");
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

			NLPSolvers.at(i)->saveProblemToFile(ss.str());
		}
	}

	processInfo->outputDebug("Solving NLP problem.");

	bool foundNLPPoint = false;

	for (int i = 0; i < NLPSolvers.size(); i++)
	{
		auto solutionStatus = NLPSolvers.at(i)->solveProblem();

		auto tmpIP = new InteriorPoint();
		tmpIP->NLPSolver = ES_NLPSolver::CuttingPlaneMiniMax;
		tmpIP->point = NLPSolvers.at(i)->getSolution();

		auto maxDev = processInfo->originalProblem->getMostDeviatingConstraint(tmpIP->point);
		tmpIP->maxDevatingConstraint = maxDev;

		processInfo->interiorPts.push_back(*tmpIP);

		foundNLPPoint = (foundNLPPoint || (maxDev.value <= 0));

		//foundNLPPoint = (solutionStatus == E_NLPSolutionStatus::Feasible
		//		|| solutionStatus == E_NLPSolutionStatus::Optimal) || foundNLPPoint;
	}

	if (!foundNLPPoint)
	{
		processInfo->stopTimer("InteriorPointTotal");
		throw TaskExceptionInteriorPoint("No interior point found");
	}

	processInfo->outputDebug("Finished solving NLP problem.");

	processInfo->numOriginalInteriorPoints = processInfo->interiorPts.size();

	processInfo->stopTimer("InteriorPointTotal");
}

std::string TaskFindInteriorPoint::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
