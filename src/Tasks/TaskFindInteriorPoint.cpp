#include "TaskFindInteriorPoint.h"

TaskFindInteriorPoint::TaskFindInteriorPoint()
{
}

TaskFindInteriorPoint::~TaskFindInteriorPoint()
{
	NLPSolvers.clear();
}

void TaskFindInteriorPoint::run()
{
	ProcessInfo::getInstance().startTimer("InteriorPointTotal");

	ProcessInfo::getInstance().outputDebug("Initializing NLP solver");
	auto solver = static_cast<ES_NLPSolver>(Settings::getInstance().getIntSetting("ESH.InteriorPoint.Solver", "Dual"));

	if (solver == ES_NLPSolver::CuttingPlaneMiniMax)
	{
		NLPSolvers.emplace_back(new NLPSolverCuttingPlaneMinimax());

		NLPSolvers[0]->setProblem(ProcessInfo::getInstance().originalProblem->getProblemInstance());

		ProcessInfo::getInstance().outputDebug("Cutting plane minimax selected as NLP solver.");
	}
	else if (solver == ES_NLPSolver::IpoptMinimax)
	{
		NLPSolvers.emplace_back(new NLPSolverIpoptMinimax());

		NLPSolvers[0]->setProblem(ProcessInfo::getInstance().originalProblem->getProblemInstance());

		ProcessInfo::getInstance().outputDebug("Ipopt minimax selected as NLP solver.");
	}
	else if (solver == ES_NLPSolver::IpoptRelaxed)
	{
		NLPSolvers.emplace_back(new NLPSolverIpoptRelaxed());

		NLPSolvers[0]->setProblem(ProcessInfo::getInstance().originalProblem->getProblemInstance());

		ProcessInfo::getInstance().outputDebug("Ipopt relaxed selected as NLP solver.");
	}
	else if (solver == ES_NLPSolver::IpoptMinimaxAndRelaxed)
	{
		NLPSolvers.emplace_back(new NLPSolverIpoptMinimax());

		NLPSolvers[0]->setProblem(ProcessInfo::getInstance().originalProblem->getProblemInstance());

		NLPSolvers.emplace_back(new NLPSolverIpoptRelaxed());

		NLPSolvers[1]->setProblem(ProcessInfo::getInstance().originalProblem->getProblemInstance());

		ProcessInfo::getInstance().outputDebug("Ipopt minimax and relaxed selected as NLP solver.");
	}
	else
	{
		return;
	}

	if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
	{
		for (int i = 0; i < NLPSolvers.size(); i++)
		{
			stringstream ss;
			ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
			ss << "/interiorpointnlp";
			ss << i;
			ss << ".txt";

			NLPSolvers.at(i)->saveProblemToFile(ss.str());
		}
	}

	ProcessInfo::getInstance().outputDebug(" Solving NLP problem.");

	bool foundNLPPoint = false;

	for (int i = 0; i < NLPSolvers.size(); i++)
	{
		auto solutionStatus = NLPSolvers.at(i)->solveProblem();

		std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

		tmpIP->NLPSolver = static_cast<ES_NLPSolver>(Settings::getInstance().getIntSetting("ESH.InteriorPoint.Solver", "Dual"));

		tmpIP->point = NLPSolvers.at(i)->getSolution();

		if (solver == ES_NLPSolver::IpoptRelaxed && tmpIP->point.size() < ProcessInfo::getInstance().originalProblem->getNumberOfVariables())
		{
			tmpIP->point.push_back(
				ProcessInfo::getInstance().originalProblem->calculateOriginalObjectiveValue(tmpIP->point));
		}

		while (tmpIP->point.size() > ProcessInfo::getInstance().originalProblem->getNumberOfVariables())
		{
			tmpIP->point.pop_back();
		}

		auto maxDev = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(tmpIP->point);
		tmpIP->maxDevatingConstraint = maxDev;

		if (maxDev.value > 0)
		{
			ProcessInfo::getInstance().outputWarning("\n Maximum deviation in interior point is too large: " + UtilityFunctions::toString(maxDev.value));
		}
		else
		{
			ProcessInfo::getInstance().outputError("\n Valid interior point with constraint deviation " + UtilityFunctions::toString(maxDev.value) + " found.");
			ProcessInfo::getInstance().interiorPts.push_back(tmpIP);
		}

		foundNLPPoint = (foundNLPPoint || (maxDev.value <= 0));

		if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
		{
			auto tmpVars = ProcessInfo::getInstance().originalProblem->getVariableNames();
			std::string filename = Settings::getInstance().getStringSetting("Debug.Path", "Output") + "/interiorpoint_" + to_string(i) + ".txt";
			UtilityFunctions::saveVariablePointVectorToFile(tmpIP->point, tmpVars, filename);
		}
	}

	if (!foundNLPPoint)
	{
		ProcessInfo::getInstance().outputError("\n No interior point found!                            ");
		ProcessInfo::getInstance().stopTimer("InteriorPointTotal");
		
		return;
	}

	ProcessInfo::getInstance().outputDebug("     Finished solving NLP problem.");

	ProcessInfo::getInstance().numOriginalInteriorPoints = ProcessInfo::getInstance().interiorPts.size();

	ProcessInfo::getInstance().stopTimer("InteriorPointTotal");
}

std::string TaskFindInteriorPoint::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
