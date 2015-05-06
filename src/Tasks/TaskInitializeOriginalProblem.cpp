#include "TaskInitializeOriginalProblem.h"

TaskInitializeOriginalProblem::TaskInitializeOriginalProblem(OSInstance *originalInstance)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("Reformulation");

	instance = originalInstance;

	bool isObjNonlinear = UtilityFunctions::isObjectiveGenerallyNonlinear(originalInstance);
	bool isObjQuadratic = UtilityFunctions::isObjectiveQuadratic(originalInstance);
	bool isQuadraticUsed = ((static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm")))
			== ES_QPStrategy::QuadraticObjective)
			|| ((static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm")))
					== ES_QPStrategy::QuadraticallyConstrained);

	if (isObjNonlinear || (isObjQuadratic && !isQuadraticUsed))
	{
		processInfo->logger.message(2) << "Nonlinear objective function detected" << CoinMessageEol;
		processInfo->originalProblem = new OptProblemOriginalNonlinearObjective();
	}
	else if (isObjQuadratic && isQuadraticUsed)
	{
		processInfo->logger.message(2) << "Quadratic objective function detected" << CoinMessageEol;
		processInfo->originalProblem = new OptProblemOriginalQuadraticObjective();
	}
	else //Linear objective function
	{
		processInfo->logger.message(2) << "Linear objective function detected" << CoinMessageEol;
		processInfo->originalProblem = new OptProblemOriginalLinearObjective();
	}

	processInfo->originalProblem->setProblem(originalInstance);
	auto debugPath = settings->getStringSetting("DebugPath", "SHOTSolver");

	if (settings->getBoolSetting("Debug", "SHOTSolver"))
	{
		processInfo->originalProblem->saveProblemModelToFile(
				settings->getStringSetting("DebugPath", "SHOTSolver") + "/originalproblem.txt");
	}

	processInfo->initializeResults(1, processInfo->originalProblem->getNumberOfVariables(),
			processInfo->originalProblem->getNumberOfConstraints());

	processInfo->stopTimer("Reformulation");
}

TaskInitializeOriginalProblem::~TaskInitializeOriginalProblem()
{
}

void TaskInitializeOriginalProblem::run()
{

}

std::string TaskInitializeOriginalProblem::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
