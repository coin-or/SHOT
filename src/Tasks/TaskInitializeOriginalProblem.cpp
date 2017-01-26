#include "TaskInitializeOriginalProblem.h"

TaskInitializeOriginalProblem::TaskInitializeOriginalProblem(OSInstance *originalInstance)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("Reformulation");

	instance = originalInstance;

	bool useQuadraticObjective = (static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm")))
			== ES_QPStrategy::QuadraticObjective;

	bool useQuadraticConstraint = (static_cast<ES_QPStrategy>(settings->getIntSetting("QPStrategy", "Algorithm")))
			== ES_QPStrategy::QuadraticallyConstrained;

	bool isObjNonlinear = UtilityFunctions::isObjectiveGenerallyNonlinear(originalInstance);
	bool isObjQuadratic = UtilityFunctions::isObjectiveQuadratic(originalInstance);
	bool isQuadraticUsed = (useQuadraticObjective || (useQuadraticConstraint));

	if (isObjNonlinear || (isObjQuadratic && !isQuadraticUsed))
	{
		processInfo->outputInfo("Nonlinear objective function detected.");
		processInfo->originalProblem = new OptProblemOriginalNonlinearObjective();
	}
	else if (isObjQuadratic && isQuadraticUsed)
	{
		processInfo->outputInfo("Quadratic objective function detected.");
		processInfo->originalProblem = new OptProblemOriginalQuadraticObjective();
	}
	else //Linear objective function
	{
		processInfo->outputInfo("Linear objective function detected.");
		processInfo->originalProblem = new OptProblemOriginalLinearObjective();
	}

	processInfo->originalProblem->setProblem(originalInstance);
	auto debugPath = settings->getStringSetting("DebugPath", "SHOTSolver");

	if (settings->getBoolSetting("Debug", "SHOTSolver"))
	{
		processInfo->originalProblem->saveProblemModelToFile(
				settings->getStringSetting("DebugPath", "SHOTSolver") + "/originalproblem.txt");
	}

	int numConstr = processInfo->originalProblem->getNumberOfConstraints();

	int numVar = processInfo->originalProblem->getNumberOfVariables();

	if (processInfo->originalProblem->isObjectiveFunctionNonlinear())
	{
		numVar = numVar - 1; // Removes the extra objective variable
		numConstr = numConstr - 1; // Removes the extra objective constraint
	}

	processInfo->initializeResults(1, numVar, numConstr);

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
