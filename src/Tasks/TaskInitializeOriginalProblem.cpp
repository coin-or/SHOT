#include "TaskInitializeOriginalProblem.h"

TaskInitializeOriginalProblem::TaskInitializeOriginalProblem(OSInstance *originalInstance)
{
	//processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	ProcessInfo::getInstance().startTimer("Reformulation");

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
		ProcessInfo::getInstance().outputInfo("Nonlinear objective function detected.");
		ProcessInfo::getInstance().originalProblem = new OptProblemOriginalNonlinearObjective();
	}
	else if (isObjQuadratic && isQuadraticUsed)
	{
		ProcessInfo::getInstance().outputInfo("Quadratic objective function detected.");
		ProcessInfo::getInstance().originalProblem = new OptProblemOriginalQuadraticObjective();
	}
	else //Linear objective function
	{
		ProcessInfo::getInstance().outputInfo("Linear objective function detected.");
		ProcessInfo::getInstance().originalProblem = new OptProblemOriginalLinearObjective();
	}

	ProcessInfo::getInstance().originalProblem->setProblem(originalInstance);
	auto debugPath = settings->getStringSetting("DebugPath", "SHOTSolver");

	if (settings->getBoolSetting("Debug", "SHOTSolver"))
	{
		ProcessInfo::getInstance().originalProblem->saveProblemModelToFile(
				settings->getStringSetting("DebugPath", "SHOTSolver") + "/originalproblem.txt");
	}

	int numConstr = ProcessInfo::getInstance().originalProblem->getNumberOfConstraints();

	int numVar = ProcessInfo::getInstance().originalProblem->getNumberOfVariables();

	if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear())
	{
		numVar = numVar - 1; // Removes the extra objective variable
		numConstr = numConstr - 1; // Removes the extra objective constraint
	}

	ProcessInfo::getInstance().initializeResults(1, numVar, numConstr);

	ProcessInfo::getInstance().stopTimer("Reformulation");
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
