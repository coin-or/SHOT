#include "TaskInitializeOriginalProblem.h"

TaskInitializeOriginalProblem::TaskInitializeOriginalProblem(OSInstance *originalInstance)
{

	ProcessInfo::getInstance().startTimer("Reformulation");

	instance = originalInstance;

	bool useQuadraticObjective = (static_cast<ES_QPStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QPStrategy::QuadraticObjective;

	bool useQuadraticConstraint = (static_cast<ES_QPStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QPStrategy::QuadraticallyConstrained;

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
	auto debugPath = Settings::getInstance().getStringSetting("Debug.Path", "Output");

	if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
	{
		ProcessInfo::getInstance().originalProblem->saveProblemModelToFile(
				Settings::getInstance().getStringSetting("Debug.Path", "Output") + "/originalproblem.txt");
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
