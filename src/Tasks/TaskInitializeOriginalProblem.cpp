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

	/*
	 if (useQuadraticObjective && !processInfo->MILPSolver->supportsQuadraticObjective())
	 {
	 // MIP solver does not support quadratic objectives, reseting both settings
	 useQuadraticObjective = false;
	 useQuadraticConstraint = false;
	 settings->updateSetting("QPStrategy", "Algorithm", (int) ES_QPStrategy::Nonlinear);
	 processInfo->logger.message(2)
	 << "Quadratic objective setting activated, but MIP solver does not support it. Resetting setting!"
	 << CoinMessageEol;
	 }
	 else if (useQuadraticConstraint && !processInfo->MILPSolver->supportsQuadraticConstraints()())
	 {
	 // MIP solver supports quadratic objectives but not quadratic constraints, reseting setting
	 useQuadraticConstraint = false;
	 settings->updateSetting("QPStrategy", "Algorithm", (int) ES_QPStrategy::QuadraticObjective);
	 processInfo->logger.message(2)
	 << "Quadratic constraint setting activated, but MIP solver does not support it. Resetting setting!"
	 << CoinMessageEol;
	 }*/

	bool isObjNonlinear = UtilityFunctions::isObjectiveGenerallyNonlinear(originalInstance);
	bool isObjQuadratic = UtilityFunctions::isObjectiveQuadratic(originalInstance);
	bool isQuadraticUsed = (useQuadraticObjective || (useQuadraticConstraint));

	/*if (isQuadraticUsed && isObjQuadratic && !processInfo->MILPSolver->supportsQuadraticObjective())
	 {
	 processInfo->logger.message(2) << "Quadratic objective function detected, but MIP solver does not support it!"
	 << CoinMessageEol;
	 processInfo->originalProblem = new OptProblemOriginalNonlinearObjective();
	 }
	 else*/
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
