#include "NLPSolverBase.h"

void NLPSolverBase::setProblem(OSInstance* origInstance)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	originalInstance = origInstance;
	isProblemInitialized = false;
}

void NLPSolverBase::initializeProblem()
{
	if (!isProblemInitialized)
	{
		createProblemInstance (originalInstance);

		isProblemInitialized = true;
	}
}

void NLPSolverBase::saveProblemToFile(std::string fileName)
{
	if (!isProblemInitialized) initializeProblem();

	NLPProblem->saveProblemModelToFile(fileName);
}

E_NLPSolutionStatus NLPSolverBase::solveProblem()
{
	if (!isProblemInitialized) initializeProblem();

	if (settings->getBoolSetting("UsePresolveBoundsForPrimalNLP", "Presolve"))
	{
		auto numVar = processInfo->originalProblem->getNumberOfVariables();

		for (int i = 0; i < numVar; i++)
		{
			if (i == processInfo->originalProblem->getNonlinearObjectiveVariableIdx()) continue;

			if (processInfo->originalProblem->hasVariableBoundsBeenTightened(i))
			{
				NLPProblem->setVariableLowerBound(i, processInfo->originalProblem->getVariableLowerBound(i));
				NLPProblem->setVariableUpperBound(i, processInfo->originalProblem->getVariableUpperBound(i));
			}
		}
	}

	auto solStatus = solveProblemInstance();

	processInfo->numNLPProbsSolved++;
	return (solStatus);
}

std::vector<double> NLPSolverBase::getVariableLowerBounds()
{

	if (!isProblemInitialized) initializeProblem();

	return (getCurrentVariableLowerBounds());
}

std::vector<double> NLPSolverBase::getVariableUpperBounds()
{
	if (!isProblemInitialized) initializeProblem();

	return (getCurrentVariableUpperBounds());
}
