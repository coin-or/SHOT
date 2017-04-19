#include "MILPSolverBase.h"

MILPSolverBase::MILPSolverBase()
{
}

MILPSolverBase::~MILPSolverBase()
{
}

void MILPSolverBase::startTimer()
{
	if (discreteVariablesActivated) processInfo->startTimer("MILP");
	else processInfo->startTimer("LP");
}

void MILPSolverBase::stopTimer()
{
	if (discreteVariablesActivated) processInfo->stopTimer("MILP");
	else processInfo->stopTimer("LP");
}

double MILPSolverBase::getObjectiveValue()
{
	double objval = getObjectiveValue(0);
	return (objval);
}

bool MILPSolverBase::getDiscreteVariableStatus()
{
	return (discreteVariablesActivated);
}

std::vector<SolutionPoint> MILPSolverBase::getAllVariableSolutions()
{
	if (cachedSolutionHasChanged == false) return (lastSolutions);

	int numSol = getNumberOfSolutions();
	int numVar = processInfo->originalProblem->getNumberOfVariables();

	std::vector < SolutionPoint > allSolutions(numSol);

	// Should be moved to separate task
	bool isMILP = getDiscreteVariableStatus();
	if (isMILP && settings->getBoolSetting("PopulateSolutionPool", "MILP"))
	{
		populateSolutionPool();
	}

	for (int i = 0; i < numSol; i++)
	{
		SolutionPoint tmpSolPt;

		auto tmpPt = getVariableSolution(i);

		auto maxDev = processInfo->originalProblem->getMostDeviatingConstraint(tmpPt);

		tmpSolPt.point = tmpPt;
		tmpSolPt.objectiveValue = getObjectiveValue(i);
		tmpSolPt.iterFound = processInfo->getCurrentIteration()->iterationNumber;
		tmpSolPt.maxDeviation = maxDev;

		allSolutions.at(i) = tmpSolPt;
	}

	cachedSolutionHasChanged = false;
	lastSolutions = allSolutions;

	return (lastSolutions);
}

void MILPSolverBase::createHyperplane(Hyperplane hyperplane)
{
	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration
	auto originalProblem = processInfo->originalProblem;
	std::vector < IndexValuePair > elements;

	auto varNames = originalProblem->getVariableNames();

	double constant = originalProblem->calculateConstraintFunctionValue(hyperplane.sourceConstraintIndex,
			hyperplane.generatedPoint);

	if (hyperplane.sourceConstraintIndex == -1
			|| hyperplane.sourceConstraintIndex == originalProblem->getNonlinearObjectiveConstraintIdx())
	{
		processInfo->outputDebug("     HP point generated for auxiliary objective function constraint");

		auto tmpArray = processInfo->originalProblem->getProblemInstance()->calculateObjectiveFunctionGradient(
				&hyperplane.generatedPoint.at(0), -1, true);
		int number = processInfo->originalProblem->getNumberOfVariables();
		processInfo->numGradientEvals++;

		for (int i = 0; i < number - 1; i++)
		{
			if (tmpArray[i] != 0)
			{
				IndexValuePair pair;
				pair.idx = i;
				pair.value = tmpArray[i];

				elements.push_back(pair);
				constant += -tmpArray[i] * hyperplane.generatedPoint.at(i);

				processInfo->outputDebug(
						"     Gradient for variable " + varNames.at(i) + ": " + to_string(tmpArray[i]));
			}
		}

		processInfo->outputDebug("     Gradient for obj.var.: -1");

		IndexValuePair pair;
		pair.idx = processInfo->originalProblem->getNonlinearObjectiveVariableIdx();
		pair.value = -1.0;

		elements.push_back(pair);
		constant += /*-(-1) **/hyperplane.generatedPoint.at(pair.idx);
	}
	else
	{
		processInfo->outputDebug(
				"     HP point generated for constraint index " + to_string(hyperplane.sourceConstraintIndex));

		auto nablag = originalProblem->calculateConstraintFunctionGradient(hyperplane.sourceConstraintIndex,
				hyperplane.generatedPoint);

		for (int i = 0; i < nablag->number; i++)
		{
			IndexValuePair pair;
			pair.idx = nablag->indexes[i];
			pair.value = nablag->values[i];

			elements.push_back(pair);
			constant += -nablag->values[i] * hyperplane.generatedPoint.at(nablag->indexes[i]);

			processInfo->outputDebug(
					"     Gradient for variable" + varNames.at(nablag->indexes[i]) + ": "
							+ to_string(nablag->values[i]));
		}
	}

	bool hyperplaneIsOk = true;

	for (auto E : elements)
	{
		if (E.value != E.value) //Check for NaN
		{

			processInfo->outputWarning("     Warning: hyperplane not generated, NaN found in linear terms!");
			hyperplaneIsOk = false;
			break;
		}
	}

	if (hyperplaneIsOk)
	{
		int constrIndex = processInfo->MILPSolver->addLinearConstraint(elements, constant);
		addedHyperplanes++;
		GeneratedHyperplane genHyperplane;

		genHyperplane.generatedConstraintIndex = constrIndex;
		genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
		genHyperplane.generatedPoint = hyperplane.generatedPoint;
		genHyperplane.source = hyperplane.source;
		genHyperplane.generatedIter = currIter->iterationNumber;

		generatedHyperplanes.push_back(genHyperplane);

		currIter->numHyperplanesAdded++;
		currIter->totNumHyperplanes++;
	}
}

void MILPSolverBase::createInteriorHyperplane(Hyperplane hyperplane)
{
	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration
	auto originalProblem = processInfo->originalProblem;
	std::vector < IndexValuePair > elements;

	auto varNames = originalProblem->getVariableNames();

	double constant = originalProblem->calculateConstraintFunctionValue(hyperplane.sourceConstraintIndex,
			hyperplane.generatedPoint);

	auto tmpArray = processInfo->originalProblem->getProblemInstance()->calculateObjectiveFunctionGradient(
			&hyperplane.generatedPoint.at(0), -1, true);
	int number = processInfo->originalProblem->getNumberOfVariables();
	processInfo->numGradientEvals++;

	for (int i = 0; i < number - 1; i++)
	{
		if (tmpArray[i] != 0)
		{
			IndexValuePair pair;
			pair.idx = i;
			pair.value = tmpArray[i];

			elements.push_back(pair);
			constant += -tmpArray[i] * hyperplane.generatedPoint.at(i);
		}
	}

	IndexValuePair pair;
	pair.idx = processInfo->originalProblem->getNonlinearObjectiveVariableIdx();
	pair.value = -1.0;

	elements.push_back(pair);
	constant += hyperplane.generatedPoint.at(pair.idx);

	bool hyperplaneIsOk = true;

	for (auto E : elements)
	{
		if (E.value != E.value) //Check for NaN
		{
			processInfo->outputWarning("     Warning: hyperplane not generated, NaN found in linear terms!");

			hyperplaneIsOk = false;
			break;
		}
	}

	if (hyperplaneIsOk)
	{
		int constrIndex = processInfo->MILPSolver->addLinearConstraint(elements, constant, false);
		addedHyperplanes++;
		GeneratedHyperplane genHyperplane;

		genHyperplane.generatedConstraintIndex = constrIndex;
		genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
		genHyperplane.generatedPoint = hyperplane.generatedPoint;
		genHyperplane.source = hyperplane.source;
		genHyperplane.generatedIter = currIter->iterationNumber;

		generatedHyperplanes.push_back(genHyperplane);

		currIter->numHyperplanesAdded++;
		currIter->totNumHyperplanes++;
	}

	currIter->totNumHyperplanes = processInfo->getPreviousIteration()->totNumHyperplanes
			+ currIter->numHyperplanesAdded;
}

std::vector<GeneratedHyperplane> *MILPSolverBase::getGeneratedHyperplanes()
{
	return (&generatedHyperplanes);
}

void MILPSolverBase::presolveAndUpdateBounds()
{
	auto newBounds = this->presolveAndGetNewBounds();

	auto numVar = processInfo->originalProblem->getNumberOfVariables();

	for (int i = 0; i < numVar; i++)
	{
		auto currBounds = this->getCurrentVariableBounds(i);

		bool newLB = false;
		bool newUB = false;

		if (newBounds.first.at(i) > currBounds.first) newLB = true;
		if (newBounds.second.at(i) > currBounds.second) newUB = true;

		if (newLB)
		{
			processInfo->originalProblem->setVariableUpperBound(i, newBounds.second.at(i));
			processInfo->outputInfo(
					"     Lower bound for variable (" + to_string(i) + ") updated from "
							+ UtilityFunctions::toString(currBounds.first) + " to "
							+ UtilityFunctions::toString(newBounds.first.at(i)));

			if (!processInfo->originalProblem->hasVariableBoundsBeenTightened(i))
			{
				processInfo->originalProblem->setVariableBoundsAsTightened(i);
				processInfo->numVariableBoundsTightenedInPresolve++;
			}
		}

		if (newUB)
		{
			processInfo->originalProblem->setVariableUpperBound(i, newBounds.second.at(i));
			processInfo->outputInfo(
					"     Upper bound for variable (" + to_string(i) + ") updated from "
							+ UtilityFunctions::toString(currBounds.second) + " to "
							+ UtilityFunctions::toString(newBounds.second.at(i)));

			if (!processInfo->originalProblem->hasVariableBoundsBeenTightened(i))
			{
				processInfo->originalProblem->setVariableBoundsAsTightened(i);
				processInfo->numVariableBoundsTightenedInPresolve++;
			}
		}

		if (settings->getBoolSetting("UsePresolveBoundsForMIP", "Presolve") && (newLB || newUB))
		{
			updateVariableBound(i, newBounds.first.at(i), newBounds.second.at(i));
			processInfo->outputInfo("     Bounds updated also in MIP problem");
		}
	}
}

void MILPSolverBase::updateNonlinearObjectiveFromPrimalDualBounds()
{
	if (!processInfo->originalProblem->isObjectiveFunctionNonlinear())
	{
		return;
	}

	auto varIdx = processInfo->originalProblem->getNonlinearObjectiveVariableIdx();

	auto newLB = processInfo->getDualBound();
	auto newUB = processInfo->getPrimalBound();

	auto currBounds = this->getCurrentVariableBounds(varIdx);

	if (newLB > currBounds.first || newUB < currBounds.second)
	{
		this->updateVariableBound(varIdx, newLB, newUB);
		processInfo->outputInfo(
				"     Bounds for nonlinear objective function updated to " + UtilityFunctions::toString(newLB) + " and "
						+ UtilityFunctions::toString(newUB));
	}
}
