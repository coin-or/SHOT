#include "MILPSolverBase.h"
#include "../UtilityFunctions.h"

MILPSolverBase::MILPSolverBase()
{
}

MILPSolverBase::~MILPSolverBase()
{
}

void MILPSolverBase::startTimer()
{
	if (discreteVariablesActivated) ProcessInfo::getInstance().startTimer("MILP");
	else ProcessInfo::getInstance().startTimer("LP");
}

void MILPSolverBase::stopTimer()
{
	if (discreteVariablesActivated) ProcessInfo::getInstance().stopTimer("MILP");
	else ProcessInfo::getInstance().stopTimer("LP");
}

double MILPSolverBase::getObjectiveValue()
{
	double objval = getObjectiveValue(0);
	return (objval);
}

bool MILPSolverBase::getDiscreteVariableStatus()
{
	if (ProcessInfo::getInstance().originalProblem->getNumberOfDiscreteVariables() == 0)
	{
		return (false);
	}
	else
	{
		return (discreteVariablesActivated);
	}
}

std::vector<SolutionPoint> MILPSolverBase::getAllVariableSolutions()
{
	if (cachedSolutionHasChanged == false) return (lastSolutions);

	int numSol = getNumberOfSolutions();

	int numVar = originalProblem->getNumberOfVariables();
	std::vector < SolutionPoint > allSolutions(numSol);

	// Should be moved to separate task
	bool isMILP = getDiscreteVariableStatus();
	if (isMILP && Settings::getInstance().getBoolSetting("MIP.SolutionPool.Populate", "Dual"))
	{
		populateSolutionPool();
	}

	for (int i = 0; i < numSol; i++)
	{
		SolutionPoint tmpSolPt;

		auto tmpPt = getVariableSolution(i);

		auto maxDev = originalProblem->getMostDeviatingConstraint(tmpPt);

		tmpSolPt.point = tmpPt;
		tmpSolPt.objectiveValue = getObjectiveValue(i);
		tmpSolPt.iterFound = ProcessInfo::getInstance().getCurrentIteration()->iterationNumber;
		tmpSolPt.maxDeviation = maxDev;

		allSolutions.at(i) = tmpSolPt;

	}

	cachedSolutionHasChanged = false;
	lastSolutions = allSolutions;

	return (lastSolutions);
}

/*
 void MILPSolverBase::createHyperplane(Hyperplane hyperplane)
 {
 auto fp = std::bind(&addLinearConstraint, _1, _2);

 createHyperplane(hyperplane, fp);
 }*/

void MILPSolverBase::createHyperplane(Hyperplane hyperplane)
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration

	auto optional = createHyperplaneTerms(hyperplane);

	if (!optional)
	{
		return;
	}

	auto tmpPair = optional.get();

	bool hyperplaneIsOk = true;

	for (auto E : tmpPair.first)
	{
		if (E.value != E.value) //Check for NaN
		{
			std::cout << E.value << std::endl;
			ProcessInfo::getInstance().outputWarning(
					"     Warning: hyperplane not generated, NaN found in linear terms!");
			hyperplaneIsOk = false;
			break;
		}
	}

	if (hyperplaneIsOk)
	{
		GeneratedHyperplane genHyperplane;

		int constrIndex = addLinearConstraint(tmpPair.first, tmpPair.second);

		genHyperplane.generatedConstraintIndex = constrIndex;
		genHyperplane.sourceConstraintIndex = hyperplane.sourceConstraintIndex;
		genHyperplane.generatedPoint = hyperplane.generatedPoint;
		genHyperplane.source = hyperplane.source;
		genHyperplane.generatedIter = currIter->iterationNumber;
		genHyperplane.isLazy = false;
		genHyperplane.isRemoved = false;

		generatedHyperplanes.push_back(genHyperplane);

		currIter->numHyperplanesAdded++;
		currIter->totNumHyperplanes++;
	}
}

boost::optional<std::pair<std::vector<IndexValuePair>, double>> MILPSolverBase::createHyperplaneTerms(
		Hyperplane hyperplane)
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration
	std::vector < IndexValuePair > elements;

	auto varNames = originalProblem->getVariableNames();

	double constant = originalProblem->calculateConstraintFunctionValue(hyperplane.sourceConstraintIndex,
			hyperplane.generatedPoint);

	auto nablag = originalProblem->calculateConstraintFunctionGradient(hyperplane.sourceConstraintIndex,
			hyperplane.generatedPoint);

	ProcessInfo::getInstance().outputInfo(
			"     HP point generated for constraint index " + to_string(hyperplane.sourceConstraintIndex) + " with "
					+ to_string(nablag->number) + " elements.");

	for (int i = 0; i < nablag->number; i++)
	{
		IndexValuePair pair;
		pair.idx = nablag->indexes[i];
		pair.value = nablag->values[i];

		elements.push_back(pair);

		constant += -nablag->values[i] * hyperplane.generatedPoint.at(nablag->indexes[i]);

		ProcessInfo::getInstance().outputInfo(
				"     Gradient for variable " + varNames.at(nablag->indexes[i]) + ": " + to_string(nablag->values[i]));
	}

	boost::optional<std::pair<std::vector<IndexValuePair>, double>> optional;
	if (elements.size() > 0) optional = std::make_pair(elements, constant);

	return (optional);
}

void MILPSolverBase::createInteriorHyperplane(Hyperplane hyperplane)
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); // The unsolved new iteration
	std::vector < IndexValuePair > elements;

	auto varNames = originalProblem->getVariableNames();

	double constant = originalProblem->calculateConstraintFunctionValue(hyperplane.sourceConstraintIndex,
			hyperplane.generatedPoint);

	auto tmpArray = originalProblem->getProblemInstance()->calculateObjectiveFunctionGradient(
			&hyperplane.generatedPoint.at(0), -1, true);
	int number = originalProblem->getNumberOfVariables();
	ProcessInfo::getInstance().numGradientEvals++;

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
	pair.idx = originalProblem->getNonlinearObjectiveVariableIdx();
	pair.value = -1.0;

	elements.push_back(pair);
	constant += hyperplane.generatedPoint.at(pair.idx);

	bool hyperplaneIsOk = true;

	for (auto E : elements)
	{
		if (E.value != E.value) //Check for NaN
		{
			ProcessInfo::getInstance().outputWarning(
					"     Warning: hyperplane not generated, NaN found in linear terms!");

			hyperplaneIsOk = false;
			break;
		}
	}

	if (hyperplaneIsOk)
	{
		int constrIndex = addLinearConstraint(elements, constant, false);
		//addedHyperplanes++;
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

	currIter->totNumHyperplanes = ProcessInfo::getInstance().getPreviousIteration()->totNumHyperplanes
			+ currIter->numHyperplanesAdded;
}

std::vector<GeneratedHyperplane> *MILPSolverBase::getGeneratedHyperplanes()
{
	return (&generatedHyperplanes);
}

void MILPSolverBase::presolveAndUpdateBounds()
{
	auto newBounds = this->presolveAndGetNewBounds();

	auto numVar = originalProblem->getNumberOfVariables();

	for (int i = 0; i < numVar; i++)
	{
		auto currBounds = this->getCurrentVariableBounds(i);

		bool newLB = false;
		bool newUB = false;

		if (newBounds.first.at(i) > currBounds.first) newLB = true;
		if (newBounds.second.at(i) > currBounds.second) newUB = true;

		if (newLB)
		{
			originalProblem->setVariableUpperBound(i, newBounds.second.at(i));
			//ProcessInfo::getInstance().originalProblem->setVariableUpperBound(i, newBounds.second.at(i));
			ProcessInfo::getInstance().outputInfo(
					"     Lower bound for variable (" + to_string(i) + ") updated from "
							+ UtilityFunctions::toString(currBounds.first) + " to "
							+ UtilityFunctions::toString(newBounds.first.at(i)));

			if (!originalProblem->hasVariableBoundsBeenTightened(i))
			{
				//ProcessInfo::getInstance().originalProblem->setVariableBoundsAsTightened(i);
				originalProblem->setVariableBoundsAsTightened(i);
				ProcessInfo::getInstance().numVariableBoundsTightenedInPresolve++;
			}
		}

		if (newUB)
		{
			originalProblem->setVariableUpperBound(i, newBounds.second.at(i));
			//ProcessInfo::getInstance().originalProblem->setVariableUpperBound(i, newBounds.second.at(i));
			ProcessInfo::getInstance().outputInfo(
					"     Upper bound for variable (" + to_string(i) + ") updated from "
							+ UtilityFunctions::toString(currBounds.second) + " to "
							+ UtilityFunctions::toString(newBounds.second.at(i)));

			if (!originalProblem->hasVariableBoundsBeenTightened(i))
			{
				//ProcessInfo::getInstance().originalProblem->setVariableBoundsAsTightened(i);
				originalProblem->setVariableBoundsAsTightened(i);
				ProcessInfo::getInstance().numVariableBoundsTightenedInPresolve++;
			}
		}

		if (Settings::getInstance().getBoolSetting("MIP.Presolve.UpdateObtainedBounds", "Dual") && (newLB || newUB))
		{
			updateVariableBound(i, newBounds.first.at(i), newBounds.second.at(i));
			ProcessInfo::getInstance().outputInfo("     Bounds updated also in MIP problem");
		}
	}
}

void MILPSolverBase::fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues)
{
	if (isVariablesFixed)
	{
		unfixVariables();
	}

	int size = variableIndexes.size();

	if (size == 0) return;

	vector < pair<double, double> > originalBounds(size);

	activateDiscreteVariables(false);

	for (int i = 0; i < size; i++)
	{
		originalBounds.at(i) = this->getCurrentVariableBounds(variableIndexes.at(i));
		this->fixVariable(variableIndexes.at(i), variableValues.at(i));
	}

	fixedVariableIndexes = variableIndexes;
	fixedVariableOriginalBounds = originalBounds;

	isVariablesFixed = true;
}

void MILPSolverBase::unfixVariables()
{
	for (int i = 0; i < fixedVariableIndexes.size(); i++)
	{
		updateVariableBound(fixedVariableIndexes.at(i), fixedVariableOriginalBounds.at(i).first,
				fixedVariableOriginalBounds.at(i).second);
	}

	isVariablesFixed = false;
}

void MILPSolverBase::updateNonlinearObjectiveFromPrimalDualBounds()
{
	if (!originalProblem->isObjectiveFunctionNonlinear())
	{
		return;
	}

	auto varIdx = originalProblem->getNonlinearObjectiveVariableIdx();

	auto newLB = ProcessInfo::getInstance().getDualBound();
	auto newUB = ProcessInfo::getInstance().getPrimalBound();

	auto currBounds = this->getCurrentVariableBounds(varIdx);

	if (newLB > currBounds.first || newUB < currBounds.second)
	{
		this->updateVariableBound(varIdx, newLB, newUB);
		ProcessInfo::getInstance().outputInfo(
				"     Bounds for nonlinear objective function updated to " + UtilityFunctions::toString(newLB) + " and "
						+ UtilityFunctions::toString(newUB));
	}
}

void MILPSolverBase::createIntegerCut(std::vector<int> binaryIndexes)
{
	std::vector < IndexValuePair > elements;

	for (int i = 0; i < binaryIndexes.size(); i++)
	{
		IndexValuePair pair;
		pair.idx = binaryIndexes.at(i);
		pair.value = 1.0;

		elements.push_back(pair);
	}

	this->addLinearConstraint(elements, -(binaryIndexes.size() - 1.0));
	ProcessInfo::getInstance().numIntegerCutsAdded++;
}
