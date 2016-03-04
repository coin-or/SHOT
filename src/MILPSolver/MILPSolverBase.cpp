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

	//int constrIdx = hyperplane.sourceConstraintIndex;
	//auto point = hyperplane.generatedPoint;

	if (hyperplane.sourceConstraintIndex != -1
			&& !originalProblem->isConstraintNonlinear(hyperplane.sourceConstraintIndex))
	{
		processInfo->logger.message(1) << CoinMessageNewline
				<< "Error: cutting plane added to linear constraint with index: " << hyperplane.sourceConstraintIndex
				<< CoinMessageNewline << CoinMessageEol;
	}

	auto varNames = originalProblem->getVariableNames();
	/*
	 processInfo->logger.message(1) << " HP point is: " << CoinMessageEol;


	 for (int i = 0; i < point.size(); i++)
	 {
	 processInfo->logger.message(3) << "  " << varNames.at(i) << ": " << point[i] << CoinMessageEol;
	 }*/

	double constant = originalProblem->calculateConstraintFunctionValue(hyperplane.sourceConstraintIndex,
			hyperplane.generatedPoint);

	if (hyperplane.sourceConstraintIndex == -1
			|| hyperplane.sourceConstraintIndex == originalProblem->getNonlinearObjectiveConstraintIdx())
	{
		processInfo->logger.message(3) << " HP point generated for auxiliary objective function constraint"
				<< CoinMessageEol;

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

				processInfo->logger.message(3) << " Gradient for variable" << varNames.at(i) << ": " << tmpArray[i]
						<< CoinMessageEol;
			}
		}

		processInfo->logger.message(3) << " Gradient for obj.var.: -1" << CoinMessageEol;

		IndexValuePair pair;
		pair.idx = processInfo->originalProblem->getNonlinearObjectiveVariableIdx();
		pair.value = -1.0;

		elements.push_back(pair);
		constant += /*-(-1) **/hyperplane.generatedPoint.at(pair.idx);
	}
	else
	{
		processInfo->logger.message(3) << " HP point generated for constraint index" << hyperplane.sourceConstraintIndex
				<< CoinMessageEol;

		auto nablag = originalProblem->calculateConstraintFunctionGradient(hyperplane.sourceConstraintIndex,
				hyperplane.generatedPoint);

		for (int i = 0; i < nablag->number; i++)
		{
			IndexValuePair pair;
			pair.idx = nablag->indexes[i];
			pair.value = nablag->values[i];

			elements.push_back(pair);
			constant += -nablag->values[i] * hyperplane.generatedPoint.at(nablag->indexes[i]);

			processInfo->logger.message(3) << " Gradient for variable" << varNames.at(nablag->indexes[i]) << ": "
					<< nablag->values[i] << CoinMessageEol;
		}
	}

	/*
	 for (auto E : elements)
	 {
	 processInfo->logger.message(3) << " HP coefficient for variable " << varNames.at(E.idx) << ": " << E.value
	 << CoinMessageEol;
	 }

	 processInfo->logger.message(3) << " HP constant " << constant << CoinMessageEol;
	 */

	bool hyperplaneIsOk = true;

	for (auto E : elements)
	{
		if (E.value != E.value) //Check for NaN
		{
			processInfo->logger.message(0) << "Warning: hyperplane not generated, NaN found in linear terms!"
					<< CoinMessageEol;
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

	currIter->totNumHyperplanes = processInfo->getPreviousIteration()->totNumHyperplanes
			+ currIter->numHyperplanesAdded;
}

void MILPSolverBase::createInteriorHyperplane(Hyperplane hyperplane)
{
	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration
	auto originalProblem = processInfo->originalProblem;
	std::vector < IndexValuePair > elements;

	auto varNames = originalProblem->getVariableNames();

	double constant = originalProblem->calculateConstraintFunctionValue(hyperplane.sourceConstraintIndex,
			hyperplane.generatedPoint);

	//processInfo->logger.message(1) << " HP point generated for auxiliary objective function constraint"
	//		<< CoinMessageEol;

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

			//processInfo->logger.message(3) << " Gradient for variable" << varNames.at(i) << ": " << tmpArray[i]
			//		<< CoinMessageEol;
		}
	}

	//processInfo->logger.message(1) << "   Gradient for obj.var.: -1" << CoinMessageEol;

	IndexValuePair pair;
	pair.idx = processInfo->originalProblem->getNonlinearObjectiveVariableIdx();
	pair.value = -1.0;

	elements.push_back(pair);
	constant += hyperplane.generatedPoint.at(pair.idx);

	//processInfo->logger.message(1) << "    HP constant " << constant << CoinMessageEol;
	bool hyperplaneIsOk = true;

	for (auto E : elements)
	{
		if (E.value != E.value) //Check for NaN
		{
			processInfo->logger.message(0) << "Warning: hyperplane not generated, NaN found in linear terms!"
					<< CoinMessageEol;
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
