/*
 * TaskAddHyperplanes.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: alundell
 */

#include <TaskAddHyperplanes.h>

TaskAddHyperplanes::TaskAddHyperplanes()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	itersWithoutAddedHPs = 0;
}

TaskAddHyperplanes::~TaskAddHyperplanes()
{
	// TODO Auto-generated destructor stub
}

void TaskAddHyperplanes::run()
{
	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration

	//if (currIter->isMILP() && true) return; //Use with lazy + CPLEX

	if (!currIter->isMILP() || !settings->getBoolSetting("DelayedConstraints", "MILP")
			|| !currIter->MILPSolutionLimitUpdated || itersWithoutAddedHPs > 5)
	{
		int addedHyperplanes = 0;

		for (int k = processInfo->hyperplaneWaitingList.size(); k > 0; k--)
		{
			if (addedHyperplanes >= settings->getIntSetting("MaxHyperplanesPerIteration", "Algorithm")) break;

			auto tmpItem = processInfo->hyperplaneWaitingList.at(k - 1);

			if (tmpItem.source == E_HyperplaneSource::PrimalSolutionSearchInteriorObjective)
			{
				processInfo->MILPSolver->createInteriorHyperplane(tmpItem);
			}
			else
			{
				processInfo->MILPSolver->createHyperplane(tmpItem);
				addedHyperplanes++;
			}
		}

		processInfo->hyperplaneWaitingList.clear();
		itersWithoutAddedHPs = 0;
	}
	else
	{
		itersWithoutAddedHPs++;
	}
}

void TaskAddHyperplanes::createHyperplane(int constrIdx, std::vector<double> point)
{
	auto currIter = processInfo->getCurrentIteration(); // The unsolved new iteration
	auto originalProblem = processInfo->originalProblem;
	std::vector < IndexValuePair > elements;

	if (constrIdx != -1 && !originalProblem->isConstraintNonlinear(constrIdx))
	{
		processInfo->outputWarning("Cutting plane added to linear constraint with index: " + to_string(constrIdx));
	}

	auto varNames = originalProblem->getVariableNames();

	double constant = originalProblem->calculateConstraintFunctionValue(constrIdx, point);

	if (constrIdx == -1 || constrIdx == originalProblem->getNonlinearObjectiveConstraintIdx())
	{
		processInfo->outputInfo("HP point generated for auxiliary objective function constraint");

		auto tmpArray = processInfo->originalProblem->getProblemInstance()->calculateObjectiveFunctionGradient(
				&point.at(0), -1, true);
		processInfo->numGradientEvals++;

		int number = processInfo->originalProblem->getNumberOfVariables();

		for (int i = 0; i < number - 1; i++)
		{
			if (tmpArray[i] != 0)
			{
				IndexValuePair pair;
				pair.idx = i;
				pair.value = tmpArray[i];

				elements.push_back(pair);
				constant += -tmpArray[i] * point.at(i);

				processInfo->outputInfo("Gradient for variable" + varNames.at(i) + ": " + to_string(tmpArray[i]));
			}
		}

		processInfo->outputInfo("Gradient for obj.var.: -1");

		IndexValuePair pair;
		pair.idx = processInfo->originalProblem->getNonlinearObjectiveVariableIdx();
		pair.value = -1.0;

		elements.push_back(pair);
		constant += /*-(-1) **/point.at(pair.idx);
	}
	else
	{
		processInfo->outputInfo("HP point generated for constraint index" + to_string(constrIdx));

		auto nablag = originalProblem->calculateConstraintFunctionGradient(constrIdx, point);

		for (int i = 0; i < nablag->number; i++)
		{
			IndexValuePair pair;
			pair.idx = nablag->indexes[i];
			pair.value = nablag->values[i];

			elements.push_back(pair);
			constant += -nablag->values[i] * point.at(nablag->indexes[i]);

			processInfo->outputInfo(
					"Gradient for variable" + varNames.at(nablag->indexes[i]) + ": " + to_string(nablag->indexes[i]));
		}
	}

	bool hyperplaneIsOk = true;

	for (auto E : elements)
	{
		if (E.value != E.value) //Check for NaN
		{

			processInfo->outputError("Warning: hyperplane not generated, NaN found in linear terms!");
			hyperplaneIsOk = false;
			break;
		}
	}

	if (hyperplaneIsOk)
	{
		processInfo->MILPSolver->addLinearConstraint(elements, constant);

		currIter->numHyperplanesAdded++;
		currIter->totNumHyperplanes++;
	}

	if (settings->getBoolSetting("Debug", "SHOTSolver"))
	{
		stringstream ss;
		ss << settings->getStringSetting("DebugPath", "SHOTSolver");
		ss << "/lp";
		ss << currIter->iterationNumber - 1;
		ss << ".lp";
		processInfo->MILPSolver->writeProblemToFile(ss.str());
	}

	currIter->totNumHyperplanes = processInfo->getPreviousIteration()->totNumHyperplanes
			+ currIter->numHyperplanesAdded;
}

std::string TaskAddHyperplanes::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

