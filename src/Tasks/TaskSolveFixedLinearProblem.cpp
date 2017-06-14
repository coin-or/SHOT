#include "TaskSolveFixedLinearProblem.h"

TaskSolveFixedLinearProblem::TaskSolveFixedLinearProblem(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("PrimalBoundTotal");
	processInfo->startTimer("PrimalBoundFixedLP");

	discreteVariableIndexes = processInfo->originalProblem->getDiscreteVariableIndices();

	processInfo->stopTimer("PrimalBoundFixedLP");
	processInfo->stopTimer("PrimalBoundTotal");
}

TaskSolveFixedLinearProblem::~TaskSolveFixedLinearProblem()
{
	// TODO Auto-generated destructor stub
}

void TaskSolveFixedLinearProblem::run()
{
	processInfo->startTimer("PrimalBoundTotal");
	processInfo->startTimer("PrimalBoundFixedLP");
	auto currIter = processInfo->getCurrentIteration(); //The one not solved yet

	if (currIter->MILPSolutionLimitUpdated) return;

	if (currIter->iterationNumber < 5)
	{
		processInfo->stopTimer("PrimalBoundFixedLP");
		processInfo->stopTimer("PrimalBoundTotal");
		return;
	}

	if (currIter->maxDeviation <= settings->getDoubleSetting("SolveFixedLPConstrTol", "Algorithm"))
	{
		processInfo->stopTimer("PrimalBoundFixedLP");
		processInfo->stopTimer("PrimalBoundTotal");
		return;
	}

	auto prevIter = processInfo->getPreviousIteration();

	if (prevIter->iterationNumber < 4)
	{

		processInfo->stopTimer("PrimalBoundFixedLP");
		processInfo->stopTimer("PrimalBoundTotal");
		return;
	}

	auto prevIter2 = &processInfo->iterations.at(prevIter->iterationNumber - 2);
	auto prevIter3 = &processInfo->iterations.at(prevIter->iterationNumber - 3);

	if (!prevIter->isMILP() && !prevIter2->isMILP() && !prevIter3->isMILP())
	{
		processInfo->stopTimer("PrimalBoundFixedLP");
		processInfo->stopTimer("PrimalBoundTotal");
		return;
	}

	if (currIter->numHyperplanesAdded == 0)
	{
		processInfo->stopTimer("PrimalBoundFixedLP");
		processInfo->stopTimer("PrimalBoundTotal");
		return;
	}

	auto discreteIdxs = processInfo->originalProblem->getDiscreteVariableIndices();

	auto currSolPt = prevIter->solutionPoints.at(0).point;

	bool isDifferent1 = UtilityFunctions::isDifferentSelectedElements(currSolPt, prevIter2->solutionPoints.at(0).point,
			discreteIdxs);

	bool isDifferent2 = UtilityFunctions::isDifferentSelectedElements(currSolPt, prevIter3->solutionPoints.at(0).point,
			discreteIdxs);

	if (isDifferent1 || isDifferent2)
	{
		processInfo->stopTimer("PrimalBoundFixedLP");
		processInfo->stopTimer("PrimalBoundTotal");
		return;
	}

	std::vector<double> fixValues(discreteVariableIndexes.size());

	for (int i = 0; i < discreteVariableIndexes.size(); i++)
	{
		fixValues.at(i) = currSolPt.at(discreteVariableIndexes.at(i));
	}

	MILPSolver->fixVariables(discreteVariableIndexes, fixValues);

	int numVar = processInfo->originalProblem->getNumberOfVariables();

	bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();

	double prevObjVal = COIN_DBL_MAX;

	int iterLastObjUpdate = 0;
	int maxIter = settings->getIntSetting("SolveFixedLPMaxIter", "Algorithm");
	double objTol = settings->getDoubleSetting("SolveFixedLPObjTol", "Algorithm");
	double constrTol = settings->getDoubleSetting("SolveFixedLPConstrTol", "Algorithm");

	for (int k = 0; k < maxIter; k++)
	{
		auto solStatus = MILPSolver->solveProblem();

		if (solStatus != E_ProblemSolutionStatus::Optimal)
		{
			auto tmpLine = boost::format("%|4s| %|-10s| %|=10s| %|=44s|  %|-14s|") % k % "FIXLP INF" % "" % "" % "";
			processInfo->outputSummary(tmpLine.str());
			break;
		}
		else
		{
			auto varSol = MILPSolver->getVariableSolution(0);
			auto objVal = MILPSolver->getObjectiveValue(0);

			auto mostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(varSol);

			std::vector<double> externalPoint = varSol;
			std::vector<double> internalPoint = processInfo->interiorPts.at(0).point;

			try
			{
				auto xNewc = processInfo->linesearchMethod->findZero(internalPoint, externalPoint,
						settings->getIntSetting("LinesearchMaxIter", "Linesearch"),
						settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"),
						settings->getDoubleSetting("LinesearchConstrEps", "Linesearch"));

				processInfo->stopTimer("HyperplaneLinesearch");
				internalPoint = xNewc.first;
				externalPoint = xNewc.second;

				auto errorExternal = processInfo->originalProblem->getMostDeviatingConstraint(externalPoint);

				Hyperplane hyperplane;
				hyperplane.sourceConstraintIndex = errorExternal.idx;
				hyperplane.generatedPoint = externalPoint;
				hyperplane.source = E_HyperplaneSource::LPFixedIntegers;

				MILPSolver->createHyperplane(hyperplane);

			}
			catch (std::exception &e)
			{

				processInfo->outputWarning(
						"     Cannot find solution with linesearch for fixed LP, using solution point instead:");
				processInfo->outputWarning(e.what());
			}

			std::stringstream tmpType;

			bool hasSolution = true;

			bool isMIQP = (processInfo->originalProblem->getObjectiveFunctionType()
					== E_ObjectiveFunctionType::Quadratic);
			bool isMIQCP = (processInfo->originalProblem->getQuadraticConstraintIndexes().size() > 0);
			bool isDiscrete = false;

			if (isMIQCP) tmpType << "FIXQCP";
			else if (isMIQP) tmpType << "FIXQP";
			else tmpType << "FIXLP";

			if (varSol.size() == 0) hasSolution = false;

			if (solStatus == E_ProblemSolutionStatus::Error)
			{
				tmpType << " ERR";
				hasSolution = false;
			}
			else if (solStatus == E_ProblemSolutionStatus::Feasible)
			{
				tmpType << " FEA";
			}
			else if (solStatus == E_ProblemSolutionStatus::Infeasible)
			{
				tmpType << " INF";
				hasSolution = false;
			}
			else if (solStatus == E_ProblemSolutionStatus::IterationLimit)
			{
				tmpType << " ITL";
			}
			else if (solStatus == E_ProblemSolutionStatus::Optimal)
			{
				tmpType << " OPT";
			}
			else if (solStatus == E_ProblemSolutionStatus::TimeLimit)
			{
				tmpType << " TIL";
				hasSolution = false;
			}
			else if (solStatus == E_ProblemSolutionStatus::Unbounded)
			{
				tmpType << " UNB";
				hasSolution = false;
			}

			std::string hyperplanesExpr;

			auto numHyperTot = currIter->totNumHyperplanes;

			hyperplanesExpr = "+1 = " + to_string(numHyperTot);

			std::string tmpObjVal = ((boost::format("%.3f") % objVal).str());

			std::string tmpConstr;

			if (hasSolution && mostDevConstr.idx != -1)
			{
				tmpConstr = processInfo->originalProblem->getConstraintNames()[mostDevConstr.idx] + ": "
						+ ((boost::format("%.5f") % mostDevConstr.value).str());
			}
			else if (hasSolution)
			{
				tmpConstr = processInfo->originalProblem->getConstraintNames().back() + ": "
						+ ((boost::format("%.5f") % mostDevConstr.value).str());
			}
			else
			{
				tmpConstr = "";
			}

			std::string primalBoundExpr;
			std::string dualBoundExpr = "";

			auto primalBound = processInfo->getPrimalBound();
			auto dualBound = processInfo->getDualBound();

			if (primalBound != lastPrimalBound && processInfo->primalSolutions.size() > 0)
			{
				primalBoundExpr = UtilityFunctions::toString(primalBound);
				lastPrimalBound = primalBound;
			}
			else
			{
				primalBoundExpr = "";
			}

			if (mostDevConstr.value <= constrTol)
			{
				auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % k % "FIXLP CON"
						% hyperplanesExpr % "" % tmpObjVal % primalBoundExpr % tmpConstr;
				processInfo->outputSummary(tmpLine.str());
				break;
			}

			if (k - iterLastObjUpdate > 10)
			{
				auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % k % "FIXLP ITR"
						% hyperplanesExpr % "" % tmpObjVal % primalBoundExpr % tmpConstr;
				processInfo->outputSummary(tmpLine.str());
				break;
			}

			if (objVal > processInfo->getPrimalBound())
			{
				auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % k % "FIXLP PB "
						% hyperplanesExpr % "" % tmpObjVal % primalBoundExpr % tmpConstr;
				processInfo->outputSummary(tmpLine.str());

				break;
			}

			auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % k % tmpType.str()
					% hyperplanesExpr % "" % tmpObjVal % primalBoundExpr % tmpConstr;
			processInfo->outputSummary(tmpLine.str());

			if (abs(prevObjVal - objVal) > prevObjVal * objTol)
			{
				iterLastObjUpdate = k;
				prevObjVal = objVal;
			}
		}
	}

	MILPSolver->activateDiscreteVariables(true);

	MILPSolver->unfixVariables();

	processInfo->stopTimer("PrimalBoundFixedLP");
	processInfo->stopTimer("PrimalBoundTotal");
	return;
}

std::string TaskSolveFixedLinearProblem::getType()
{
	std::string type = typeid(this).name();
	return (type);
}

