#include <TaskSolveFixedLinearProblem.h>

/*
 class Test4
 {
 private:
 OptProblemOriginal *originalProblem;

 public:
 std::vector<double> firstPt;
 std::vector<double> secondPt;
 Test4(OptProblemOriginal *prob)
 {
 originalProblem = prob;
 }

 double operator()(const double x)
 {
 int length = firstPt.size();
 std::vector<double> ptNew(length);

 for (int i = 0; i < length; i++)
 {
 ptNew.at(i) = x * firstPt.at(i) + (1 - x) * secondPt.at(i);
 }

 auto validNewPt = originalProblem->getMostDeviatingConstraint(ptNew).value;

 return (validNewPt);

 }
 };

 class TerminationCondition4
 {
 private:
 double tol;

 public:
 TerminationCondition4(double tolerance)
 {
 tol = tolerance;
 }

 bool operator()(double min, double max)
 {
 return (abs(min - max) <= tol);
 }
 };*/

TaskSolveFixedLinearProblem::TaskSolveFixedLinearProblem()
{
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

	/*if (processInfo->primalSolution.size() == 0) return;

	 if (processInfo->primalSolutions.back().iterFound != currIter->iterationNumber - 1) return;

	 if (processInfo->primalSolutions.back().sourceType == E_PrimalSolutionSource::LPFixedIntegers) return;

	 auto primalSolution = processInfo->primalSolutions.back();
	 if (primalSolution.maxDevatingConstraint.value > 0) return;
	 */

	vector < pair<double, double> > originalBounds(discreteVariableIndexes.size());

	processInfo->MILPSolver->activateDiscreteVariables(false);

	for (int i = 0; i < discreteVariableIndexes.size(); i++)
	{
		originalBounds.at(i) = processInfo->MILPSolver->getCurrentVariableBounds(discreteVariableIndexes.at(i));
		processInfo->MILPSolver->fixVariable(discreteVariableIndexes.at(i),
				currSolPt.at(discreteVariableIndexes.at(i)));
	}

	//double tol = max(settings->getDoubleSetting("LinesearchLambdaEps", "Linesearch"), 0.000000000000001);
	//double tol = settings->getDoubleSetting("LinesearchEps", "Linesearch");
	//boost::uintmax_t N = settings->getIntSetting("LinesearchMaxIter", "Linesearch");
	//double maxDev = max(0.00001, settings->getDoubleSetting("ConstrTermTolMILP", "Algorithm"));
	int numVar = processInfo->originalProblem->getNumberOfVariables();

	//Test4 t(processInfo->originalProblem);

	bool isMinimization = processInfo->originalProblem->isTypeOfObjectiveMinimize();

	//processInfo->outputSummary("─────────────────────────────────────────────────────────────────────────────────────");

	double prevObjVal = COIN_DBL_MAX;

	int iterLastObjUpdate = 0;
	int maxIter = settings->getIntSetting("SolveFixedLPMaxIter", "Algorithm");
	double objTol = settings->getDoubleSetting("SolveFixedLPObjTol", "Algorithm");
	double constrTol = settings->getDoubleSetting("SolveFixedLPConstrTol", "Algorithm");

	for (int k = 0; k < maxIter; k++)
	{
		auto solStatus = processInfo->MILPSolver->solveProblem();

		if (solStatus != E_ProblemSolutionStatus::Optimal)
		{
			auto tmpLine = boost::format("%|4s| %|-10s| %|=10s| %|=44s|  %|-14s|") % k % "FIXLP INF" % "" % "" % "";
			processInfo->outputSummary(tmpLine.str());
			break;
		}
		else
		{
			std::stringstream tmpType;

			auto varSol = processInfo->MILPSolver->getVariableSolution(0);
			auto objVal = processInfo->MILPSolver->getObjectiveValue(0);

			auto mostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(varSol);

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
			/*else if (solStatus == E_ProblemSolutionStatus::SolutionLimit)
			 {
			 tmpType << " SL";
			 tmpType << std::to_string(currIter->usedMILPSolutionLimit);
			 }*/
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

			if (mostDevConstr.value <= constrTol)
			{
				auto tmpLine = boost::format("%|4s| %|-10s| %|=10s| %|=44s|  %|-14s|") % k % "FIXLP CON"
						% hyperplanesExpr % tmpObjVal % tmpConstr;
				processInfo->outputSummary(tmpLine.str());
				break;
			}

			if (k - iterLastObjUpdate > 10)
			{
				auto tmpLine = boost::format("%|4s| %|-10s| %|=10s| %|=44s|  %|-14s|") % k % "FIXLP ITR"
						% hyperplanesExpr % tmpObjVal % tmpConstr;
				processInfo->outputSummary(tmpLine.str());
				break;
			}

			if (objVal > processInfo->getPrimalBound())
			{
				auto tmpLine = boost::format("%|4s| %|-10s| %|=10s| %|=44s|  %|-14s|") % k % "FIXLP PB "
						% hyperplanesExpr % tmpObjVal % tmpConstr;
				processInfo->outputSummary(tmpLine.str());

				break;
			}

			auto tmpLine = boost::format("%|4s| %|-10s| %|=10s| %|=44s|  %|-14s|") % k % tmpType.str() % hyperplanesExpr
					% tmpObjVal % tmpConstr;
			processInfo->outputSummary(tmpLine.str());

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

				//processInfo->addPrimalSolutionCandidate(internalPoint, E_PrimalSolutionSource::LPFixedIntegers,
				//currIter->iterationNumber);

				auto errorExternal = processInfo->originalProblem->getMostDeviatingConstraint(externalPoint);

				Hyperplane hyperplane;
				hyperplane.sourceConstraintIndex = errorExternal.idx;
				hyperplane.generatedPoint = externalPoint;
				hyperplane.source = E_HyperplaneSource::LPFixedIntegers;

				processInfo->MILPSolver->createHyperplane(hyperplane);

			}
			catch (std::exception &e)
			{

				processInfo->outputWarning(
						"Cannot find solution with linesearch for fixed LP, using solution point instead:");
				processInfo->outputWarning(e.what());
			}

			if (abs(prevObjVal - objVal) > prevObjVal * objTol)
			{
				iterLastObjUpdate = k;
				prevObjVal = objVal;
			}

			/*if (k == 0)
			 {
			 prevObjVal = objVal;
			 iterLastObjUpdate = 0;
			 }*/
		}

	}

	processInfo->MILPSolver->activateDiscreteVariables(true);

	for (int i = 0; i < discreteVariableIndexes.size(); i++)
	{
		processInfo->MILPSolver->updateVariableBound(discreteVariableIndexes.at(i), originalBounds.at(i).first,
				originalBounds.at(i).second);
	}

	//processInfo->outputSummary("─────────────────────────────────────────────────────────────────────────────────────");

	processInfo->stopTimer("PrimalBoundFixedLP");
	processInfo->stopTimer("PrimalBoundTotal");
	return;
}

std::string TaskSolveFixedLinearProblem::getType()
{
	std::string type = typeid(this).name();
	return (type);
}

