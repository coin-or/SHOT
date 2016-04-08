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
	if (settings->getIntSetting("LinesearchMethod", "Linesearch") == static_cast<int>(ES_LinesearchMethod::Boost))
	{
		processInfo->logger.message(2) << "Boost linesearch implementation selected for primal heuristics"
				<< CoinMessageEol;
		linesearchMethod = new LinesearchMethodBoost();
	}
	else if (settings->getIntSetting("LinesearchMethod", "Linesearch")
			== static_cast<int>(ES_LinesearchMethod::Bisection))
	{
		processInfo->logger.message(2) << "Bisection linesearch selected primal heuristics" << CoinMessageEol;
		linesearchMethod = new LinesearchMethodBisection();
	}

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

	double prevObjVal;

	int maxIter = settings->getIntSetting("SolveFixedLPMaxIter", "Algorithm");
	int objTol = settings->getDoubleSetting("SolveFixedLPObjTol", "Algorithm");
	double constrTol = settings->getDoubleSetting("SolveFixedLPConstrTol", "Algorithm");

	for (int k = 0; k < maxIter; k++)
	{
		auto solStatus = processInfo->MILPSolver->solveProblem();

		if (solStatus != E_ProblemSolutionStatus::Optimal)
		{
			//std::cout << "nonoptimal" << std::endl;
			break;
		}
		else
		{
			auto varSol = processInfo->MILPSolver->getVariableSolution(0);
			auto objVal = processInfo->MILPSolver->getObjectiveValue(0);

			auto mostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(varSol);

			//bool hasSolution = true;
			std::stringstream tmpType;

			if (processInfo->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic
					|| processInfo->originalProblem->getQuadraticConstraintIndexes().size() > 0)
			{
				tmpType << "Q";
			}
			else
			{
				tmpType << "L";
			}

			tmpType << " X";

			if (solStatus == E_ProblemSolutionStatus::Error)
			{
				tmpType << " E";
				//hasSolution = false;
			}
			else if (solStatus == E_ProblemSolutionStatus::Feasible)
			{
				tmpType << " F";
			}
			else if (solStatus == E_ProblemSolutionStatus::Infeasible)
			{
				tmpType << " I";
				//hasSolution = false;
			}
			else if (solStatus == E_ProblemSolutionStatus::IterationLimit)
			{
				tmpType << " IL";
			}
			else if (solStatus == E_ProblemSolutionStatus::Optimal)
			{
				tmpType << " O";
			}
			else if (solStatus == E_ProblemSolutionStatus::SolutionLimit)
			{
				tmpType << " SL";
			}
			else if (solStatus == E_ProblemSolutionStatus::TimeLimit)
			{
				tmpType << " TL";
			}
			else if (solStatus == E_ProblemSolutionStatus::Unbounded)
			{
				tmpType << " UB";
				//hasSolution = false;
			}

			auto tmpLine = boost::format(
					"%1% %|4t|%2% %|10t|%3% %|14t|+%4% = %5% %|24t|%6% %|38t|%7% %|46t|%8%: %|54t|%9% %|70t|%10%") % k
					% tmpType.str() % " " % "1" % (currIter->totNumHyperplanes + 1) % objVal % " " % " "
					% mostDevConstr.value % "";

			processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;

			if (mostDevConstr.value <= constrTol)
			{
				//std::cout << "Constr break" << std::endl;
				break;
			}

			if (k > 0 && abs(prevObjVal - objVal) < objTol)
			{
				//std::cout << "Obj break" << std::endl;
				break;
			}

			std::vector<double> externalPoint = varSol;
			std::vector<double> internalPoint = processInfo->interiorPts.at(0).point;

			try
			{
				auto xNewc = linesearchMethod->findZero(internalPoint, externalPoint,
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

				processInfo->logger.message(0)
						<< "Cannot find solution with linesearch for fixed LP, using solution point instead: "
						<< CoinMessageNewline << e.what() << CoinMessageEol;
			}

			if (k == 0) prevObjVal = objVal;
		}

	}

	processInfo->MILPSolver->activateDiscreteVariables(true);

	for (int i = 0; i < discreteVariableIndexes.size(); i++)
	{
		processInfo->MILPSolver->updateVariableBound(discreteVariableIndexes.at(i), originalBounds.at(i).first,
				originalBounds.at(i).second);
	}

	processInfo->stopTimer("PrimalBoundFixedLP");
	processInfo->stopTimer("PrimalBoundTotal");
	return;
}

std::string TaskSolveFixedLinearProblem::getType()
{
	std::string type = typeid(this).name();
	return (type);
}

void TaskSolveFixedLinearProblem::printIterationReport()
{
	auto currIter = processInfo->getCurrentIteration();

	try
	{
		std::string tmpboundaryDistance = " ";

		std::stringstream tmpType;

		bool hasSolution = true;

		if (processInfo->originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic
				|| processInfo->originalProblem->getQuadraticConstraintIndexes().size() > 0)
		{
			tmpType << "Q";
		}
		else
		{
			tmpType << "L";
		}

		tmpType << " X";

		if (currIter->solutionStatus == E_ProblemSolutionStatus::Error)
		{
			tmpType << " E";
			hasSolution = false;
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Feasible)
		{
			tmpType << " F";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Infeasible)
		{
			tmpType << " I";
			hasSolution = false;
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::IterationLimit)
		{
			tmpType << " IL";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
		{
			tmpType << " O";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
		{
			tmpType << " SL";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::TimeLimit)
		{
			tmpType << " TL";
		}
		else if (currIter->solutionStatus == E_ProblemSolutionStatus::Unbounded)
		{
			tmpType << " UB";
			hasSolution = false;
		}

		std::string solLimit = (currIter->isMILP() ? std::to_string(currIter->usedMILPSolutionLimit) : " ");

		std::string tmpConstr;

		if (hasSolution && currIter->maxDeviationConstraint != -1)
		{
			tmpConstr = processInfo->originalProblem->getConstraintNames()[currIter->maxDeviationConstraint];
		}
		else if (hasSolution)
		{
			tmpConstr = processInfo->originalProblem->getConstraintNames().back();
		}
		else
		{
			tmpConstr = "";
		}

		auto tmpLine = boost::format(
				"%1% %|4t|%2% %|10t|%3% %|14t|+%4% = %5% %|24t|%6% %|38t|%7% %|46t|%8%: %|54t|%9% %|70t|%10%")
				% currIter->iterationNumber % tmpType.str() % solLimit % currIter->numHyperplanesAdded
				% currIter->totNumHyperplanes % currIter->objectiveValue % currIter->usedConstraintTolerance % tmpConstr
				% currIter->maxDeviation % tmpboundaryDistance;

		processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;
	}
	catch (...)
	{
		processInfo->logger.message(1) << "ERROR, cannot write iteration solution report!" << CoinMessageEol;
	}
}
