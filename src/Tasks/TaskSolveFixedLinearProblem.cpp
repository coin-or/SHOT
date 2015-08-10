#include <TaskSolveFixedLinearProblem.h>

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
};

TaskSolveFixedLinearProblem::TaskSolveFixedLinearProblem()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	processInfo->startTimer("PrimalBoundTotal");
	processInfo->startTimer("PrimalBoundLinesearch");
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

	processInfo->stopTimer("PrimalBoundLinesearch");
	processInfo->stopTimer("PrimalBoundTotal");
}

TaskSolveFixedLinearProblem::~TaskSolveFixedLinearProblem()
{
	// TODO Auto-generated destructor stub
}

void TaskSolveFixedLinearProblem::run()
{
	auto currIter = processInfo->getCurrentIteration();
	if (processInfo->primalSolution.size() == 0) return;

	if (processInfo->primalSolutions.back().iterFound != currIter->iterationNumber - 1) return;

	if (processInfo->primalSolutions.back().sourceType == E_PrimalSolutionSource::LPFixedIntegers) return;

	auto primalSolution = processInfo->primalSolutions.back();
	if (primalSolution.maxDevatingConstraint.value > 0) return;

	vector < pair<double, double> > originalBounds(discreteVariableIndexes.size());

	processInfo->MILPSolver->activateDiscreteVariables(false);

	for (int i = 0; i < discreteVariableIndexes.size(); i++)
	{
		originalBounds.at(i) = processInfo->MILPSolver->getCurrentVariableBounds(discreteVariableIndexes.at(i));
		processInfo->MILPSolver->fixVariable(discreteVariableIndexes.at(i),
				primalSolution.point.at(discreteVariableIndexes.at(i)));
	}

	double tol = settings->getDoubleSetting("LinesearchEps", "Linesearch");
	boost::uintmax_t N = settings->getIntSetting("LinesearchMaxIter", "Linesearch");
	int numVar = processInfo->originalProblem->getNumberOfVariables();

	Test4 t(processInfo->originalProblem);

	double prevObjVal;

	for (int k = 0; k < 10; k++)
	{

		auto solStatus = processInfo->MILPSolver->solveProblem();

		if (solStatus != E_ProblemSolutionStatus::Optimal)
		{
			std::cout << "nonoptimal" << std::endl;
			break;
		}
		else
		{
			auto varSol = processInfo->MILPSolver->getVariableSolution(0);
			auto objVal = processInfo->MILPSolver->getObjectiveValue();

			auto mostDevConstr = processInfo->originalProblem->getMostDeviatingConstraint(varSol);

			bool hasSolution = true;
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
				hasSolution = false;
			}
			else if (solStatus == E_ProblemSolutionStatus::Feasible)
			{
				tmpType << " F";
			}
			else if (solStatus == E_ProblemSolutionStatus::Infeasible)
			{
				tmpType << " I";
				hasSolution = false;
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
				hasSolution = false;
			}

			auto tmpLine = boost::format(
					"%1% %|4t|%2% %|10t|%3% %|14t|+%4% = %5% %|24t|%6% %|38t|%7% %|46t|%8%: %|54t|%9% %|70t|%10%") % k
					% tmpType.str() % " " % "1" % currIter->totNumHyperplanes % objVal % " " % " " % mostDevConstr.value
					% "";

			processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;

			if (mostDevConstr.value <= 0.00001) break;

			if (k > 0 && abs(prevObjVal - objVal) < 0.0001) break;

			t.firstPt = varSol;
			t.secondPt = primalSolution.point;

			typedef std::pair<double, double> Result;
			boost::uintmax_t max_iter = N;
			Result r1 = boost::math::tools::toms748_solve(t, 0.0, 1.0, TerminationCondition4(tol), max_iter);
			if (max_iter == N)
			{
				processInfo->logger.message(1)
						<< "Warning, number of line search iterations reached for primal LP strategy!"
						<< CoinMessageEol;
			}

			std::vector<double> ptNew(numVar);
			std::vector<double> ptNew2(numVar);

			std::cout << "lambda: " << r1.second << std::endl;
			for (int i = 0; i < numVar; i++)
			{
				ptNew.at(i) = r1.second * varSol.at(i) + (1 - r1.second) * primalSolution.point.at(i);
				ptNew2.at(i) = r1.first * varSol.at(i) + (1 - r1.first) * primalSolution.point.at(i);
			}

			auto error = processInfo->originalProblem->getMostDeviatingConstraint(ptNew);
			auto error2 = processInfo->originalProblem->getMostDeviatingConstraint(ptNew2);

			std::cout << "Errors: " << error.value << " and " << error2.value << std::endl;

			if (error.value <= 0)
			{
				processInfo->addPrimalSolutionCandidate(ptNew, E_PrimalSolutionSource::LPFixedIntegers,
						currIter->iterationNumber);
			}
			else
			{
				processInfo->MILPSolver->createHyperplane(error.idx, ptNew);
				std::cout << "Hyperplane added for constraint " << error.idx << " with error " << error.value
						<< std::endl;

			}

			if (error2.value <= 0 && r1.second != r1.first)
			{
				processInfo->addPrimalSolutionCandidate(ptNew2, E_PrimalSolutionSource::LPFixedIntegers,
						currIter->iterationNumber);
			}
			else
			{
				processInfo->MILPSolver->createHyperplane(error2.idx, ptNew2);
				std::cout << "Hyperplane added for constraint " << error2.idx << " with error " << error2.value
						<< std::endl;
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
