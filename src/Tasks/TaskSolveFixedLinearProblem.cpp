/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSolveFixedLinearProblem.h"

TaskSolveFixedLinearProblem::TaskSolveFixedLinearProblem(IMIPSolver *MIPSolver)
{
	this->MIPSolver = MIPSolver;

	ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
	ProcessInfo::getInstance().startTimer("PrimalBoundFixedLP");

	discreteVariableIndexes = ProcessInfo::getInstance().originalProblem->getDiscreteVariableIndices();

	ProcessInfo::getInstance().stopTimer("PrimalBoundFixedLP");
	ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
}

TaskSolveFixedLinearProblem::~TaskSolveFixedLinearProblem()
{
}

void TaskSolveFixedLinearProblem::run()
{
	ProcessInfo::getInstance().startTimer("PrimalBoundTotal");
	ProcessInfo::getInstance().startTimer("PrimalBoundFixedLP");
	auto currIter = ProcessInfo::getInstance().getCurrentIteration(); //The one not solved yet

	if (currIter->MIPSolutionLimitUpdated)
		return;

	if (currIter->iterationNumber < 5)
	{
		ProcessInfo::getInstance().stopTimer("PrimalBoundFixedLP");
		ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
		return;
	}

	if (currIter->maxDeviation <= Settings::getInstance().getDoubleSetting("FixedInteger.ConstraintTolerance", "Dual"))
	{
		ProcessInfo::getInstance().stopTimer("PrimalBoundFixedLP");
		ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
		return;
	}

	auto prevIter = ProcessInfo::getInstance().getPreviousIteration();

	if (prevIter->iterationNumber < 4)
	{

		ProcessInfo::getInstance().stopTimer("PrimalBoundFixedLP");
		ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
		return;
	}

	auto prevIter2 = &ProcessInfo::getInstance().iterations.at(prevIter->iterationNumber - 2);
	auto prevIter3 = &ProcessInfo::getInstance().iterations.at(prevIter->iterationNumber - 3);

	if (!prevIter->isMIP() && !prevIter2->isMIP() && !prevIter3->isMIP())
	{
		ProcessInfo::getInstance().stopTimer("PrimalBoundFixedLP");
		ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
		return;
	}

	if (currIter->numHyperplanesAdded == 0)
	{
		ProcessInfo::getInstance().stopTimer("PrimalBoundFixedLP");
		ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
		return;
	}

	auto discreteIdxs = ProcessInfo::getInstance().originalProblem->getDiscreteVariableIndices();

	auto currSolPt = prevIter->solutionPoints.at(0).point;

	bool isDifferent1 = UtilityFunctions::isDifferentSelectedElements(currSolPt, prevIter2->solutionPoints.at(0).point,
																	  discreteIdxs);

	bool isDifferent2 = UtilityFunctions::isDifferentSelectedElements(currSolPt, prevIter3->solutionPoints.at(0).point,
																	  discreteIdxs);

	if (isDifferent1 || isDifferent2)
	{
		ProcessInfo::getInstance().stopTimer("PrimalBoundFixedLP");
		ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
		return;
	}

	std::vector<double> fixValues(discreteVariableIndexes.size());

	for (int i = 0; i < discreteVariableIndexes.size(); i++)
	{
		fixValues.at(i) = currSolPt.at(discreteVariableIndexes.at(i));
	}

	MIPSolver->fixVariables(discreteVariableIndexes, fixValues);

	int numVar = ProcessInfo::getInstance().originalProblem->getNumberOfVariables();

	bool isMinimization = ProcessInfo::getInstance().originalProblem->isTypeOfObjectiveMinimize();

	double prevObjVal = COIN_DBL_MAX;

	int iterLastObjUpdate = 0;
	int maxIter = Settings::getInstance().getIntSetting("FixedInteger.MaxIterations", "Dual");
	double objTol = Settings::getInstance().getDoubleSetting("FixedInteger.ObjectiveTolerance", "Dual");
	double constrTol = Settings::getInstance().getDoubleSetting("FixedInteger.ConstraintTolerance", "Dual");

	for (int k = 0; k < maxIter; k++)
	{
		auto solStatus = MIPSolver->solveProblem();

		if (solStatus != E_ProblemSolutionStatus::Optimal)
		{
			auto tmpLine = boost::format("%|4s| %|-10s| %|=10s| %|=44s|  %|-14s|") % k % "FIXLP INF" % "" % "" % "";
			ProcessInfo::getInstance().outputSummary(tmpLine.str());
			break;
		}
		else
		{
			auto varSol = MIPSolver->getVariableSolution(0);
			auto objVal = MIPSolver->getObjectiveValue(0);

			auto mostDevConstr = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(varSol);

			std::vector<double> externalPoint = varSol;
			IndexValuePair errorExternal;

			if (ProcessInfo::getInstance().interiorPts.size() > 0)
			{
				std::vector<double> internalPoint = ProcessInfo::getInstance().interiorPts.at(0)->point;

				auto tmpMostDevConstr2 = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(internalPoint);

				try
				{
					auto xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(internalPoint, externalPoint,
																					   Settings::getInstance().getIntSetting("Rootsearch.MaxIterations", "Subsolver"),
																					   Settings::getInstance().getDoubleSetting("Rootsearch.TerminationTolerance", "Subsolver"),
																					   Settings::getInstance().getDoubleSetting("Rootsearch.ActiveConstraintTolerance", "Subsolver"));

					ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
					internalPoint = xNewc.first;
					externalPoint = xNewc.second;
				}
				catch (std::exception &e)
				{

					ProcessInfo::getInstance().outputWarning(
						"     Cannot find solution with linesearch for fixed LP, using solution point instead:");
					ProcessInfo::getInstance().outputWarning(e.what());
				}
			}

			errorExternal = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(externalPoint);

			Hyperplane hyperplane;
			hyperplane.sourceConstraintIndex = errorExternal.idx;
			hyperplane.generatedPoint = externalPoint;
			hyperplane.source = E_HyperplaneSource::LPFixedIntegers;

			MIPSolver->createHyperplane(hyperplane);

			std::stringstream tmpType;

			bool hasSolution = true;

			bool isMIQP = (ProcessInfo::getInstance().originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic);
			bool isMIQCP = (ProcessInfo::getInstance().originalProblem->getQuadraticConstraintIndexes().size() > 0);
			bool isDiscrete = false;

			if (isMIQCP)
				tmpType << "FIXQCP";
			else if (isMIQP)
				tmpType << "FIXQP";
			else
				tmpType << "FIXLP";

			if (varSol.size() == 0)
				hasSolution = false;

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
				tmpConstr = ProcessInfo::getInstance().originalProblem->getConstraintNames()[mostDevConstr.idx] + ": " + ((boost::format("%.5f") % mostDevConstr.value).str());
			}
			else if (hasSolution)
			{
				tmpConstr = ProcessInfo::getInstance().originalProblem->getConstraintNames().back() + ": " + ((boost::format("%.5f") % mostDevConstr.value).str());
			}
			else
			{
				tmpConstr = "";
			}

			std::string primalBoundExpr;
			std::string dualBoundExpr = "";

			auto primalBound = ProcessInfo::getInstance().getPrimalBound();
			auto dualBound = ProcessInfo::getInstance().getDualBound();

			if (primalBound != lastPrimalBound && ProcessInfo::getInstance().primalSolutions.size() > 0)
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
				auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % k % "FIXLP CON" % hyperplanesExpr % "" % tmpObjVal % primalBoundExpr % tmpConstr;
				ProcessInfo::getInstance().outputSummary(tmpLine.str());
				break;
			}

			if (k - iterLastObjUpdate > 10)
			{
				auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % k % "FIXLP ITR" % hyperplanesExpr % "" % tmpObjVal % primalBoundExpr % tmpConstr;
				ProcessInfo::getInstance().outputSummary(tmpLine.str());
				break;
			}

			if (objVal > ProcessInfo::getInstance().getPrimalBound())
			{
				auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % k % "FIXLP PB " % hyperplanesExpr % "" % tmpObjVal % primalBoundExpr % tmpConstr;
				ProcessInfo::getInstance().outputSummary(tmpLine.str());

				break;
			}

			auto tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % k % tmpType.str() % hyperplanesExpr % "" % tmpObjVal % primalBoundExpr % tmpConstr;
			ProcessInfo::getInstance().outputSummary(tmpLine.str());

			if (abs(prevObjVal - objVal) > prevObjVal * objTol)
			{
				iterLastObjUpdate = k;
				prevObjVal = objVal;
			}
		}
	}

	MIPSolver->activateDiscreteVariables(true);

	MIPSolver->unfixVariables();

	ProcessInfo::getInstance().stopTimer("PrimalBoundFixedLP");
	ProcessInfo::getInstance().stopTimer("PrimalBoundTotal");
	return;
}

std::string TaskSolveFixedLinearProblem::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
