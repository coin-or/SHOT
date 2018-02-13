/*
 * NLPSolverCuttingPlaneMinimax.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: alundell
 */

#include "NLPSolverCuttingPlaneMinimax.h"
#include "../Tasks/TaskAddHyperplanes.h"

class MinimizationFunction
{
  private:
	std::vector<double> firstPt;
	std::vector<double> secondPt;
	OptProblem *NLPProblem;

  public:
	MinimizationFunction(std::vector<double> ptA, std::vector<double> ptB, OptProblem *prob)
	{
		firstPt = ptA;
		secondPt = ptB;
		NLPProblem = prob;
	}

	double operator()(const double x)
	{
		int length = secondPt.size();
		std::vector<double> ptNew(length);

		for (int i = 0; i < length; i++)
		{
			ptNew.at(i) = x * firstPt.at(i) + (1 - x) * secondPt.at(i);
		}

		auto validNewPt = NLPProblem->getMostDeviatingConstraint(ptNew).value + ptNew.back();

		return (validNewPt);
	}
};

NLPSolverCuttingPlaneMinimax::NLPSolverCuttingPlaneMinimax()
{

	auto solver = static_cast<ES_MIPSolver>(Settings::getInstance().getIntSetting("MIP.Solver", "Dual"));

	if (solver == ES_MIPSolver::Cplex)
	{
		LPSolver = new MIPSolverCplex();
		ProcessInfo::getInstance().outputInfo("Cplex selected as MIP solver for minimax solver.");
	}
	else if (solver == ES_MIPSolver::Gurobi)
	{
		LPSolver = new MIPSolverGurobi();
		ProcessInfo::getInstance().outputInfo("Gurobi selected as MIP solver for minimax solver.");
	}
	else if (solver == ES_MIPSolver::Cbc)
	{
		LPSolver = new MIPSolverOsiCbc();
		ProcessInfo::getInstance().outputInfo("Cbc selected as MIP solver for minimax solver.");
	}
	else
	{
		throw new ErrorClass("Error in MIP solver definition for minimax solver.");
	}

	NLPProblem = new OptProblemNLPMinimax();
}

NLPSolverCuttingPlaneMinimax::~NLPSolverCuttingPlaneMinimax()
{
	delete NLPProblem;
	delete LPSolver;
}

/*void NLPSolverCuttingPlaneMinimax::saveProblemModelToFile(std::string fileName)
 {
 NLPProblem->saveProblemModelToFile(fileName);
 }*/

E_NLPSolutionStatus NLPSolverCuttingPlaneMinimax::solveProblemInstance()
{
	int numVar = NLPProblem->getNumberOfVariables();

	// Sets the maximal number of iterations
	int maxIter = Settings::getInstance().getIntSetting("ESH.InteriorPoint.CuttingPlane.IterationLimit", "Dual");
	double termObjTolAbs = Settings::getInstance().getDoubleSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceAbs", "Dual");
	double termObjTolRel = Settings::getInstance().getDoubleSetting("ESH.InteriorPoint.CuttingPlane.TerminationToleranceRel", "Dual");
	double constrSelTol = Settings::getInstance().getDoubleSetting("ESH.InteriorPoint.CuttingPlane.ConstraintSelectionTolerance", "Dual");
	boost::uintmax_t maxIterSubsolver = Settings::getInstance().getIntSetting("ESH.InteriorPoint.CuttingPlane.IterationLimitSubsolver", "Dual");
	int bitPrecision = Settings::getInstance().getIntSetting("ESH.InteriorPoint.CuttingPlane.BitPrecision", "Dual");

	// currSol is the current LP solution, and prevSol the previous one
	vector<double> currSol, prevSol;

	double lambda; // Variable in the linesearch minimization
	double mu;	 // Objective value for the linesearch minimization value

	// Corresponds to the difference between the LP solution objective value and
	// the objective found in the linesearch minimization procedure
	double maxObjDiffAbs = OSDBL_MAX;
	double maxObjDiffRel = OSDBL_MAX;

	double LPObjVar;

	E_NLPSolutionStatus statusCode;

	int numHyperAdded, numHyperTot;
	for (int i = 0; i <= maxIter; i++)
	{
		boost::uintmax_t maxIterSubsolverTmp = maxIterSubsolver;
		// Saves the LP problem to file if in debug mode
		if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
		{
			stringstream ss;
			ss << Settings::getInstance().getStringSetting("Debug.Path", "Output");
			ss << "/lpminimax";
			ss << i;
			ss << ".lp";
			LPSolver->writeProblemToFile(ss.str());
		}

		// Solves the problem and obtains the solution
		auto solStatus = LPSolver->solveProblem();

		if (solStatus == E_ProblemSolutionStatus::Infeasible)
		{
			statusCode = E_NLPSolutionStatus::Infeasible;
			break;
		}
		else if (solStatus == E_ProblemSolutionStatus::Error)
		{
			statusCode = E_NLPSolutionStatus::Error;
			break;
		}
		else if (solStatus == E_ProblemSolutionStatus::Unbounded)
		{
			statusCode = E_NLPSolutionStatus::Unbounded;
			break;
		}
		else if (solStatus == E_ProblemSolutionStatus::TimeLimit)
		{
			statusCode = E_NLPSolutionStatus::TimeLimit;
		}
		else if (solStatus == E_ProblemSolutionStatus::IterationLimit)
		{
			statusCode = E_NLPSolutionStatus::IterationLimit;
		}
		else
		{
		}

		auto LPVarSol = LPSolver->getVariableSolution(0);
		LPObjVar = LPSolver->getObjectiveValue();

		if (i == 0) // No linesearch minimization in first iteration, just add cutting plane in LP solution point
		{
			currSol = LPVarSol;
			lambda = -1; // For reporting purposes only
			mu = LPObjVar;
			numHyperAdded = 0;
			numHyperTot = 0;

			std::stringstream tmpLine;

#ifdef _WIN32
			tmpLine << "\nÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ\n";

			tmpLine
				<< boost::format("%|=14| %|=11| %|=14| %|=14| %|=14|  %s\n") % " Iteration" % "HPs" % "Obj. LP" % "Obj. LS" % "Abs. diff" % "Rel. diff";

			tmpLine << "ÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ\n";
#else
			tmpLine << "\n═════════════════════════════════════════════════════════════════════════════════════\n";

			tmpLine
				<< boost::format("%|=14| %|=11| %|=14| %|=14| %|=14|  %s\n") % " Iteration" % "HPs" % "Obj. LP" % "Obj. LS" % "Abs. diff" % "Rel. diff";

			tmpLine << "═════════════════════════════════════════════════════════════════════════════════════\n";
#endif

			ProcessInfo::getInstance().outputSummary(tmpLine.str());
		}
		else
		{
			MinimizationFunction funct(LPVarSol, prevSol, NLPProblem);

			// Solves the minization problem wrt lambda in [0, 1]

			auto minimizationResult = boost::math::tools::brent_find_minima(funct, 0.0, 1.0, bitPrecision,
																			maxIterSubsolverTmp);

			lambda = minimizationResult.first;
			mu = minimizationResult.second;

			// Calculates the corresponding solution point
			for (int i = 0; i < numVar; i++)
			{
				currSol.at(i) = lambda * LPVarSol.at(i) + (1 - lambda) * prevSol.at(i);
			}

			// The difference between linesearch and LP objective values
			maxObjDiffAbs = abs(mu - LPObjVar);
			maxObjDiffRel = maxObjDiffAbs / ((1e-10) + abs(LPObjVar));
		}

		// Gets the most deviated constraints with a tolerance
		auto tmpMostDevs = NLPProblem->getMostDeviatingConstraints(currSol, constrSelTol);

		std::string hyperplanesExpr;

		hyperplanesExpr = "+" + to_string(numHyperAdded) + " = " + to_string(numHyperTot);

		std::string tmpObjLP = UtilityFunctions::toString(LPObjVar);
		std::string tmpObjLS = UtilityFunctions::toString(mu);

		boost::format tmpLine;

		if (i == 0) // No linesearch minimization in first iteration, just add cutting plane in LP solution point
		{
			tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % (i + 1) % "LP OPT" % hyperplanesExpr % tmpObjLP % "" % "" % "";
		}
		else
		{
			std::string tmpAbsDiff = ((boost::format("%.5f") % maxObjDiffAbs).str());
			std::string tmpRelDiff = ((boost::format("%.5f") % maxObjDiffRel).str());
			tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % (i + 1) % "LP OPT" % hyperplanesExpr % tmpObjLP % tmpObjLS % tmpAbsDiff % tmpRelDiff;
		}

		ProcessInfo::getInstance().outputSummary(tmpLine.str());

		if (mu <= 0 && (maxObjDiffAbs < termObjTolAbs || maxObjDiffRel < termObjTolRel))
		{
			statusCode = E_NLPSolutionStatus::Optimal;

			break;
		}

		numHyperAdded = tmpMostDevs.size();
		numHyperTot = numHyperTot + numHyperAdded;

		for (int j = 0; j < numHyperAdded; j++)
		{
			std::vector<IndexValuePair> elements; // Contains the terms in the hyperplane

			double constant = NLPProblem->calculateConstraintFunctionValue(tmpMostDevs.at(j).idx, currSol);

			// Calculates the gradient
			auto nablag = NLPProblem->calculateConstraintFunctionGradient(tmpMostDevs.at(j).idx, currSol);
			ProcessInfo::getInstance().numGradientEvals++;

			for (int i = 0; i < nablag->number; i++)
			{
				//if (nablag->indexes[i] != numVar - 1)
				//{
				IndexValuePair pair;
				pair.idx = nablag->indexes[i];
				pair.value = nablag->values[i];

				elements.push_back(pair);

				constant += -nablag->values[i] * currSol.at(nablag->indexes[i]);
				//}
			}

			// Creates the term -mu
			//IndexValuePair pair;
			//pair.idx = numVar - 1;
			//pair.value = -1;

			//elements.push_back(pair);

			// Adds the linear constraint
			LPSolver->addLinearConstraint(elements, constant);

			if (mu >= 0 && Settings::getInstance().getBoolSetting("ESH.InteriorPoint.CuttingPlane.Reuse", "Dual") && tmpMostDevs.at(j).idx != NLPProblem->getNonlinearObjectiveConstraintIdx())
			{
				auto tmpPoint = currSol;

				while (tmpPoint.size() > ProcessInfo::getInstance().originalProblem->getNumberOfVariables())
				{
					tmpPoint.pop_back();
				}

				Hyperplane hyperplane;
				hyperplane.sourceConstraintIndex = j;
				hyperplane.generatedPoint = tmpPoint;
				hyperplane.source = E_HyperplaneSource::InteriorPointSearch;

				ProcessInfo::getInstance().hyperplaneWaitingList.push_back(hyperplane);
			}
		}

		prevSol = currSol;

		if (i == maxIter - 1)
		{
			statusCode = E_NLPSolutionStatus::IterationLimit;
			break;
		}
	}

	currSol.pop_back();

	solution = currSol;
	objectiveValue = LPObjVar;

	return (statusCode);
}

double NLPSolverCuttingPlaneMinimax::getSolution(int i)
{
	return (solution.at(i));
}

std::vector<double> NLPSolverCuttingPlaneMinimax::getSolution()
{
	auto tmpSol = solution;

	if (ProcessInfo::getInstance().originalProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic)
	{
		tmpSol.pop_back();
	}

	return (tmpSol);
}

double NLPSolverCuttingPlaneMinimax::getObjectiveValue()
{
	return (objectiveValue);
}

bool NLPSolverCuttingPlaneMinimax::createProblemInstance(OSInstance *origInstance)
{
	ProcessInfo::getInstance().outputInfo("Creating NLP problem for minimax solver");
	dynamic_cast<OptProblemNLPMinimax *>(NLPProblem)->reformulate(origInstance);
	ProcessInfo::getInstance().outputInfo("NLP problem for minimax solver created");

	ProcessInfo::getInstance().outputInfo("Creating LP problem for minimax solver");
	LPSolver->createLinearProblem(NLPProblem);
	LPSolver->initializeSolverSettings();
	LPSolver->activateDiscreteVariables(false);
	ProcessInfo::getInstance().outputInfo("LP problem for minimax solver created");

	return (true);
}

void NLPSolverCuttingPlaneMinimax::fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues)
{
	LPSolver->fixVariables(variableIndexes, variableValues);
}

void NLPSolverCuttingPlaneMinimax::unfixVariables()
{
	LPSolver->unfixVariables();
}

void NLPSolverCuttingPlaneMinimax::setStartingPoint(std::vector<int> variableIndexes,
													std::vector<double> variableValues)
{
}

bool NLPSolverCuttingPlaneMinimax::isObjectiveFunctionNonlinear()
{
	return (false);
}

int NLPSolverCuttingPlaneMinimax::getObjectiveFunctionVariableIndex()
{
	return (COIN_INT_MAX);
}

std::vector<double> NLPSolverCuttingPlaneMinimax::getCurrentVariableLowerBounds()
{
	return (NLPProblem->getVariableLowerBounds());
}

std::vector<double> NLPSolverCuttingPlaneMinimax::getCurrentVariableUpperBounds()
{
	return (NLPProblem->getVariableUpperBounds());
}

void NLPSolverCuttingPlaneMinimax::clearStartingPoint()
{
}

void NLPSolverCuttingPlaneMinimax::saveOptionsToFile(std::string fileName)
{
}
