/*
 * NLPSolverCuttingPlaneRelaxed.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: alundell
 */

#include "NLPSolverCuttingPlaneRelaxed.h"
#include "../Tasks/TaskAddHyperplanes.h"

NLPSolverCuttingPlaneRelaxed::NLPSolverCuttingPlaneRelaxed()
{

	auto solver = static_cast<ES_MILPSolver>(Settings::getInstance().getIntSetting("MILPSolver", "MILP"));

	if (solver == ES_MILPSolver::Cplex)
	{
		LPSolver = new MILPSolverCplex();
		ProcessInfo::getInstance().outputInfo("Cplex selected as MILP solver for minimax solver.");
	}
	else if (solver == ES_MILPSolver::Gurobi)
	{
		LPSolver = new MILPSolverGurobi();
		ProcessInfo::getInstance().outputInfo("Gurobi selected as MILP solver for minimax solver.");
	}
	else if (solver == ES_MILPSolver::Cbc)
	{
		LPSolver = new MILPSolverOsiCbc();
		ProcessInfo::getInstance().outputInfo("Cbc selected as MILP solver for minimax solver.");
	}
	else
	{
		throw new ErrorClass("Error in MILP solver definition for minimax solver.");
	}

	//NLPProblem = new OptProblemNLPRelaxed();
	lastHyperplaneAdded = 0;
}

NLPSolverCuttingPlaneRelaxed::~NLPSolverCuttingPlaneRelaxed()
{
	delete NLPProblem, LPSolver;
}

/*void NLPSolverCuttingPlaneRelaxed::saveProblemModelToFile(std::string fileName)
 {
 NLPProblem->saveProblemModelToFile(fileName);
 }*/

E_NLPSolutionStatus NLPSolverCuttingPlaneRelaxed::solveProblemInstance()
{
	int numVar = NLPProblem->getNumberOfVariables();

	// Sets the maximal number of iterations
	int maxIter = Settings::getInstance().getIntSetting("IterLimit", "InteriorPointCuttingPlane");
	double constrSelTol = Settings::getInstance().getDoubleSetting("ConstraintSelectionTolerance",
			"InteriorPointCuttingPlane");
	double termTol = Settings::getInstance().getDoubleSetting("TermToleranceAbs", "InteriorPointCuttingPlane");
	boost::uintmax_t maxIterSubsolver = Settings::getInstance().getIntSetting("IterLimitSubsolver",
			"InteriorPointCuttingPlane");
	int bitPrecision = Settings::getInstance().getIntSetting("BitPrecision", "InteriorPointCuttingPlane");

	// currSol is the current LP solution, and prevSol the previous one
	vector<double> currSol, prevSol, LPVarSol;

	double LPObjVar;

	E_NLPSolutionStatus statusCode;

	int numHyperAdded = 0;
	int numHyperTot = 0;

	// Adds the hyperplanes created elsewhere
	for (int i = lastHyperplaneAdded; i < ProcessInfo::getInstance().addedHyperplanes.size(); i++)
	{
		if (ProcessInfo::getInstance().addedHyperplanes.at(i).source == E_HyperplaneSource::LPFixedIntegers) continue;

		LPSolver->createHyperplane(ProcessInfo::getInstance().addedHyperplanes.at(i));
	}

	for (int i = 0; i < numVar; i++)
	{
		if (NLPProblem->hasVariableBoundsBeenTightened(i))
		{
			LPSolver->updateVariableBound(i, NLPProblem->getVariableLowerBound(i),
					NLPProblem->getVariableUpperBound(i));
		}
	}

	lastHyperplaneAdded = ProcessInfo::getInstance().addedHyperplanes.size();

	for (int i = 0; i < maxIter; i++)
	{
		boost::uintmax_t maxIterSubsolverTmp = maxIterSubsolver;
		// Saves the LP problem to file if in debug mode
		if (Settings::getInstance().getBoolSetting("Debug", "SHOTSolver"))
		{
			stringstream ss;
			ss << Settings::getInstance().getStringSetting("DebugPath", "SHOTSolver");
			ss << "/nlpcuttingplanerelaxed";
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
			break;
		}
		else if (solStatus == E_ProblemSolutionStatus::IterationLimit)
		{
			statusCode = E_NLPSolutionStatus::IterationLimit;
			break;
		}

		LPVarSol = LPSolver->getVariableSolution(0);
		LPObjVar = LPSolver->getObjectiveValue();

		if (true)
		{
			std::vector<double> externalPoint = LPVarSol;
			std::vector<double> internalPoint = ProcessInfo::getInstance().interiorPts.at(0)->point;

			try
			{
				auto xNewc = ProcessInfo::getInstance().linesearchMethod->findZero(internalPoint, externalPoint,
						Settings::getInstance().getIntSetting("LinesearchMaxIter", "Linesearch"),
						Settings::getInstance().getDoubleSetting("LinesearchLambdaEps", "Linesearch"),
						Settings::getInstance().getDoubleSetting("LinesearchConstrEps", "Linesearch"));

				ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
				internalPoint = xNewc.first;
				externalPoint = xNewc.second;

				ProcessInfo::getInstance().addPrimalSolutionCandidate(internalPoint, E_PrimalSolutionSource::NLPRelaxed,
						ProcessInfo::getInstance().getCurrentIteration()->iterationNumber);

				auto errorExternal = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraints(
						externalPoint, constrSelTol);

				numHyperAdded = errorExternal.size();
				numHyperTot = numHyperTot + numHyperAdded;

				for (int j = 0; j < numHyperAdded; j++)
				{
					Hyperplane hyperplane;
					hyperplane.sourceConstraintIndex = errorExternal.at(j).idx;
					hyperplane.generatedPoint = externalPoint;
					hyperplane.source = E_HyperplaneSource::LPFixedIntegers;

					LPSolver->createHyperplane(hyperplane);

					if (Settings::getInstance().getBoolSetting("CopyCuttingPlanes", "InteriorPointCuttingPlane"))
					{
						ProcessInfo::getInstance().hyperplaneWaitingList.push_back(hyperplane);
					}
				}

				std::string hyperplanesExpr;

				hyperplanesExpr = "+" + to_string(numHyperAdded) + " = " + to_string(numHyperTot);

				std::string tmpObjLP = UtilityFunctions::toString(LPObjVar);

				boost::format tmpLine;
				std::string tmpAbsDiff = "";		//((boost::format("%.5f") % maxObjDiffAbs).str());
				std::string tmpRelDiff = ((boost::format("%.5f") % errorExternal.at(0).value).str());
				tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % (i + 1) % "LP OPT"
						% hyperplanesExpr % "" % tmpObjLP % tmpAbsDiff % tmpRelDiff;

				ProcessInfo::getInstance().outputSummary(tmpLine.str());

				currSol = externalPoint;
				prevSol = currSol;

				if (errorExternal.at(0).value <= termTol)
				{
					statusCode = E_NLPSolutionStatus::Optimal;
					break;
				}

				if (i == maxIter - 1)
				{
					statusCode = E_NLPSolutionStatus::IterationLimit;
					break;
				}
			}
			catch (std::exception &e)
			{

				ProcessInfo::getInstance().outputWarning(
						"     Cannot find solution with linesearch for fixed LP, using solution point instead:");
				ProcessInfo::getInstance().outputWarning(e.what());
			}

		}
		else
		{

			currSol = LPVarSol;

			// Gets the most deviated constraints with a tolerance
			auto tmpMostDevs = NLPProblem->getMostDeviatingConstraints(currSol, constrSelTol);

			std::string hyperplanesExpr;

			hyperplanesExpr = "+" + to_string(numHyperAdded) + " = " + to_string(numHyperTot);

			std::string tmpObjLP = UtilityFunctions::toString(LPObjVar);

			boost::format tmpLine;
			std::string tmpAbsDiff = "";		//((boost::format("%.5f") % maxObjDiffAbs).str());
			std::string tmpRelDiff = ((boost::format("%.5f") % tmpMostDevs.at(0).value).str());
			tmpLine = boost::format("%|4| %|-10s| %|=10s| %|=14s| %|=14s| %|=14s|  %|-14s|") % (i + 1) % "LP OPT"
					% hyperplanesExpr % "" % tmpObjLP % tmpAbsDiff % tmpRelDiff;

			ProcessInfo::getInstance().outputSummary(tmpLine.str());

			if (tmpMostDevs.at(0).value <= termTol)
			{
				statusCode = E_NLPSolutionStatus::Optimal;

				break;
			}

			numHyperAdded = tmpMostDevs.size();
			numHyperTot = numHyperTot + numHyperAdded;

			for (int j = 0; j < numHyperAdded; j++)
			{
				Hyperplane hyperplane;
				hyperplane.sourceConstraintIndex = tmpMostDevs.at(j).idx;
				hyperplane.generatedPoint = currSol;
				hyperplane.source = E_HyperplaneSource::LPFixedIntegers;

				LPSolver->createHyperplane(hyperplane);

				if (Settings::getInstance().getBoolSetting("CopyCuttingPlanes", "InteriorPointCuttingPlane"))
				{
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
	}

	if (currSol.size() > 0 && Settings::getInstance().getBoolSetting("Debug", "SHOTSolver"))
	{
		auto tmpVars = NLPProblem->getVariableNames();
		std::string filename = Settings::getInstance().getStringSetting("DebugPath", "SHOTSolver") + "/nlppoint"
				+ to_string(ProcessInfo::getInstance().getCurrentIteration()->iterationNumber) + ".txt";
		UtilityFunctions::saveVariablePointVectorToFile(currSol, tmpVars, filename);
	}

	solution = LPVarSol;
	objectiveValue = LPObjVar;

	return (statusCode);
}

double NLPSolverCuttingPlaneRelaxed::getSolution(int i)
{
	return (solution.at(i));
}

std::vector<double> NLPSolverCuttingPlaneRelaxed::getSolution()
{
	auto tmpSol = solution;

	if (tmpSol.size() > 0
			&& (NLPProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::GeneralNonlinear
					|| NLPProblem->getObjectiveFunctionType() == E_ObjectiveFunctionType::Quadratic))
	{
		tmpSol.pop_back();
	}

	return (tmpSol);
}

double NLPSolverCuttingPlaneRelaxed::getObjectiveValue()
{
	return (objectiveValue);
}

bool NLPSolverCuttingPlaneRelaxed::createProblemInstance(OSInstance * origInstance)
{
	ProcessInfo::getInstance().outputInfo("Creating NLP problem for relaxed cutting plane solver");

	bool useQuadraticObjective = (static_cast<ES_QPStrategy>(Settings::getInstance().getIntSetting("QPStrategy",
			"Algorithm"))) == ES_QPStrategy::QuadraticObjective;

	bool useQuadraticConstraint = (static_cast<ES_QPStrategy>(Settings::getInstance().getIntSetting("QPStrategy",
			"Algorithm"))) == ES_QPStrategy::QuadraticallyConstrained;

	bool isObjNonlinear = UtilityFunctions::isObjectiveGenerallyNonlinear(originalInstance);
	bool isObjQuadratic = UtilityFunctions::isObjectiveQuadratic(originalInstance);
	bool isQuadraticUsed = (useQuadraticObjective || (useQuadraticConstraint));

	auto solver = static_cast<ES_MILPSolver>(Settings::getInstance().getIntSetting("MILPSolver", "MILP"));

	if (solver == ES_MILPSolver::Cplex)
	{
		if (isObjNonlinear || (isObjQuadratic && !isQuadraticUsed))
		{
			NLPProblem = new OptProblemOriginalNonlinearObjective();
			dynamic_cast<OptProblemOriginalNonlinearObjective*>(NLPProblem)->setProblem(originalInstance);
		}
		else if (isObjQuadratic && isQuadraticUsed)
		{
			NLPProblem = new OptProblemOriginalQuadraticObjective();
			dynamic_cast<OptProblemOriginalQuadraticObjective*>(NLPProblem)->setProblem(originalInstance);
		}
		else
		{
			NLPProblem = new OptProblemOriginalLinearObjective();
			dynamic_cast<OptProblemOriginalLinearObjective*>(NLPProblem)->setProblem(originalInstance);
		}
	}
	else if (solver == ES_MILPSolver::Gurobi)
	{
	}
	else if (solver == ES_MILPSolver::Cbc)
	{
	}
	else
	{
		throw new ErrorClass("Error in solver definition for relaxed NLP.");
	}

	ProcessInfo::getInstance().outputInfo("NLP problem for relaxed cutting plane created");

	ProcessInfo::getInstance().outputInfo("Creating LP problem for relaxed cutting plane solver");
	LPSolver->createLinearProblem(NLPProblem);
	ProcessInfo::getInstance().outputInfo("MILP problem for relaxed cutting plane solver created");
	LPSolver->initializeSolverSettings();
	LPSolver->activateDiscreteVariables(false);

	return (true);
}

void NLPSolverCuttingPlaneRelaxed::fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues)
{
	LPSolver->fixVariables(variableIndexes, variableValues);
}

void NLPSolverCuttingPlaneRelaxed::unfixVariables()
{
	LPSolver->unfixVariables();
}

void NLPSolverCuttingPlaneRelaxed::setStartingPoint(std::vector<int> variableIndexes,
		std::vector<double> variableValues)
{
}

bool NLPSolverCuttingPlaneRelaxed::isObjectiveFunctionNonlinear()
{
	return (NLPProblem->isObjectiveFunctionNonlinear());
}

int NLPSolverCuttingPlaneRelaxed::getObjectiveFunctionVariableIndex()
{
	return (NLPProblem->getNonlinearObjectiveVariableIdx());
}

std::vector<double> NLPSolverCuttingPlaneRelaxed::getCurrentVariableLowerBounds()
{
	return (NLPProblem->getVariableLowerBounds());
}

std::vector<double> NLPSolverCuttingPlaneRelaxed::getCurrentVariableUpperBounds()
{
	return (NLPProblem->getVariableUpperBounds());
}

void NLPSolverCuttingPlaneRelaxed::clearStartingPoint()
{
}

void NLPSolverCuttingPlaneRelaxed::saveOptionsToFile(std::string fileName)
{
}
