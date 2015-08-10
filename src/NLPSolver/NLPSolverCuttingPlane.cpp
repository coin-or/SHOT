/*
 * NLPSolverCuttingPlane.cpp
 *
 *  Created on: Apr 16, 2015
 *      Author: alundell
 */

#include <NLPSolverCuttingPlane.h>

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

			auto validNewPt = NLPProblem->getMostDeviatingConstraint(ptNew).value;

			return (validNewPt);
		}
};

NLPSolverCuttingPlane::NLPSolverCuttingPlane()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	processInfo->startTimer("InteriorPointMinimax");
	auto solver = static_cast<ES_MILPSolver>(settings->getIntSetting("MILPSolver", "MILP"));

	if (solver == ES_MILPSolver::Cplex)
	{
		MILPSolver = new MILPSolverCplex();
		processInfo->logger.message(2) << "Cplex selected as MILP solver for minimax solver." << CoinMessageEol;
	}
	else if (solver == ES_MILPSolver::Gurobi)
	{
		MILPSolver = new MILPSolverGurobi();
		processInfo->logger.message(2) << "Gurobi selected as MILP solver for minimax solver." << CoinMessageEol;
	}
	else if (solver == ES_MILPSolver::Cbc)
	{
		MILPSolver = new MILPSolverOsiCbc();
		processInfo->logger.message(2) << "Cbc selected as MILP solver for minimax solver." << CoinMessageEol;
	}
	else
	{
		processInfo->stopTimer("InteriorPointMinimax");
		throw new ErrorClass("Error in MILP solver definition for minimax solver.");
	}

	//NLPProblem = new OptProblemNLPSHOTMinimax();
	NLPProblem = new OptProblemNLPMinimax();

	processInfo->stopTimer("InteriorPointMinimax");
}

NLPSolverCuttingPlane::~NLPSolverCuttingPlane()
{
	delete NLPProblem, MILPSolver;
}

bool NLPSolverCuttingPlane::createProblem(OSInstance* origInstance)
{

	processInfo->startTimer("InteriorPointMinimax");

	processInfo->logger.message(3) << "Creating NLP problem for minimax solver" << CoinMessageEol;
	NLPProblem->reformulate(origInstance);
	processInfo->logger.message(3) << "NLP problem for minimax solver created" << CoinMessageEol;

	processInfo->logger.message(3) << "Creating LP problem for minimax solver" << CoinMessageEol;
	MILPSolver->createLinearProblem(NLPProblem);
	processInfo->logger.message(3) << "MILP problem for minimax solver created" << CoinMessageEol;

	processInfo->stopTimer("InteriorPointMinimax");
	return (true);
}

bool NLPSolverCuttingPlane::solveProblem()
{

	processInfo->startTimer("InteriorPointMinimax");
	int numVar = NLPProblem->getNumberOfVariables();

	// Sets the maximal number of iterations
	int maxIter = settings->getIntSetting("IterLimit", "MinimaxNLP");
	double termObjTolAbs = settings->getDoubleSetting("TermToleranceAbs", "MinimaxNLP");
	double termObjTolRel = settings->getDoubleSetting("TermToleranceRel", "MinimaxNLP");
	double constrSelTol = settings->getDoubleSetting("ConstraintSelectionTolerance", "MinimaxNLP");
	boost::uintmax_t maxIterSubsolver = settings->getIntSetting("IterLimitSubsolver", "MinimaxNLP");
	int bitPrecision = settings->getIntSetting("BitPrecision", "MinimaxNLP");

	// currSol is the current LP solution, and prevSol the previous one
	vector<double> currSol, prevSol;

	double lambda; // Variable in the linesearch minimization
	double mu; // Objective value for the linesearch minimization value

	// Corresponds to the difference between the LP solution objective value and
	// the objective found in the linesearch minimization procedure
	double maxObjDiffAbs = DBL_MAX;
	double maxObjDiffRel = DBL_MAX;

	int numHPsAdded, numHPsTotal;
	for (int i = 0; i < maxIter; i++)
	{
		boost::uintmax_t maxIterSubsolverTmp = maxIterSubsolver;
		// Saves the LP problem to file if in debug mode
		if (settings->getBoolSetting("Debug", "SHOTSolver"))
		{
			stringstream ss;
			ss << settings->getStringSetting("DebugPath", "SHOTSolver");
			ss << "/lpminimax";
			ss << i;
			ss << ".lp";
			MILPSolver->writeProblemToFile(ss.str());
		}

		// Solves the problem and obtains the solution
		MILPSolver->solveProblem();
		auto LPVarSol = MILPSolver->getVariableSolution(0);
		auto LPObjVar = MILPSolver->getObjectiveValue();

		if (i == 0) // No linesearch minimization in first iteration, just add cutting plane in LP solution point
		{
			currSol = LPVarSol;
			lambda = -1; // For reporting purposes only
			mu = LPObjVar;
			numHPsAdded = 0;
			numHPsTotal = 0;

			auto tmpLine = boost::format("%1% %|4t|%2% %3% %|15t|%4% %|30t|%5% %|45t|%6% %|60t|%7% %|75t|%8%") % "#"
					% "HPs" % " " % "Obj. LP" % "Obj. LS" % "Abs. diff." % "Rel. diff." % "Lambda";

			processInfo->logger.message(2)
					<< "=================================================================================="
					<< CoinMessageEol << tmpLine.str() << CoinMessageEol
					<< "=================================================================================="
					<< CoinMessageEol;
		}
		else
		{
			MinimizationFunction funct(LPVarSol, prevSol, NLPProblem);

			// Solves the minization problem wrt lambda in [0, 1]

			auto minimizationResult = boost::math::tools::brent_find_minima(funct, 0.0, 1.0, bitPrecision,
					maxIterSubsolverTmp);

			//std::cout << "maxIter: " << maxIterSubsolverTmp << std::endl;
			lambda = minimizationResult.first;
			mu = minimizationResult.second;

			//std::cout << "mu: " << mu << " lambda: " << lambda << std::endl;

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

		auto tmpLine = boost::format("%1% %|4t|+%2% = %3% %|15t|%4% %|30t|%5% %|45t|%6% %|60t|%7% %|75t|%8%") % i
				% numHPsAdded % numHPsTotal % LPObjVar % mu % maxObjDiffAbs % maxObjDiffRel % lambda;

		processInfo->logger.message(2) << tmpLine.str() << CoinMessageEol;

		// Checks termination condition
		if (mu <= 0 && (maxObjDiffAbs < termObjTolAbs || maxObjDiffRel < termObjTolRel))
		{
			break;
		}

		numHPsAdded = tmpMostDevs.size();
		numHPsTotal = numHPsTotal + numHPsAdded;

		for (int j = 0; j < numHPsAdded; j++)
		{
			std::vector < IndexValuePair > elements; // Contains the terms in the hyperplane

			double constant = NLPProblem->calculateConstraintFunctionValue(tmpMostDevs.at(j).idx, currSol);

			//constant += LPObjVar;
			processInfo->logger.message(3) << " HP point generated for constraint index" << tmpMostDevs.at(j).idx
					<< CoinMessageEol;

			processInfo->logger.message(3) << "  Constant " << constant << CoinMessageEol;

			// Calculates the gradient
			auto nablag = NLPProblem->calculateConstraintFunctionGradient(tmpMostDevs.at(j).idx, currSol);

			for (int i = 0; i < nablag->number; i++)
			{
				if (nablag->indexes[i] != numVar - 1)
				{
					IndexValuePair pair;
					pair.idx = nablag->indexes[i];
					pair.value = nablag->values[i];

					elements.push_back(pair);

					constant += -nablag->values[i] * currSol.at(nablag->indexes[i]);
				}
			}

			// Creates the term -mu
			IndexValuePair pair;
			pair.idx = numVar - 1;
			pair.value = -1;

			elements.push_back(pair);

			for (auto E : elements)
			{
				processInfo->logger.message(3) << "  Coefficient for variable index " << E.idx << ": " << E.value
						<< CoinMessageEol;
			}

			processInfo->logger.message(3) << "  Constant " << constant << CoinMessageEol;

			// Adds the linear constraint
			MILPSolver->addLinearConstraint(elements, constant);
		}

		prevSol = currSol;
	}

// Removes the mu variable value from the point
	currSol.pop_back();

	auto tmpIP = new InteriorPoint();
	tmpIP->NLPSolver = static_cast<int>(ES_NLPSolver::CuttingPlaneMiniMax);
	tmpIP->point = currSol;

	processInfo->interiorPts.push_back(*tmpIP);

	delete MILPSolver;

	processInfo->stopTimer("InteriorPointMinimax");

	return (true);
}

void NLPSolverCuttingPlane::saveProblemModelToFile(std::string fileName)
{
	NLPProblem->saveProblemModelToFile(fileName);
}
