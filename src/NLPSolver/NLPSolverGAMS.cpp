/**
 * @file NLPSolverGAMS.cpp
 */

#include "NLPSolverGAMS.h"
#include "../GAMS/GAMS2OS.h"
#include "../OptProblems/OptProblemNLPRelaxed.h"

NLPSolverGAMS::NLPSolverGAMS() :
		gmo(NULL), gev(NULL), timelimit(10.0), iterlimit(ITERLIM_INFINITY), showlog(false)
{
	NLPProblem = new OptProblemNLPRelaxed();

	strcpy(nlpsolver, "conopt");
	*nlpsolveropt = '\0';

	strcpy(nlpsolver, Settings::getInstance().getStringSetting("GAMS.NLP.Solver", "Subsolver").c_str());
	strcpy(nlpsolveropt, Settings::getInstance().getStringSetting("GAMS.NLP.OptionsFilename", "Subsolver").c_str());

	timelimit = Settings::getInstance().getDoubleSetting("FixedInteger.TimeLimit", "Primal");
	iterlimit = Settings::getInstance().getIntSetting("FixedInteger.IterationLimit", "Primal");

	// TODO: showlog seems to have no effect...
	showlog = Settings::getInstance().getBoolSetting("Console.ShowGAMSOutput", "Output");

}

void NLPSolverGAMS::setStartingPoint(std::vector<int> variableIndexes, std::vector<double> variableValues)
{
	for (size_t i = 0; i < variableIndexes.size(); ++i)
	{
		assert(variableIndexes[i] < gmoN(gmo));
		gmoSetVarLOne(gmo, variableIndexes[i], variableValues[i]);
	}
}

void NLPSolverGAMS::clearStartingPoint()
{
}

void NLPSolverGAMS::fixVariables(std::vector<int> variableIndexes, std::vector<double> variableValues)
{
	for (size_t i = 0; i < variableIndexes.size(); ++i)
	{
		assert(variableIndexes[i] < gmoN(gmo));
		gmoSetAltVarLowerOne(gmo, variableIndexes[i], variableValues[i]);
		gmoSetAltVarUpperOne(gmo, variableIndexes[i], variableValues[i]);
	}
}

void NLPSolverGAMS::unfixVariables()
{
	for (size_t i = 0; i < gmoN(gmo); ++i)
	{
		gmoSetAltVarLowerOne(gmo, i, gmoGetVarLowerOne(gmo, i));
		gmoSetAltVarUpperOne(gmo, i, gmoGetVarUpperOne(gmo, i));
	}
}

void NLPSolverGAMS::saveOptionsToFile(std::string fileName)
{
	//throw std::logic_error("saveOptionsToFile() not implemented");
}

std::vector<double> NLPSolverGAMS::getSolution()
{
	std::vector<double> sol(gmoN (gmo));

	gmoGetVarL(gmo, &sol[0]);

	return sol;
}

double NLPSolverGAMS::getSolution(int i)
{
	throw std::logic_error("getSolution(int) not implemented");
}

double NLPSolverGAMS::getObjectiveValue()
{
	return gmoGetHeadnTail(gmo, gmoHobjval);
}

E_NLPSolutionStatus NLPSolverGAMS::solveProblemInstance()
{
	char msg[GMS_SSSIZE];

	/* set which options file to use */
	if (*nlpsolveropt)
	{
		gmoOptFileSet(gmo, 1);
		gmoNameOptFileSet(gmo, nlpsolveropt);
	}
	else
	{
		/* don't read SHOT options file */
		gmoOptFileSet(gmo, 0);
	}

	gmoAltBoundsSet(gmo, 1); /* use alternative bounds */
	gmoForceContSet(gmo, 1);

	if (gevCallSolver(gev, gmo, "", nlpsolver, gevSolveLinkLoadLibrary, showlog ? gevSolverSameStreams : gevSolverQuiet,
			NULL, NULL, timelimit, iterlimit, 0, 0.0, 0.0, NULL, msg) != 0)
	{
		gmoModelStatSet(gmo, gmoModelStat_ErrorNoSolution);
		//throw std::logic_error(std::string("Calling GAMS NLP solver failed: ") + msg);
	}

	gmoAltBoundsSet(gmo, 0);
	gmoForceContSet(gmo, 0);

	/* the callSolver calls installs a SIGINT handler again, which prevents stopping on Ctrl+C */
	gevTerminateUninstall (gev);

	switch (gmoModelStat(gmo))
	{
		case gmoModelStat_OptimalGlobal:
		case gmoModelStat_OptimalLocal:
		case gmoModelStat_SolvedUnique:
		case gmoModelStat_Solved:
		case gmoModelStat_SolvedSingular:
		return E_NLPSolutionStatus::Optimal;
		case gmoModelStat_Unbounded:
		case gmoModelStat_UnboundedNoSolution:
		return E_NLPSolutionStatus::Unbounded;
		case gmoModelStat_InfeasibleGlobal:
		case gmoModelStat_InfeasibleLocal:
		case gmoModelStat_IntegerInfeasible:
		case gmoModelStat_InfeasibleNoSolution:
		return E_NLPSolutionStatus::Infeasible;
		case gmoModelStat_Feasible:
		case gmoModelStat_Integer:
		return E_NLPSolutionStatus::Feasible;

		case gmoModelStat_InfeasibleIntermed:
		case gmoModelStat_NonIntegerIntermed:
		case gmoModelStat_NoSolutionReturned:
		switch (gmoSolveStat(gmo))
		{
			case gmoSolveStat_Iteration:
			return E_NLPSolutionStatus::IterationLimit;
			case gmoSolveStat_Resource:
			return E_NLPSolutionStatus::TimeLimit;
			case gmoSolveStat_Normal:
			case gmoSolveStat_User:
			return E_NLPSolutionStatus::Infeasible;
			default:
			return E_NLPSolutionStatus::Error;
		}

		case gmoModelStat_LicenseError:
		case gmoModelStat_ErrorUnknown:
		case gmoModelStat_ErrorNoSolution:
		return E_NLPSolutionStatus::Error;
	}
}

bool NLPSolverGAMS::createProblemInstance(OSInstance * origInstance)
{
	dynamic_cast<OptProblemNLPRelaxed*>(NLPProblem)->reformulate(origInstance);

	GAMSOSInstance* gamsosinstance = static_cast<GAMSOSInstance*>(origInstance);
	// TODO cannot do dynamic_cast to check type
//	if( gamsosinstance == NULL )
//		return false;

	gmo = gamsosinstance->gmo;
	gev = (gevHandle_t) gmoEnvironment(gmo);

	return true;
}

bool NLPSolverGAMS::isObjectiveFunctionNonlinear()
{
	return (NLPProblem->isObjectiveFunctionNonlinear());
}

int NLPSolverGAMS::getObjectiveFunctionVariableIndex()
{
	return (NLPProblem->getNonlinearObjectiveVariableIdx());
}

#if 0
std::vector<double> NLPSolverGAMS::getCurrentVariableLowerBounds()
{
	return (NLPProblem->getVariableLowerBounds());
}

std::vector<double> NLPSolverGAMS::getCurrentVariableUpperBounds()
{
	return (NLPProblem->getVariableUpperBounds());
}
#endif

std::vector<double> NLPSolverGAMS::getCurrentVariableLowerBounds()
{
	assert(gmo != NULL);

	std::vector<double> lb(gmoN (gmo));

	gmoGetVarLower(gmo, &lb[0]);

	return lb;
}

std::vector<double> NLPSolverGAMS::getCurrentVariableUpperBounds()
{
	assert(gmo != NULL);

	std::vector<double> ub(gmoN (gmo));

	gmoGetVarUpper(gmo, &ub[0]);

	return ub;
}

