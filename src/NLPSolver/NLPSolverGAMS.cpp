/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverGAMS.h"

namespace SHOT
{

NLPSolverGAMS::NLPSolverGAMS(EnvironmentPtr envPtr, gmoHandle_t modelingObject)
    : INLPSolver(envPtr)
    , modelingObject(modelingObject)
    , modelingEnvironment(NULL)
    , timelimit(10.0)
    , iterlimit(ITERLIM_INFINITY)
    , showlog(false)
{
    modelingEnvironment = (gevHandle_t)gmoEnvironment(modelingObject);
    strcpy(nlpsolver, "conopt");
    *nlpsolveropt = '\0';

    strcpy(nlpsolver, env->settings->getStringSetting("GAMS.NLP.Solver", "Subsolver").c_str());
    strcpy(nlpsolveropt, env->settings->getStringSetting("GAMS.NLP.OptionsFilename", "Subsolver").c_str());

    timelimit = env->settings->getDoubleSetting("FixedInteger.TimeLimit", "Primal");
    iterlimit = env->settings->getIntSetting("FixedInteger.IterationLimit", "Primal");

    // TODO: showlog seems to have no effect...
    showlog = env->settings->getBoolSetting("Console.GAMS.Show", "Output");
}

NLPSolverGAMS::~NLPSolverGAMS() {}

void NLPSolverGAMS::setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues)
{
    for(size_t i = 0; i < variableIndexes.size(); ++i)
    {
        assert(variableIndexes[i] < gmoN(modelingObject));
        gmoSetVarLOne(modelingObject, variableIndexes[i], variableValues[i]);
    }
}

void NLPSolverGAMS::clearStartingPoint() {}

void NLPSolverGAMS::fixVariables(VectorInteger variableIndexes, VectorDouble variableValues)
{
    for(size_t i = 0; i < variableIndexes.size(); ++i)
    {
        assert(variableIndexes[i] < gmoN(modelingObject));
        gmoSetAltVarLowerOne(modelingObject, variableIndexes[i], variableValues[i]);
        gmoSetAltVarUpperOne(modelingObject, variableIndexes[i], variableValues[i]);
    }
}

void NLPSolverGAMS::unfixVariables()
{
    for(size_t i = 0; i < gmoN(modelingObject); ++i)
    {
        gmoSetAltVarLowerOne(modelingObject, i, gmoGetVarLowerOne(modelingObject, i));
        gmoSetAltVarUpperOne(modelingObject, i, gmoGetVarUpperOne(modelingObject, i));
    }
}

void NLPSolverGAMS::saveOptionsToFile(std::string fileName) {}

void NLPSolverGAMS::saveProblemToFile(std::string fileName) {}

VectorDouble NLPSolverGAMS::getSolution()
{
    VectorDouble sol(gmoN(modelingObject));

    gmoGetVarL(modelingObject, &sol[0]);

    return sol;
}

double NLPSolverGAMS::getSolution(int i) { throw std::logic_error("getSolution(int) not implemented"); }

double NLPSolverGAMS::getObjectiveValue() { return gmoGetHeadnTail(modelingObject, gmoHobjval); }

E_NLPSolutionStatus NLPSolverGAMS::solveProblemInstance()
{
    char msg[GMS_SSSIZE];

    /* set which options file to use */
    if(*nlpsolveropt)
    {
        gmoOptFileSet(modelingObject, 1);
        gmoNameOptFileSet(modelingObject, nlpsolveropt);
    }
    else
    {
        /* don't read SHOT options file */
        gmoOptFileSet(modelingObject, 0);
    }

    gmoAltBoundsSet(modelingObject, 1); /* use alternative bounds */
    gmoForceContSet(modelingObject, 1);

    if(gevCallSolver(modelingEnvironment, modelingObject, "", nlpsolver, gevSolveLinkLoadLibrary,
           showlog ? gevSolverSameStreams : gevSolverQuiet, NULL, NULL, timelimit, iterlimit, 0, 0.0, 0.0, NULL, msg)
        != 0)
    {
        gmoModelStatSet(modelingObject, gmoModelStat_ErrorNoSolution);
        throw std::logic_error(std::string("Calling GAMS NLP solver failed: ") + msg);
    }

    gmoAltBoundsSet(modelingObject, 0);
    gmoForceContSet(modelingObject, 0);

    /* the callSolver calls installs a SIGINT handler again, which prevents stopping on Ctrl+C */
    gevTerminateUninstall(modelingEnvironment);

    switch(gmoModelStat(modelingObject))
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
        switch(gmoSolveStat(modelingObject))
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

VectorDouble NLPSolverGAMS::getVariableLowerBounds()
{
    assert(modelingObject != NULL);

    VectorDouble lb(gmoN(modelingObject));

    gmoGetVarLower(modelingObject, &lb[0]);

    return lb;
}

VectorDouble NLPSolverGAMS::getVariableUpperBounds()
{
    assert(modelingObject != NULL);

    VectorDouble ub(gmoN(modelingObject));

    gmoGetVarUpper(modelingObject, &ub[0]);

    return ub;
}

void NLPSolverGAMS::updateVariableLowerBound(int variableIndex, double bound)
{
    gmoSetAltVarLowerOne(modelingObject, variableIndex, bound);
}

void NLPSolverGAMS::updateVariableUpperBound(int variableIndex, double bound)
{
    gmoSetAltVarUpperOne(modelingObject, variableIndex, bound);
}

void updateVariableUpperBound(double bound);
} // namespace SHOT