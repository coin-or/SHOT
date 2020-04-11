/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverGAMS.h"

#include "../Output.h"
#include "../Settings.h"

#include <cstdio>
#include <cstring>

namespace SHOT
{

NLPSolverGAMS::NLPSolverGAMS(EnvironmentPtr envPtr, gmoHandle_t modelingObject, palHandle_t auditLicensing)
    : INLPSolver(envPtr)
    , modelingObject(modelingObject)
    , modelingEnvironment(nullptr)
    , timelimit(10.0)
    , iterlimit(ITERLIM_INFINITY)
    , showlog(false)
{
    modelingEnvironment = (gevHandle_t)gmoEnvironment(modelingObject);

    nlpsolver = env->settings->getSetting<std::string>("GAMS.NLP.Solver", "Subsolver");
    nlpsolveropt = env->settings->getSetting<std::string>("GAMS.NLP.OptionsFilename", "Subsolver");

    timelimit = env->settings->getSetting<double>("FixedInteger.TimeLimit", "Primal");
    iterlimit = env->settings->getSetting<int>("FixedInteger.IterationLimit", "Primal");

    if(nlpsolver == "auto")
    {
        assert(auditLicensing != nullptr);
        if(!palLicenseCheckSubSys(auditLicensing, (char*)"CO"))
        {
            env->output->outputDebug("CONOPT licensed. Using CONOPT as GAMS NLP solver.");
            nlpsolver = "conopt";
            selectedNLPSolver = "CONOPT (automatically selected)";
        }
        else if(!palLicenseCheckSubSys(auditLicensing, (char*)"KN"))
        {
            env->output->outputDebug("CONOPT not licensed. KNITRO licensed. Using KNITRO as GAMS NLP solver.");
            nlpsolver = "knitro";
            selectedNLPSolver = "KNITRO (automatically selected)";
        }
        else if(!palLicenseCheckSubSys(auditLicensing, (char*)"SN"))
        {
            env->output->outputDebug("CONOPT and KNITRO not licensed. SNOPT licensed. Using SNOPT as GAMS NLP solver.");
            nlpsolver = "snopt";
            selectedNLPSolver = "SNOPT (automatically selected)";
        }
        else if(!palLicenseCheckSubSys(auditLicensing, (char*)"M5"))
        {
            env->output->outputDebug(
                "CONOPT, KNITRO, and SNOPT not licensed. MINOS licensed. Using MINOS as GAMS NLP solver.");
            nlpsolver = "minos";
            selectedNLPSolver = "MINOS (automatically selected)";
        }
        else if(!palLicenseCheckSubSys(auditLicensing, (char*)"IP"))
        {
            env->output->outputDebug(
                "CONOPT, KNITRO, SNOPT, and MINOS not licensed. IPOPTH licensed. Using IPOPTH as GAMS NLP solver.");
            nlpsolver = "ipopth";
            selectedNLPSolver = "IPOPTH (automatically selected)";
        }
        else
        {
            env->output->outputDebug(
                "CONOPT, KNITRO, SNOPT, MINOS, and IPOPTH not licensed. Using IPOPT as GAMS NLP solver.");
            nlpsolver = "ipopt";
            selectedNLPSolver = "IPOPT (automatically selected)";
        }
    }
    else
    {
        selectedNLPSolver = nlpsolver;
        std::transform(selectedNLPSolver.begin(), selectedNLPSolver.end(), selectedNLPSolver.begin(), ::toupper);
    }

    // TODO: showlog seems to have no effect...
    showlog = env->settings->getSetting<bool>("Console.PrimalSolver.Show", "Output");
}

NLPSolverGAMS::~NLPSolverGAMS() = default;

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
    for(int i = 0; i < gmoN(modelingObject); ++i)
    {
        gmoSetAltVarLowerOne(modelingObject, i, gmoGetVarLowerOne(modelingObject, i));
        gmoSetAltVarUpperOne(modelingObject, i, gmoGetVarUpperOne(modelingObject, i));
    }
}

void NLPSolverGAMS::saveOptionsToFile([[maybe_unused]] std::string fileName) {}

void NLPSolverGAMS::saveProblemToFile([[maybe_unused]] std::string fileName) {}

VectorDouble NLPSolverGAMS::getSolution()
{
    VectorDouble sol(gmoN(modelingObject));

    gmoGetVarL(modelingObject, &sol[0]);

    return sol;
}

double NLPSolverGAMS::getSolution([[maybe_unused]] int i)
{
    throw std::logic_error("getSolution(int) not implemented");
}

double NLPSolverGAMS::getObjectiveValue() { return gmoGetHeadnTail(modelingObject, gmoHobjval); }

E_NLPSolutionStatus NLPSolverGAMS::solveProblemInstance()
{
    char msg[GMS_SSSIZE];

    /* set which options file to use */
    if(!nlpsolveropt.empty())
    {
        gmoOptFileSet(modelingObject, 1);
        gmoNameOptFileSet(modelingObject, nlpsolveropt.c_str());
    }
    else
    {
        /* don't read SHOT options file */
        gmoOptFileSet(modelingObject, 0);
    }

    gmoAltBoundsSet(modelingObject, 1); /* use alternative bounds */
    gmoForceContSet(modelingObject, 1);

    if(gevCallSolver(modelingEnvironment, modelingObject, "", nlpsolver.c_str(), gevSolveLinkLoadLibrary,
           showlog ? gevSolverSameStreams : gevSolverQuiet, nullptr, nullptr, timelimit, iterlimit, 0, 0.0, 0.0,
           nullptr, msg)
        != 0)
    {
        gmoModelStatSet(modelingObject, gmoModelStat_ErrorNoSolution);
        throw std::logic_error(std::string("Calling GAMS NLP solver failed: ") + msg);
    }

    /* if not run via GAMS, then uninstall GAMS SIGINT handler, as gevTerminateGet() is not checked (is only checked if
     * called from GAMS) */
    if(env->settings->getSetting<std::string>("ProblemFile", "Input") != "")
        gevTerminateUninstall(modelingEnvironment);

    gmoAltBoundsSet(modelingObject, 0);
    gmoForceContSet(modelingObject, 0);

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
    default:
        return E_NLPSolutionStatus::Error;
    }
}

VectorDouble NLPSolverGAMS::getVariableLowerBounds()
{
    assert(modelingObject != nullptr);

    VectorDouble lb(gmoN(modelingObject));

    gmoGetVarLower(modelingObject, &lb[0]);

    return lb;
}

VectorDouble NLPSolverGAMS::getVariableUpperBounds()
{
    assert(modelingObject != nullptr);

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

std::string NLPSolverGAMS::getSolverDescription()
{
    std::string description = fmt::format("{} in GAMS {}.{}", selectedNLPSolver, GAMSMAJOR, GAMSMINOR);

    return (description);
};
} // namespace SHOT