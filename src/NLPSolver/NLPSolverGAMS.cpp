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

    // by default we want the fast solvelink=5; but if SHOT is build with Ipopt (and is not build by GAMS for GAMS),
    // then there can be conflicts between the Ipopt library linked to SHOT and the one in GAMS;
    // so in that case we switch to solvelink=2
    solvelink = gevSolveLinkLoadLibrary;

    if(nlpsolver == "auto")
    {
        assert(auditLicensing != nullptr);
        if(!palLicenseCheckSubSys(auditLicensing, (char*)"CO"))
        {
            env->output->outputDebug("        CONOPT licensed. Using CONOPT as GAMS NLP solver.");
            nlpsolver = "conopt";
            selectedNLPSolver = "CONOPT (automatically selected)";
        }
        else if(!palLicenseCheckSubSys(auditLicensing, (char*)"KN"))
        {
            env->output->outputDebug("        CONOPT not licensed. KNITRO licensed. Using KNITRO as GAMS NLP solver.");
            nlpsolver = "knitro";
            selectedNLPSolver = "KNITRO (automatically selected)";
        }
        else if(!palLicenseCheckSubSys(auditLicensing, (char*)"SN"))
        {
            env->output->outputDebug(
                "        CONOPT and KNITRO not licensed. SNOPT licensed. Using SNOPT as GAMS NLP solver.");
            nlpsolver = "snopt";
            selectedNLPSolver = "SNOPT (automatically selected)";
        }
        else if(!palLicenseCheckSubSys(auditLicensing, (char*)"M5"))
        {
            env->output->outputDebug(
                "        CONOPT, KNITRO, and SNOPT not licensed. MINOS licensed. Using MINOS as GAMS NLP solver.");
            nlpsolver = "minos";
            selectedNLPSolver = "MINOS (automatically selected)";
        }
        else if(!palLicenseCheckSubSys(auditLicensing, (char*)"IP"))
        {
            env->output->outputDebug("        CONOPT, KNITRO, SNOPT, and MINOS not licensed. IPOPTH licensed. Using "
                                     "IPOPTH as GAMS NLP solver.");
            nlpsolver = "ipopth";
            selectedNLPSolver = "IPOPTH (automatically selected)";
#if !defined(GAMS_BUILD) && defined(HAS_IPOPT)
            solvelink = gevSolveLinkCallModule;
#endif
        }
        else
        {
            env->output->outputDebug(
                "        CONOPT, KNITRO, SNOPT, MINOS, and IPOPTH not licensed. Using IPOPT as GAMS NLP solver.");
            nlpsolver = "ipopt";
            selectedNLPSolver = "IPOPT (automatically selected)";
#if !defined(GAMS_BUILD) && defined(HAS_IPOPT)
            solvelink = gevSolveLinkCallModule;
#endif
        }
    }
    else
    {
        selectedNLPSolver = nlpsolver;
        std::transform(selectedNLPSolver.begin(), selectedNLPSolver.end(), selectedNLPSolver.begin(), ::toupper);

#if !defined(GAMS_BUILD) && defined(HAS_IPOPT)
        if( selectedNLPSolver.compare(0, 5, "IPOPT", 5) == 0 )
            solvelink = gevSolveLinkCallModule;
#endif
    }

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

typedef struct
{
    Environment* env;
    gevHandle_t gev;
    void* orighandle;
    bool switchhandle;
} gevwritecallback_data;

static void GEV_CALLCONV gevwritecallback(const char* msg, int mode, void* usrmem)
{
    // mode == 2 seems to be the log, ignore status file (mode==1)
    if(mode != 2)
        return;

    // convert Pascal string to C string (which will start at msg+1)
    (const_cast<char*>(msg))[std::min(GMS_SSSIZE - 1, (int)*msg)] = '\0';

    gevwritecallback_data* cbdata = static_cast<gevwritecallback_data*>(usrmem);

    // restore original writecallback in gev, so that output below doesn't come back to this callback
    if(cbdata->switchhandle)
        gevRestoreLogStat(cbdata->gev, &cbdata->orighandle);

    // puts(msg+1);
    cbdata->env->output->outputInfo(fmt::format("      | {} ", msg + 1));

    // install this function as writecallback in gev again
    if(cbdata->switchhandle)
        gevSwitchLogStat(cbdata->gev, 3, nullptr, 0, nullptr, 0, gevwritecallback, usrmem, &cbdata->orighandle);
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

    // if we are called from GAMS (via EntryPointsGAMS), then there will be no input problem file set
    bool fromGAMS = env->settings->getSetting<std::string>("ProblemFile", "Input").empty();

    // redirect output from NLP solver to gevwritecallback
    gevwritecallback_data cbdata;
    cbdata.env = env.get();
    cbdata.gev = modelingEnvironment;
    // if run from within GAMS, then EntryPointsGAMS will have installed an spdlog sink, so that messages to
    // env->output go through modelingEnvironment since there is only one modelingEnvironment, we need to take extra
    // care that output send to gevwritecallback() gets to the original modelingEnvironment output stream
    cbdata.switchhandle = fromGAMS;

    if(showlog)
        gevSwitchLogStat(modelingEnvironment, 3, nullptr, 0, nullptr, 0, gevwritecallback, &cbdata, &cbdata.orighandle);

    if(gevCallSolver(modelingEnvironment, modelingObject, "", nlpsolver.c_str(), solvelink,
           showlog ? gevSolverSameStreams : gevSolverQuiet, nullptr, nullptr, timelimit, iterlimit, 0, 0.0, 0.0,
           nullptr, msg)
        != 0)
    {
        gmoModelStatSet(modelingObject, gmoModelStat_ErrorNoSolution);
        throw std::logic_error(std::string("Calling GAMS NLP solver failed: ") + msg);
    }

    if(showlog)
        gevRestoreLogStat(modelingEnvironment, &cbdata.orighandle);

    /* if not run via GAMS, then uninstall GAMS SIGINT handler, as gevTerminateGet() is not checked (is only checked if
     * called from GAMS) */
    if(!fromGAMS)
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
    std::string description = fmt::format("{} in GAMS", selectedNLPSolver);

    return (description);
};
} // namespace SHOT