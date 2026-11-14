/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverGAMS.h"

#include "../Output.h"
#include "../ModelingSystem/ModelingSystemGAMS.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <chrono>

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

    auto modelingSystem = std::dynamic_pointer_cast<ModelingSystemGAMS>(env->modelingSystem);

    if(modelingSystem && modelingSystem->modelingObject == modelingObject)
    {
        if(!modelingSystem->nlpLicenseState)
            modelingSystem->nlpLicenseState = std::make_shared<GAMSNLPLicenseState>();
        licenseState = modelingSystem->nlpLicenseState;
    }
    else
        licenseState = std::make_shared<GAMSNLPLicenseState>();

    nlpsolver = env->settings->getSetting<std::string>("Subsolver.GAMS.NLP.Solver");
    nlpsolveropt = env->settings->getSetting<std::string>("Subsolver.GAMS.NLP.OptionsFilename");

    timelimit = env->settings->getSetting<double>("Primal.FixedInteger.TimeLimit");
    // Without an iteration limit, the GAMS NLP solver has none
    if(int iterationLimit = env->settings->getSetting<int>("Primal.FixedInteger.IterationLimit"); iterationLimit > 0)
        iterlimit = iterationLimit;

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
        if(selectedNLPSolver.compare(0, 5, "IPOPT", 5) == 0)
            solvelink = gevSolveLinkCallModule;
#endif
    }

    showlog = env->settings->getSetting<bool>("Output.Console.PrimalSolver.Show");

    lowerBounds.resize(gmoN(modelingObject));
    upperBounds.resize(gmoN(modelingObject));

    if(gmoN(modelingObject) > 0)
    {
        gmoGetVarLower(modelingObject, lowerBounds.data());
        gmoGetVarUpper(modelingObject, upperBounds.data());
    }
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

void NLPSolverGAMS::clearStartingPoint() { }

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
    // The bounds of the GAMS model would discard those tightened through updateVariableLowerBound() and
    // updateVariableUpperBound()
    for(int i = 0; i < gmoN(modelingObject); ++i)
    {
        gmoSetAltVarLowerOne(modelingObject, i, lowerBounds[i]);
        gmoSetAltVarUpperOne(modelingObject, i, upperBounds[i]);
    }
}

void NLPSolverGAMS::saveOptionsToFile([[maybe_unused]] std::string fileName) { }

void NLPSolverGAMS::saveProblemToFile([[maybe_unused]] std::string fileName) { }

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
    auto callStarted = std::chrono::steady_clock::now();

    if(timeLimit <= 0.0
        || env->timing->getElapsedTime("Total") >= env->settings->getSetting<double>("Termination.TimeLimit"))
        return E_NLPSolutionStatus::TimeLimit;

    auto& unavailable = licenseState->unavailableSolvers;

    if(unavailable.count(getSelectedSolverName()))
    {
        if(getSelectedSolverName() == "IPOPT")
            return E_NLPSolutionStatus::Error;

        useLicenseFallback();

        if(unavailable.count(getSelectedSolverName()))
            return E_NLPSolutionStatus::Error;
    }

    char msg[GMS_SSSIZE] = { };

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

    int originalAltBounds = gmoAltBounds(modelingObject);
    int originalForceCont = gmoForceCont(modelingObject);
    gmoAltBoundsSet(modelingObject, 1); /* use alternative bounds */
    gmoForceContSet(modelingObject, 1);

    // if we are called from GAMS (via EntryPointsGAMS), then there will be no input problem file set
    bool fromGAMS = env->settings->getSetting<std::string>("Input.ProblemFile").empty();

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

    // GAMS' resource limit is relative to this call. Cap it by the time left for the entire SHOT solve as well as
    // the fixed-NLP and task limits, like the linked Ipopt and Uno interfaces do.
    const double timeLeft
        = env->settings->getSetting<double>("Termination.TimeLimit") - env->timing->getElapsedTime("Total");
    const double solveTimeLimit = std::max(std::min({ timelimit, timeLimit, timeLeft }), 1e-5);

    // Do not mistake a status left by the previous solver for the result of a failed invocation.
    gmoModelStatSet(modelingObject, gmoModelStat_ErrorNoSolution);

    gmoSolveStatSet(modelingObject, gmoSolveStat_Solver);
    int callStatus = callSolver(solveTimeLimit, msg);
    bool licenseError = gmoModelStat(modelingObject) == gmoModelStat_LicenseError
        || gmoSolveStat(modelingObject) == gmoSolveStat_License;

    if(showlog)
        gevRestoreLogStat(modelingEnvironment, &cbdata.orighandle);

    /* if not run via GAMS, then uninstall GAMS SIGINT handler, as gevTerminateGet() is not checked (is only checked if
     * called from GAMS) */
    if(!fromGAMS)
        gevTerminateUninstall(modelingEnvironment);

    gmoAltBoundsSet(modelingObject, originalAltBounds);
    gmoForceContSet(modelingObject, originalForceCont);

    if(licenseError)
    {
        auto name = getSelectedSolverName();

        if(unavailable.insert(name).second)
            env->output->outputWarning(name == "IPOPT"
                    ? " GAMS IPOPT licence error; further GAMS NLP calls skipped."
                    : fmt::format(" GAMS {} licence error; switching to IPOPT.", name));

        if(name == "IPOPT")
            return E_NLPSolutionStatus::Error;

        useLicenseFallback();
        double remainingTime = std::min(timelimit, timeLimit)
            - std::chrono::duration<double>(std::chrono::steady_clock::now() - callStarted).count();

        if(remainingTime <= 0.0)
            return E_NLPSolutionStatus::TimeLimit;

        double originalTimeLimit = timeLimit;
        timeLimit = remainingTime;

        try
        {
            auto status = solveProblemInstance();
            timeLimit = originalTimeLimit;
            return status;
        }
        catch(...)
        {
            timeLimit = originalTimeLimit;
            throw;
        }
    }

    if(callStatus != 0)
    {
        gmoModelStatSet(modelingObject, gmoModelStat_ErrorNoSolution);
        throw std::logic_error(std::string("Calling GAMS NLP solver failed: ") + msg);
    }

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

int NLPSolverGAMS::callSolver(double solveTimeLimit, char* message)
{
    return gevCallSolver(modelingEnvironment, modelingObject, "", nlpsolver.c_str(), solvelink,
        showlog ? gevSolverSameStreams : gevSolverQuiet, nullptr, nullptr, solveTimeLimit, iterlimit, 0, 0.0, 0.0,
        nullptr, message);
}

void NLPSolverGAMS::useLicenseFallback()
{
    nlpsolver = "ipopt";
    selectedNLPSolver = "IPOPT (licence fallback)";
    // Options for the unavailable solver need not be valid Ipopt options.
    nlpsolveropt.clear();
    solvelink = gevSolveLinkLoadLibrary;
#if !defined(GAMS_BUILD) && defined(HAS_IPOPT)
    solvelink = gevSolveLinkCallModule;
#endif
    env->results->usedPrimalGAMSNLPSolver = getSelectedSolverName();
    env->results->usedPrimalNLPSolverDescription = getSolverDescription();
}

VectorDouble NLPSolverGAMS::getVariableLowerBounds() { return lowerBounds; }

VectorDouble NLPSolverGAMS::getVariableUpperBounds() { return upperBounds; }

void NLPSolverGAMS::updateVariableLowerBound(int variableIndex, double bound)
{
    lowerBounds.at(variableIndex) = bound;
    gmoSetAltVarLowerOne(modelingObject, variableIndex, bound);
}

void NLPSolverGAMS::updateVariableUpperBound(int variableIndex, double bound)
{
    upperBounds.at(variableIndex) = bound;
    gmoSetAltVarUpperOne(modelingObject, variableIndex, bound);
}

std::string NLPSolverGAMS::getSolverDescription()
{
    std::string description = fmt::format("{} in GAMS", selectedNLPSolver);

    return (description);
};

std::string NLPSolverGAMS::getSelectedSolverName()
{
    std::string name = nlpsolver;
    std::transform(name.begin(), name.end(), name.begin(), ::toupper);

    return (name);
};
} // namespace SHOT
