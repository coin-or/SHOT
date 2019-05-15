/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.
   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
 */

#include "ModelingSystemGAMS.h"

#include "../Output.h"
#include "../Report.h"
#include "../Settings.h"
#include "../Solver.h"
#include "../TaskHandler.h"
#include "../Timing.h"

#include "../Model/Problem.h"

#include "gmomcc.h"
#include "gevmcc.h"
#include "optcc.h"
#include "palmcc.h"

#if defined(_WIN32)
#if !defined(STDCALL)
#define STDCALL __stdcall
#endif
#if !defined(DllExport)
#define DllExport __declspec(dllexport)
#endif
#else
#if !defined(STDCALL)
#define STDCALL
#endif
#if !defined(DllExport)
#define DllExport
#endif
#endif

using namespace SHOT;

extern "C"
{

    typedef struct
    {
        gmoHandle_t gmo;
        optHandle_t opt;
    } gamsshot;

    DllExport void STDCALL shtXCreate(void** Cptr)
    {
        assert(Cptr != nullptr);

        *Cptr = calloc(1, sizeof(gamsshot));
    }

    DllExport int STDCALL shtcreate(void** Cptr, char* msgBuf, int msgBufLen)
    {
        assert(Cptr != nullptr);
        assert(msgBufLen > 0);
        assert(msgBuf != nullptr);

        *Cptr = calloc(1, sizeof(gamsshot));

        msgBuf[0] = 0;

        return 1;
    }

    DllExport void STDCALL shtXFree(void** Cptr)
    {
        assert(Cptr != nullptr);
        assert(*Cptr != nullptr);

        free(*Cptr);
        *Cptr = nullptr;

        gmoLibraryUnload();
        gevLibraryUnload();
    }

    DllExport int STDCALL shtfree(void** Cptr)
    {
        shtXFree(Cptr);

        return 1;
    }

    /* comp returns the compatibility mode:
           0: client is too old for the DLL, no compatibility
           1: client version and DLL version are the same, full compatibility
           2: client is older than DLL, but defined as compatible, backward compatibility
           3: client is newer than DLL, forward compatibility
           FIXME: for now, we just claim full compatibility
     */
    DllExport int STDCALL C__shtXAPIVersion(int api, char* Msg, int* comp)
    {
        *comp = 1;
        return 1;
    }

    DllExport int STDCALL D__shtXAPIVersion(int api, char* Msg, int* comp)
    {
        *comp = 1;
        return 1;
    }

    DllExport int STDCALL C__shtXCheck(const char* funcn, int ClNrArg, int Clsign[], char* Msg) { return 1; }

    DllExport int STDCALL D__shtXCheck(const char* funcn, int ClNrArg, int Clsign[], char* Msg) { return 1; }

    DllExport int STDCALL C__shtReadyAPI(void* Cptr, gmoHandle_t Gptr, optHandle_t Optr)
    {
        gamsshot* gs;

        assert(Cptr != nullptr);
        assert(Gptr != nullptr);

        char msg[256];
        if(!gmoGetReady(msg, sizeof(msg)))
            return 1;
        if(!gevGetReady(msg, sizeof(msg)))
            return 1;

        gs = (gamsshot*)Cptr;
        gs->gmo = Gptr;
        gs->opt = Optr;

        return 0;
    }

    static
    bool doLicenseChecks(
        void*            Cptr,
        Solver&          solver
        )
    {
#ifdef GAMS_BUILD
        gamsshot* gs;

        assert(Cptr != nullptr);
        gs = (gamsshot*)Cptr;
        assert(gs->gmo != nullptr);
        gevHandle_t gev = (gevHandle_t)gmoEnvironment(gs->gmo);

        if( solver.getEnvironment()->settings->getSetting<int>("MIP.Solver", "Dual") == (int)ES_MIPSolver::Cplex )
        {
            char buffer[GMS_SSSIZE];

            palHandle_t pal;
            if( !palCreate(&pal, buffer, sizeof(buffer)) )
            {
                gevLogStat(gev, buffer);
                gmoSolveStatSet(gs->gmo, gmoSolveStat_SystemErr);
                gmoModelStatSet(gs->gmo, gmoModelStat_ErrorNoSolution);
                return false;
            }

            if( !palLicenseIsDemoCheckout(pal) && palLicenseCheckSubSys(pal, const_cast<char*>("OCCPCL")) )
            {
                // TODO if user set CPLEX explicitly, then we should stop
                gevLogStat(gev, " CPLEX chosen as MIP solver, but no CPLEX license available. Changing to CBC.\n");
                //gmoSolveStatSet(gs->gmo, gmoSolveStat_License);
                //gmoModelStatSet(gs->gmo, gmoModelStat_LicenseError);
                //return false;
                solver.getEnvironment()->settings->updateSetting("MIP.Solver", "Dual", (int)ES_MIPSolver::Cbc);
            }
        }
#endif
        return true;
    }

    DllExport int STDCALL C__shtCallSolver(void* Cptr)
    {
        gamsshot* gs;

        assert(Cptr != nullptr);
        gs = (gamsshot*)Cptr;
        assert(gs->gmo != nullptr);
        assert(gs->opt == nullptr); /* we don't process GAMS options objects so far */

        // create solver, direct SHOT console output to GAMS log and status file
        Solver solver(std::make_shared<GamsOutputSink>((gevHandle_t)gmoEnvironment(gs->gmo)));

        auto env = solver.getEnvironment();

        try
        {
            env->report->outputSolverHeader();

            env->timing->startTimer("ProblemInitialization");
            std::shared_ptr<ModelingSystemGAMS> modelingSystem = std::make_shared<SHOT::ModelingSystemGAMS>(env);

            SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
            switch(modelingSystem->createProblem(problem, gs->gmo))
            {
                case E_ProblemCreationStatus::NormalCompletion :
                    break;
                case E_ProblemCreationStatus::CapabilityProblem :
                    gmoSolveStatSet(gs->gmo, gmoSolveStat_Capability);
                    gmoModelStatSet(gs->gmo, gmoModelStat_NoSolutionReturned);
                    return 0;
                default:
                    gmoSolveStatSet(gs->gmo, gmoSolveStat_SetupErr);
                    gmoModelStatSet(gs->gmo, gmoModelStat_ErrorNoSolution);
                    return 0;
            }

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::GAMS));
            env->timing->stopTimer("ProblemInitialization");

            /* correct to call this here? */
            modelingSystem->updateSettings(env->settings);

            // check for licenses on commercial solvers, if used
            if( !doLicenseChecks(Cptr, solver) )
                return 0;

            solver.registerCallback(
                E_EventType::UserTerminationCheck, [&env, gev = (gevHandle_t)gmoEnvironment(gs->gmo)] {
                    if(gevTerminateGet(gev))
                        env->tasks->terminate();
                });

            solver.setProblem(problem, modelingSystem);

            env->report->outputOptionsReport();
            env->report->outputProblemInstanceReport();

            if(!solver.solveProblem()) // solve problem
            {
                env->output->outputError(" Error when solving problem.");

                gmoSolveStatSet(gs->gmo, gmoSolveStat_Solver);
                gmoModelStatSet(gs->gmo, gmoModelStat_ErrorNoSolution);

                return (0);
            }

            env->report->outputSolutionReport();

            // pass solution info, etc. to GAMS
            modelingSystem->finalizeSolution();
        }
        catch(const Error& eclass)
        {
            env->output->outputError(eclass.message);

            gmoSolveStatSet(gs->gmo, gmoSolveStat_Solver);
            gmoModelStatSet(gs->gmo, gmoModelStat_ErrorNoSolution);

            return (0);
        }
        catch(const std::exception& e)
        {
            env->output->outputError(std::string("Error: ") + e.what());

            gmoSolveStatSet(gs->gmo, gmoSolveStat_Solver);
            gmoModelStatSet(gs->gmo, gmoModelStat_ErrorNoSolution);

            return (0);
        }

        return 0;
    }

    DllExport int STDCALL C__shtHaveModifyProblem(void* Cptr) { return 0; }

    DllExport int STDCALL C__shtModifyProblem(void* Cptr)
    {
        assert(Cptr != nullptr);
        return 1;
    }
}
