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
#include "../Structs.h"
#include "../TaskHandler.h"
#include "../Timing.h"

#include "../Model/Problem.h"

#include "gmomcc.h"
#include "gevmcc.h"
#include "palmcc.h"
#include "optcc.h"

using namespace SHOT;

extern "C"
{

    typedef struct
    {
        gmoHandle_t gmo;
        optHandle_t opt;
        palHandle_t pal;
    } gamsshot;

    DllExport void STDCALL shtXCreate(void** Cptr);
    DllExport void STDCALL shtXCreate(void** Cptr)
    {
        assert(Cptr != nullptr);

        *Cptr = calloc(1, sizeof(gamsshot));
    }

    DllExport int STDCALL shtcreate(void** Cptr, char* msgBuf, int msgBufLen);
    DllExport int STDCALL shtcreate(void** Cptr, char* msgBuf, int msgBufLen)
    {
        assert(Cptr != nullptr);
        assert(msgBufLen > 0);
        assert(msgBuf != nullptr);

        *Cptr = calloc(1, sizeof(gamsshot));

        msgBuf[0] = 0;

        return 1;
    }

    DllExport void STDCALL shtXFree(void** Cptr);
    DllExport void STDCALL shtXFree(void** Cptr)
    {
        assert(Cptr != nullptr);
        assert(*Cptr != nullptr);

        if(((gamsshot*)*Cptr)->pal != NULL)
            palFree(&((gamsshot*)*Cptr)->pal);

        free(*Cptr);
        *Cptr = nullptr;

        gmoLibraryUnload();
        gevLibraryUnload();
#ifdef GAMS_BUILD
        palLibraryUnload();
#endif
    }

    DllExport int STDCALL shtfree(void** Cptr);
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
    DllExport int STDCALL C__shtXAPIVersion([[maybe_unused]] int api, [[maybe_unused]] char* Msg, int* comp);
    DllExport int STDCALL C__shtXAPIVersion([[maybe_unused]] int api, [[maybe_unused]] char* Msg, int* comp)
    {
        *comp = 1;
        return 1;
    }

    DllExport int STDCALL D__shtXAPIVersion([[maybe_unused]] int api, [[maybe_unused]] char* Msg, int* comp);
    DllExport int STDCALL D__shtXAPIVersion([[maybe_unused]] int api, [[maybe_unused]] char* Msg, int* comp)
    {
        *comp = 1;
        return 1;
    }

    DllExport int STDCALL C__shtXCheck([[maybe_unused]] const char* funcn, [[maybe_unused]] int ClNrArg,
                                       [[maybe_unused]] int Clsign[], [[maybe_unused]] char* Msg);
    DllExport int STDCALL C__shtXCheck([[maybe_unused]] const char* funcn, [[maybe_unused]] int ClNrArg,
        [[maybe_unused]] int Clsign[], [[maybe_unused]] char* Msg)
    {
        return 1;
    }

    DllExport int STDCALL D__shtXCheck([[maybe_unused]] const char* funcn, [[maybe_unused]] int ClNrArg,
        [[maybe_unused]] int Clsign[], [[maybe_unused]] char* Msg);
    DllExport int STDCALL D__shtXCheck([[maybe_unused]] const char* funcn, [[maybe_unused]] int ClNrArg,
        [[maybe_unused]] int Clsign[], [[maybe_unused]] char* Msg)
    {
        return 1;
    }

    DllExport int STDCALL C__shtReadyAPI(void* Cptr, gmoHandle_t Gptr, optHandle_t Optr);
    DllExport int STDCALL C__shtReadyAPI(void* Cptr, gmoHandle_t Gptr, optHandle_t Optr)
    {
        gamsshot* gs;
#ifdef GAMS_BUILD
        gevHandle_t gev;
#endif

        assert(Cptr != nullptr);
        assert(Gptr != nullptr);

        char msg[256];
        if(!gmoGetReady(msg, sizeof(msg)))
        {
            fputs(msg, stderr);
            return 1;
        }
        if(!gevGetReady(msg, sizeof(msg)))
        {
            fputs(msg, stderr);
            return 1;
        }

        gs = (gamsshot*)Cptr;
        gs->gmo = Gptr;
        gs->opt = Optr;

#ifdef GAMS_BUILD
        gev = (gevHandle_t)gmoEnvironment(Gptr);
        if(!palGetReady(msg, sizeof(msg)))
        {
            gevLogStat(gev, msg);
            return 1;
        }
        if(!palCreate(&gs->pal, msg, sizeof(msg)))
        {
            gevLogStat(gev, msg);
            return 1;
        }

        /* print auditline */
#define PALPTR gs->pal
#include "shotCLsvn.h"
        palGetAuditLine(gs->pal, msg);
        gevLogStat(gev, "");
        gevLogStat(gev, msg);
        gevStatAudit(gev, msg);

        /* initialize licensing */
        palLicenseRegisterGAMS(gs->pal, 1, gevGetStrOpt(gev, "License1", msg));
        palLicenseRegisterGAMS(gs->pal, 2, gevGetStrOpt(gev, "License2", msg));
        palLicenseRegisterGAMS(gs->pal, 3, gevGetStrOpt(gev, "License3", msg));
        palLicenseRegisterGAMS(gs->pal, 4, gevGetStrOpt(gev, "License4", msg));
        palLicenseRegisterGAMS(gs->pal, 5, gevGetStrOpt(gev, "License5", msg));
        palLicenseRegisterGAMSDone(gs->pal);
        /* palLicenseCheck(pal,gmoM(gmo),gmoN(gmo),gmoNZ(gmo),gmoNLNZ(gmo),gmoNDisc(gmo)); */
#endif

        return 0;
    }

    DllExport int STDCALL C__shtCallSolver(void* Cptr);
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

            std::shared_ptr<ModelingSystemGAMS> modelingSystem = std::make_shared<SHOT::ModelingSystemGAMS>(env);
            modelingSystem->setModelingObject(gs->gmo);
            modelingSystem->updateSettings(env->settings, gs->pal);

            env->timing->startTimer("ProblemInitialization");
            SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
            switch(modelingSystem->createProblem(problem))
            {
            case E_ProblemCreationStatus::NormalCompletion:
                break;
            case E_ProblemCreationStatus::CapabilityProblem:
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

    DllExport int STDCALL C__shtHaveModifyProblem([[maybe_unused]] void* Cptr);
    DllExport int STDCALL C__shtHaveModifyProblem([[maybe_unused]] void* Cptr) { return 0; }

    DllExport int STDCALL C__shtModifyProblem(void* Cptr);
    DllExport int STDCALL C__shtModifyProblem(void* Cptr)
    {
        assert(Cptr != nullptr);
        return 1;
    }
}
