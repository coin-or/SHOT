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

#if defined(__linux) && defined(HAS_CPLEX)
#include "ilcplex/cplex.h"
#endif

using namespace SHOT;

extern "C"
{

    typedef struct
    {
        gmoHandle_t gmo;
    } gamsshot;

    // old+new API
    DllExport void STDCALL shtInitialize(void);
    DllExport void STDCALL shtInitialize(void)
    {
#if defined(__linux) && defined(HAS_CPLEX)
        CPXinitialize();
#endif

        gmoInitMutexes();
        gevInitMutexes();
        palInitMutexes();
    }

    // old+new API
    DllExport void STDCALL shtFinalize(void);
    DllExport void STDCALL shtFinalize(void)
    {
#if defined(__linux) && defined(HAS_CPLEX)
        CPXfinalize();
#endif

        gmoFiniMutexes();
        gevFiniMutexes();
        palFiniMutexes();
    }

    // old API
    DllExport int STDCALL shtcreate(void** Cptr, char* msgBuf, int msgBufLen);
    DllExport int STDCALL shtcreate(void** Cptr, char* msgBuf, int msgBufLen)
    {
        assert(Cptr != nullptr);
        assert(msgBuf != nullptr);

        *Cptr = NULL;

        if(!gmoGetReady(msgBuf, msgBufLen))
            return 0;

        if(!gevGetReady(msgBuf, msgBufLen))
            return 0;

        if(!palGetReady(msgBuf, msgBufLen))
            return 0;

        *Cptr = calloc(1, sizeof(gamsshot));
        if(*Cptr == NULL)
        {
            snprintf(msgBuf, msgBufLen, "Out of memory when creating gamsshot object.\n");
            if(msgBufLen > 0)
                msgBuf[msgBufLen] = '\0';
            return 0;
        }

        return 1;
    }

    // new API
    DllExport int STDCALL shtCreate(void** Cptr, char* msgBuf, int msgBufLen);
    DllExport int STDCALL shtCreate(void** Cptr, char* msgBuf, int msgBufLen)
    {
        return 1 - shtcreate(Cptr, msgBuf, msgBufLen);
    }

    // old API
    DllExport void STDCALL shtfree(void** Cptr);
    DllExport void STDCALL shtfree(void** Cptr)
    {
        assert(Cptr != nullptr);
        assert(*Cptr != nullptr);

        free(*Cptr);
        *Cptr = nullptr;

        gmoLibraryUnload();
        gevLibraryUnload();
        palLibraryUnload();
    }

    // new API
    DllExport void STDCALL shtFree(void** Cptr);
    DllExport void STDCALL shtFree(void** Cptr) { shtfree(Cptr); }

    // old+new API (old API ignores additional optHandle_t)
    DllExport int STDCALL shtReadyAPI(void* Cptr, gmoHandle_t Gptr);
    DllExport int STDCALL shtReadyAPI(void* Cptr, gmoHandle_t Gptr)
    {
        gamsshot* gs;

        assert(Cptr != nullptr);
        assert(Gptr != nullptr);

        gs = (gamsshot*)Cptr;
        gs->gmo = Gptr;

        return 0;
    }

    // old+new API
    DllExport int STDCALL shtCallSolver(void* Cptr);
    DllExport int STDCALL shtCallSolver(void* Cptr)
    {
        gamsshot* gs;
        char msg[GMS_SSSIZE];

        assert(Cptr != nullptr);
        gs = (gamsshot*)Cptr;
        assert(gs->gmo != nullptr);

        // create solver, direct SHOT console output to GAMS log and status file
        Solver solver(std::make_shared<GamsOutputSink>((gevHandle_t)gmoEnvironment(gs->gmo)));

        auto env = solver.getEnvironment();

        try
        {
            std::shared_ptr<ModelingSystemGAMS> modelingSystem = std::make_shared<SHOT::ModelingSystemGAMS>(env);
            modelingSystem->setModelingObject(gs->gmo);

#if PALAPIVERSION >= 3
            /* print auditline */
            palSetSystemName(modelingSystem->auditLicensing, "SHOT");
            palGetAuditLine(modelingSystem->auditLicensing, msg);
            env->output->outputInfo("");
            env->output->outputInfo(msg);
            gevStatAudit(modelingSystem->modelingEnvironment, msg);
#endif

            env->report->outputSolverHeader();

            modelingSystem->updateSettings(env->settings);

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
                E_EventType::UserTerminationCheck, [&env, gev = (gevHandle_t)gmoEnvironment(gs->gmo)](std::any args) {
                    if(gevTerminateGet(gev))
                        env->tasks->terminate();
                });

            if(!solver.setProblem(problem, modelingSystem))
            {
                env->output->outputError(" Error when initializing problem.");
                gmoSolveStatSet(gs->gmo, gmoSolveStat_SetupErr);
                gmoModelStatSet(gs->gmo, gmoModelStat_ErrorNoSolution);
                return 0;
            }

            env->report->outputProblemInstanceReport();
            env->report->outputOptionsReport();

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
        catch(const std::exception& e)
        {
            env->output->outputError(fmt::format("Error when solving problem: {}", e.what()));

            gmoSolveStatSet(gs->gmo, gmoSolveStat_Solver);
            gmoModelStatSet(gs->gmo, gmoModelStat_ErrorNoSolution);

            return (0);
        }

        return 0;
    }

    // old API
    DllExport void STDCALL C__shtInitialize(void);
    DllExport void STDCALL C__shtInitialize(void) { shtInitialize(); }

    // old API
    DllExport void STDCALL C__shtFinalize(void);
    DllExport void STDCALL C__shtFinalize(void) { shtFinalize(); }

    // old API
    DllExport void STDCALL shtXCreate(void** Cptr);
    DllExport void STDCALL shtXCreate(void** Cptr)
    {
        char msg[GMS_SSSIZE];
        shtcreate(Cptr, msg, sizeof(msg));
    }

    // old API
    DllExport void STDCALL shtXFree(void** Cptr);
    DllExport void STDCALL shtXFree(void** Cptr) { shtfree(Cptr); }

    // old API
    DllExport int STDCALL C__shtReadyAPI(void* Cptr, struct gmoRec* Gptr, struct optRec* Optr);
    DllExport int STDCALL C__shtReadyAPI(void* Cptr, struct gmoRec* Gptr, struct optRec* Optr)
    {
        return shtReadyAPI(Cptr, Gptr);
    }

    // old API
    DllExport int STDCALL C__shtCallSolver(void* Cptr);
    DllExport int STDCALL C__shtCallSolver(void* Cptr) { return shtCallSolver(Cptr); }

    // old API
    DllExport int STDCALL C__shtXAPIVersion(int api, char* Msg, int* comp);
    DllExport int STDCALL C__shtXAPIVersion([[maybe_unused]] int api, [[maybe_unused]] char* Msg, int* comp)
    {
        *comp = 1;
        return 1;
    }

    // old API
    DllExport int STDCALL D__shtXAPIVersion(int api, char* Msg, int* comp);
    DllExport int STDCALL D__shtXAPIVersion([[maybe_unused]] int api, [[maybe_unused]] char* Msg, int* comp)
    {
        *comp = 1;
        return 1;
    }

    // old API
    DllExport int STDCALL C__shtXCheck(const char* funcn, int ClNrArg, int Clsign[], char* Msg);
    DllExport int STDCALL C__shtXCheck([[maybe_unused]] const char* funcn, [[maybe_unused]] int ClNrArg,
        [[maybe_unused]] int Clsign[], [[maybe_unused]] char* Msg)
    {
        return 1;
    }

    // old API
    DllExport int STDCALL D__shtXCheck(const char* funcn, int ClNrArg, int Clsign[], char* Msg);
    DllExport int STDCALL D__shtXCheck([[maybe_unused]] const char* funcn, [[maybe_unused]] int ClNrArg,
        [[maybe_unused]] int Clsign[], [[maybe_unused]] char* Msg)
    {
        return 1;
    }
}
