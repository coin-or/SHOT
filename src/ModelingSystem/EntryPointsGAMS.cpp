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
        optHandle_t opt;
        palHandle_t pal;
    } gamsshot;

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

    DllExport int STDCALL shtcreate(void** Cptr, char* msgBuf, int msgBufLen);
    DllExport int STDCALL shtcreate(void** Cptr, char* msgBuf, int msgBufLen)
    {
        assert(Cptr != nullptr);
        assert(msgBuf != nullptr);

        *Cptr = NULL;

        if( !gmoGetReady(msgBuf, msgBufLen) )
           return 0;

        if( !gevGetReady(msgBuf, msgBufLen) )
           return 0;

        if( !palGetReady(msgBuf, msgBufLen) )
           return 0;

        *Cptr = calloc(1, sizeof(gamsshot));
        if( *Cptr == NULL )
        {
           snprintf(msgBuf, msgBufLen, "Out of memory when creating gamsshot object.\n");
           if( msgBufLen > 0 )
              msgBuf[msgBufLen] = '\0';
           return 0;
        }

        return 1;
    }

    DllExport void STDCALL shtfree(void** Cptr);
    DllExport void STDCALL shtfree(void** Cptr)
    {
        assert(Cptr != nullptr);
        assert(*Cptr != nullptr);

        if(((gamsshot*)*Cptr)->pal != NULL)
            palFree(&((gamsshot*)*Cptr)->pal);

        free(*Cptr);
        *Cptr = nullptr;

        gmoLibraryUnload();
        gevLibraryUnload();
        palLibraryUnload();
    }

    DllExport int STDCALL shtReadyAPI(void* Cptr, gmoHandle_t Gptr, optHandle_t Optr);
    DllExport int STDCALL shtReadyAPI(void* Cptr, gmoHandle_t Gptr, optHandle_t Optr)
    {
        gamsshot* gs;
        gevHandle_t gev;
        char msg[256];

        assert(Cptr != nullptr);
        assert(Gptr != nullptr);

        gs = (gamsshot*)Cptr;
        gs->gmo = Gptr;
        gs->opt = Optr;

        gev = (gevHandle_t)gmoEnvironment(Gptr);
        if(!palCreate(&gs->pal, msg, sizeof(msg)))
        {
            gevLogStat(gev, msg);
            return 1;
        }

        /* print auditline */
#ifdef GAMS_BUILD
#define PALPTR gs->pal
#include "shotCLsvn.h"
        palGetAuditLine(gs->pal, msg);
        gevLogStat(gev, "");
        gevLogStat(gev, msg);
        gevStatAudit(gev, msg);
#endif

        /* initialize licensing */
        palLicenseRegisterGAMS(gs->pal, 1, gevGetStrOpt(gev, "License1", msg));
        palLicenseRegisterGAMS(gs->pal, 2, gevGetStrOpt(gev, "License2", msg));
        palLicenseRegisterGAMS(gs->pal, 3, gevGetStrOpt(gev, "License3", msg));
        palLicenseRegisterGAMS(gs->pal, 4, gevGetStrOpt(gev, "License4", msg));
        palLicenseRegisterGAMS(gs->pal, 5, gevGetStrOpt(gev, "License5", msg));
        palLicenseRegisterGAMSDone(gs->pal);
        /* palLicenseCheck(pal,gmoM(gmo),gmoN(gmo),gmoNZ(gmo),gmoNLNZ(gmo),gmoNDisc(gmo)); */

        return 0;
    }

    DllExport int STDCALL shtCallSolver(void* Cptr);
    DllExport int STDCALL shtCallSolver(void* Cptr)
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
        catch(const std::exception& e)
        {
            env->output->outputError(fmt::format("Error when solving problem with GAMS: {}", e.what()));

            gmoSolveStatSet(gs->gmo, gmoSolveStat_Solver);
            gmoModelStatSet(gs->gmo, gmoModelStat_ErrorNoSolution);

            return (0);
        }

        return 0;
    }

    DllExport void STDCALL C__shtInitialize(void);
    DllExport void STDCALL C__shtInitialize(void)
    {
       shtInitialize();
    }

    DllExport void STDCALL C__shtFinalize(void);
    DllExport void STDCALL C__shtFinalize(void)
    {
       shtFinalize();
    }

    DllExport void STDCALL shtXCreate(void** Cptr);
    DllExport void STDCALL shtXCreate(void** Cptr)
    {
        char msg[GMS_SSSIZE];
        shtcreate(Cptr, msg, sizeof(msg));
    }

    DllExport void STDCALL shtXFree(void** Cptr);
    DllExport void STDCALL shtXFree(void** Cptr)
    {
        shtfree(Cptr);
    }

    DllExport int  STDCALL C__shtReadyAPI(void* Cptr, struct gmoRec* Gptr, struct optRec* Optr);
    DllExport int  STDCALL C__shtReadyAPI(void* Cptr, struct gmoRec* Gptr, struct optRec* Optr)
    {
        return shtReadyAPI(Cptr, Gptr, Optr);
    }

    DllExport int  STDCALL C__shtCallSolver(void* Cptr);
    DllExport int  STDCALL C__shtCallSolver(void* Cptr)
    {
        return shtCallSolver(Cptr);
    }

    DllExport int STDCALL C__shtXAPIVersion(int api, char* Msg, int* comp);
    DllExport int STDCALL C__shtXAPIVersion([[maybe_unused]] int api, [[maybe_unused]] char* Msg, int* comp)
    {
        *comp = 1;
        return 1;
    }

    DllExport int STDCALL D__shtXAPIVersion(int api, char* Msg, int* comp);
    DllExport int STDCALL D__shtXAPIVersion([[maybe_unused]] int api, [[maybe_unused]] char* Msg, int* comp)
    {
        *comp = 1;
        return 1;
    }

    DllExport int STDCALL C__shtXCheck(const char* funcn, int ClNrArg, int Clsign[], char* Msg);
    DllExport int STDCALL C__shtXCheck([[maybe_unused]] const char* funcn, [[maybe_unused]] int ClNrArg, [[maybe_unused]] int Clsign[], [[maybe_unused]] char* Msg)
    {
        return 1;
    }

    DllExport int STDCALL D__shtXCheck(const char* funcn, int ClNrArg, int Clsign[], char* Msg);
    DllExport int STDCALL D__shtXCheck([[maybe_unused]] const char* funcn, [[maybe_unused]] int ClNrArg, [[maybe_unused]] int Clsign[], [[maybe_unused]] char* Msg)
    {
        return 1;
    }
}
