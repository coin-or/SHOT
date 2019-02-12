/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.
   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
 */

#include "SHOTSolver.h"
#include "ModelingSystemGAMS.h"

#include "gmomcc.h"
#include "gevmcc.h"
#include "optcc.h"

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
        assert(Cptr != NULL);

        *Cptr = calloc(1, sizeof(gamsshot));
    }

    DllExport int STDCALL shtcreate(void** Cptr, char* msgBuf, int msgBufLen)
    {
        assert(Cptr != NULL);
        assert(msgBufLen > 0);
        assert(msgBuf != NULL);

        *Cptr = calloc(1, sizeof(gamsshot));

        msgBuf[0] = 0;

        return 1;
    }

    DllExport void STDCALL shtXFree(void** Cptr)
    {
        assert(Cptr != NULL);
        assert(*Cptr != NULL);

        free(*Cptr);
        *Cptr = NULL;

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

        assert(Cptr != NULL);
        assert(Gptr != NULL);

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

    DllExport int STDCALL C__shtCallSolver(void* Cptr)
    {
        gamsshot* gs;

        assert(Cptr != NULL);
        gs = (gamsshot*)Cptr;
        assert(gs->gmo != NULL);
        assert(gs->opt == NULL); /* we don't process GAMS options objects so far */

        // create solver, direct SHOT console output to GAMS log and status file
        SHOTSolver solver(std::make_shared<GamsOutputSink>((gevHandle_t)gmoEnvironment(gs->gmo)));

        /* solver.updateSetting("Console.LogLevel", "Output", static_cast<int>(ENUM_OUTPUT_LEVEL_debug)); */
        auto env = solver.getEnvironment();

        try
        {
            env->report->outputSolverHeader();

            env->timing->startTimer("ProblemInitialization");
            std::shared_ptr<ModelingSystemGAMS> modelingSystem = std::make_shared<SHOT::ModelingSystemGAMS>(env);

            SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
            if(modelingSystem->createProblem(problem, gs->gmo) != E_ProblemCreationStatus::NormalCompletion)
            {
                gmoSolveStatSet(gs->gmo, gmoSolveStat_Capability);
                gmoModelStatSet(gs->gmo, gmoModelStat_NoSolutionReturned);
                return 0;
            }

            env->settings->updateSetting("SourceFormat", "Input", static_cast<int>(ES_SourceFormat::GAMS));
            env->timing->stopTimer("ProblemInitialization");

            /* correct to call this here? */
            modelingSystem->updateSettings(env->settings);

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
        catch(const ErrorClass& eclass)
        {
            env->output->outputError(eclass.errormsg);

            gmoSolveStatSet(gs->gmo, gmoSolveStat_Solver);
            gmoModelStatSet(gs->gmo, gmoModelStat_ErrorNoSolution);

            return (0);
        }

        return 0;
    }

    DllExport int STDCALL C__shtHaveModifyProblem(void* Cptr) { return 0; }

    DllExport int STDCALL C__shtModifyProblem(void* Cptr)
    {
        assert(Cptr != NULL);
        return 1;
    }
}
