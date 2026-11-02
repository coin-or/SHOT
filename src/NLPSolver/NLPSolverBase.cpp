/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverBase.h"

#include "../Settings.h"

#ifndef _WIN32
#include <dlfcn.h>
#endif

namespace SHOT
{

// Limits the number of threads of the BLAS and OpenMP libraries that the NLP solvers are linked with, e.g., OpenBLAS
// used by MUMPS in Ipopt; 0 leaves them unchanged. OpenMP limits only the calling thread.
static void setNumberOfThreadsInNLPLibraries(int threads)
{
#ifndef _WIN32
    if(threads <= 0)
        return;

    // SHOT is not linked with these libraries directly, and which of them are loaded depends on how the NLP solvers
    // were built, so they are looked up at run time
    using SetNumberOfThreads = void (*)(int);

    static auto openBLASSetNumberOfThreads
        = reinterpret_cast<SetNumberOfThreads>(dlsym(RTLD_DEFAULT, "openblas_set_num_threads"));
    static auto MKLSetNumberOfThreads
        = reinterpret_cast<SetNumberOfThreads>(dlsym(RTLD_DEFAULT, "MKL_Set_Num_Threads"));
    static auto OpenMPSetNumberOfThreads
        = reinterpret_cast<SetNumberOfThreads>(dlsym(RTLD_DEFAULT, "omp_set_num_threads"));

    if(openBLASSetNumberOfThreads)
        openBLASSetNumberOfThreads(threads);

    if(MKLSetNumberOfThreads)
        MKLSetNumberOfThreads(threads);

    if(OpenMPSetNumberOfThreads)
        OpenMPSetNumberOfThreads(threads);
#endif
}

NLPSolverBase::NLPSolverBase() = default;

E_NLPSolutionStatus NLPSolverBase::solveProblem()
{
    // Before each solve, since OpenMP limits only the calling thread, which can be a thread of the MIP solver in a
    // callback
    setNumberOfThreadsInNLPLibraries(env->settings->getSetting<int>("Primal.FixedInteger.NumberOfThreads"));

    auto solStatus = solveProblemInstance();

    return (solStatus);
}
} // namespace SHOT