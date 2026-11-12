/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverBase.h"

#include "../Settings.h"

#include <cstdlib>
#ifndef _WIN32
#include <dlfcn.h>
#endif

#ifdef _OPENMP
extern "C" void omp_set_num_threads(int);
#endif

namespace SHOT
{

// Limits the number of threads of the BLAS and OpenMP libraries that the NLP solvers are linked with, e.g., OpenBLAS
// used by MUMPS in Ipopt; 0 leaves them unchanged. OpenMP limits only the calling thread.
static void setNumberOfThreadsInNLPLibraries(int threads)
{
    if(threads <= 0)
        return;

#ifdef _OPENMP
    omp_set_num_threads(threads);
#endif

#ifndef _WIN32
    // SHOT is not linked with these libraries directly, and which of them are loaded depends on how the NLP solvers
    // were built, so they are looked up at run time
    using SetNumberOfThreads = void (*)(int);

    static auto openBLASSetNumberOfThreads
        = reinterpret_cast<SetNumberOfThreads>(dlsym(RTLD_DEFAULT, "openblas_set_num_threads"));
    static auto MKLSetNumberOfThreads
        = reinterpret_cast<SetNumberOfThreads>(dlsym(RTLD_DEFAULT, "MKL_Set_Num_Threads"));
#ifndef _OPENMP
    static auto OpenMPSetNumberOfThreads
        = reinterpret_cast<SetNumberOfThreads>(dlsym(RTLD_DEFAULT, "omp_set_num_threads"));
#endif

    if(openBLASSetNumberOfThreads)
        openBLASSetNumberOfThreads(threads);

    if(MKLSetNumberOfThreads)
        MKLSetNumberOfThreads(threads);

#ifndef _OPENMP
    if(OpenMPSetNumberOfThreads)
        OpenMPSetNumberOfThreads(threads);
#endif
#endif

#ifdef __APPLE__
    std::string s = std::to_string(threads);
    setenv("VECLIB_MAXIMUM_THREADS", s.c_str(), 1);
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