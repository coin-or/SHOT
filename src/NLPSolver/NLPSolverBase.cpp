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

#ifdef __APPLE__
#include <cstdlib>
#include <mutex>
#include <string>
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

namespace SHOT
{

#ifdef __APPLE__
// Limits the threads of BLAS and LAPACK in the Accelerate framework. From macOS 15, BLASSetThreading switches the
// calling thread between one thread and as many as Accelerate decides; before, only VECLIB_MAXIMUM_THREADS is
// available, which is set once, since setting environment variables is not thread safe, and only if the user has not
// set it.
static void setNumberOfThreadsInAccelerate(int threads)
{
    // The values of the enum BLAS_THREADING in vecLib's thread_api.h
    constexpr unsigned int BLASThreadingMultiThreaded = 0;
    constexpr unsigned int BLASThreadingSingleThreaded = 1;

    using SetThreading = int (*)(unsigned int);

    static auto BLASSetThreading = reinterpret_cast<SetThreading>(dlsym(RTLD_DEFAULT, "BLASSetThreading"));

    if(BLASSetThreading)
    {
        BLASSetThreading(threads == 1 ? BLASThreadingSingleThreaded : BLASThreadingMultiThreaded);
        return;
    }

    static std::once_flag environmentSet;
    std::call_once(
        environmentSet, [threads]() { setenv("VECLIB_MAXIMUM_THREADS", std::to_string(threads).c_str(), 0); });
}
#endif

// Limits the number of threads of the BLAS and OpenMP libraries that the NLP solvers are linked with, e.g., OpenBLAS
// used by MUMPS in Ipopt or the Accelerate framework on macOS; 0 leaves them unchanged. OpenMP and, from macOS 15,
// Accelerate limit only the calling thread.
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
    setNumberOfThreadsInAccelerate(threads);
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