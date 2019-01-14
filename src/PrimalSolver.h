/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Shared.h"

namespace SHOT
{

class PrimalSolver
{
public:
    PrimalSolver(EnvironmentPtr envPtr) { env = envPtr; }

    ~PrimalSolver()
    {
        primalSolutionCandidates.clear();
        fixedPrimalNLPCandidates.clear();
    }

    void addPrimalSolutionCandidate(VectorDouble pt, E_PrimalSolutionSource source, int iter);
    void addPrimalSolutionCandidates(std::vector<VectorDouble> pts, E_PrimalSolutionSource source, int iter);

    void addPrimalSolutionCandidate(SolutionPoint pt, E_PrimalSolutionSource source);
    void addPrimalSolutionCandidates(std::vector<SolutionPoint> pts, E_PrimalSolutionSource source);

    void checkPrimalSolutionCandidates();

    bool checkPrimalSolutionPoint(PrimalSolution primalSol);

    inline void addFixedNLPCandidate(
        VectorDouble pt, E_PrimalNLPSource source, double objVal, int iter, PairIndexValue maxConstrDev)
    {
        PrimalFixedNLPCandidate cand = { pt, source, objVal, iter };
        fixedPrimalNLPCandidates.push_back(cand);
    }

    std::vector<PrimalSolution> primalSolutionCandidates;
    std::vector<PrimalFixedNLPCandidate> fixedPrimalNLPCandidates;

private:
    EnvironmentPtr env;
};

} // namespace SHOT