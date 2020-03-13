/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Environment.h"
#include "Enums.h"
#include "Structs.h"

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

    void addFixedNLPCandidate(
        VectorDouble pt, E_PrimalNLPSource source, double objVal, int iter, PairIndexValue maxConstrDev);

    bool hasFixedNLPCandidateBeenTested(double hash);

    std::vector<PrimalSolution> primalSolutionCandidates;
    std::vector<PrimalFixedNLPCandidate> fixedPrimalNLPCandidates;
    std::vector<PrimalFixedNLPCandidate> usedPrimalNLPCandidates;

private:
    EnvironmentPtr env;
};

} // namespace SHOT