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

    // Creates a candidate, with the values of the auxiliary variables in the reformulated problem and the hashes used
    // to identify it, without adding it to the candidates
    PrimalFixedNLPCandidate createFixedNLPCandidate(
        VectorDouble pt, E_PrimalNLPSource source, double objVal, int iter, PairIndexValue maxConstrDev);

    bool hasFixedNLPCandidateBeenTested(const PairDouble& hashes);

    std::vector<PrimalSolution> primalSolutionCandidates;
    std::vector<PrimalFixedNLPCandidate> fixedPrimalNLPCandidates;
    std::vector<PrimalFixedNLPCandidate> usedPrimalNLPCandidates;

    // The points given as external solutions, e.g., the starting point of the model. They are also starting points
    // for the NLP solver, which a local solver may improve, and are added to its candidates once the reformulated
    // problem exists.
    std::vector<VectorDouble> startingPointsForNLP;

    // The hashes of the combinations of the discrete variables whose NLP problems have been solved in the exhaustive
    // search, whatever the result. These are not solved again if the search is run as a fallback
    std::vector<PairDouble> enumeratedPrimalNLPCandidateHashes;

private:
    EnvironmentPtr env;
};

} // namespace SHOT