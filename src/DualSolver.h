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

class DualSolver
{
public:
    DualSolver(EnvironmentPtr envPtr);

    ~DualSolver() { dualSolutionCandidates.clear(); }

    MIPSolverPtr MIPSolver;
    std::vector<DualSolution> dualSolutionCandidates;

    void addDualSolutionCandidate(DualSolution solution);
    void checkDualSolutionCandidates();

    void addGeneratedHyperplane(const Hyperplane& hyperplane);
    bool hasHyperplaneBeenAdded(size_t hash, int constraintIndex);

    std::vector<GeneratedHyperplane> generatedHyperplanes;

    double cutOffToUse;
    bool useCutOff = false;

private:
    EnvironmentPtr env;
};

} // namespace SHOT