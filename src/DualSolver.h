/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Environment.h"
#include "Structs.h"

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
    bool hasHyperplaneBeenAdded(double hash, int constraintIndex);

    std::vector<GeneratedHyperplane> generatedHyperplanes;
    std::vector<std::pair<VectorInteger, VectorInteger>> generatedIntegerCuts;

    // First is binaries = 1, second is binaries = 0
    std::vector<std::pair<VectorInteger, VectorInteger>> integerCutWaitingList;
    std::vector<Hyperplane> hyperplaneWaitingList;

    std::vector<std::shared_ptr<InteriorPoint>> interiorPts;

    double cutOffToUse;
    bool useCutOff = false;
    bool isSingleTree = false;

private:
    EnvironmentPtr env;
};

} // namespace SHOT