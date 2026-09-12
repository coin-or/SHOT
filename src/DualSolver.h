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

#include <map>
#include <set>

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

    void addHyperplane(HyperplanePtr hyperplane);
    void addGeneratedHyperplane(const HyperplanePtr hyperplane);
    bool hasHyperplaneBeenAdded(double hash, int constraintIndex);

    void addIntegerCut(IntegerCut integerCut);
    void addGeneratedIntegerCut(IntegerCut integerCut);
    bool hasIntegerCutBeenAdded(double hash);

    std::vector<GeneratedHyperplanePtr> generatedHyperplanes;
    std::vector<HyperplanePtr> hyperplaneWaitingList;

    std::vector<IntegerCut> generatedIntegerCuts;
    std::vector<IntegerCut> integerCutWaitingList;

    std::vector<std::shared_ptr<InteriorPoint>> interiorPointCandidates;
    std::vector<std::shared_ptr<InteriorPoint>> interiorPts;

    double cutOffToUse = SHOT_DBL_INF;
    bool useCutOff = false;
    bool isSingleTree = false;

private:
    EnvironmentPtr env;

    // The hashes of the generated hyperplanes for each constraint index, where -1 is used for the objective function
    std::map<int, std::set<double>> generatedHyperplaneHashes;

    double calculateHyperplaneHash(NumericHyperplanePtr hyperplane);
};

} // namespace SHOT