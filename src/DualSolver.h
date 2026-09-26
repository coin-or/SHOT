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
#include <optional>
#include <utility>

namespace SHOT
{
class Variable;

class DualSolver
{
public:
    DualSolver(EnvironmentPtr envPtr);

    ~DualSolver() { dualSolutionCandidates.clear(); }

    MIPSolverPtr MIPSolver;
    std::vector<DualSolution> dualSolutionCandidates;

    void addDualSolutionCandidate(DualSolution solution);
    void checkDualSolutionCandidates();

    // Reports a dual bound that is not valid for the problem, the first time at warning level
    void warnAboutInvalidDualBound(double dualBound, double primalBound, double tolerance);

    void addHyperplane(HyperplanePtr hyperplane);
    void addGeneratedHyperplane(const HyperplanePtr hyperplane);
    bool hasHyperplaneBeenAdded(const VectorDouble& generatedPoint, int constraintIndex);

    // Where to generate the hyperplane for the constraint, which is the given point unless the constraint is not
    // finite there: x^2/s is infinite where s is zero, although it is convex, and neither its value nor its
    // gradient can be used. The point is then moved toward one where the constraint is finite, which a convex
    // constraint allows, since its linearization is valid wherever it is defined. Returns nothing when there is
    // no such point.
    std::optional<VectorDouble> getHyperplaneGenerationPoint(
        const VectorDouble& point, const NumericConstraintPtr& constraint);

    void addIntegerCut(IntegerCut integerCut);
    void addGeneratedIntegerCut(IntegerCut integerCut);
    bool hasIntegerCutBeenAdded(const PairDouble& hashes);

    std::vector<GeneratedHyperplanePtr> generatedHyperplanes;
    std::vector<HyperplanePtr> hyperplaneWaitingList;

    std::vector<IntegerCut> generatedIntegerCuts;
    std::vector<IntegerCut> integerCutWaitingList;

    std::vector<std::shared_ptr<InteriorPoint>> interiorPointCandidates;
    std::vector<std::shared_ptr<InteriorPoint>> interiorPts;

    double cutOffToUse = SHOT_DBL_INF;
    bool useCutOff = false;
    bool isSingleTree = false;

    // Whether the dual problem contains the whole reformulated problem instead of only being a relaxation of it. What
    // is added to it while solving, e.g. the cutoff constraint and integer cuts, only restricts it, so the problem is
    // unbounded if the dual problem is.
    bool isDualProblemExact();

    // Removes the bounds that have replaced missing bounds of the variables when the problem was read, in the problem,
    // the reformulated problem and the MIP solver. The dual bounds found so far may be too strong since they are
    // bounds for the problem with the artificial bounds, so they are reset.
    void removeArtificialBounds(const std::vector<std::shared_ptr<Variable>>& variables);

    // The variables found at artificial bounds in a callback, where the bounds cannot be changed. The MIP solver is
    // then interrupted, and the bounds are removed before solving again.
    std::vector<std::shared_ptr<Variable>> variablesAtArtificialBounds;

private:
    EnvironmentPtr env;

    // The hashes of the generated hyperplanes for each constraint index, where -1 is used for the objective
    // function. Two hashes are kept for each point, so that two different points are only taken for the same one
    // when both of them match.
    std::map<int, std::multimap<double, double>> generatedHyperplaneHashes;

    // Counts hyperplanes generated again for a point they have already been generated in, to detect that the dual
    // problem is not making progress
    int numberOfRepeatedHyperplanes = 0;
    bool repeatedHyperplaneWarningShown = false;

    // A dual bound that passes the primal bound by more than the tolerance is not valid for the problem, which is
    // worth a warning, but only the first time since the cause is the same for the following ones
    bool invalidDualBoundWarningShown = false;

    std::pair<double, double> calculateHashes(const VectorDouble& point);
    std::pair<double, double> calculateHyperplaneHashes(NumericHyperplanePtr hyperplane);

    bool hasHyperplaneBeenAdded(const std::pair<double, double>& hashes, int constraintIndex);

    // The largest magnitude of the hyperplane in the point, and its value in the point to cut off, which is
    // positive when that point is cut off. The terms are the ones the hyperplane is built from, so that a point
    // accepted here is not rejected by MIPSolverBase::createHyperplane.
    std::optional<std::pair<double, double>> evaluateHyperplaneTerms(
        const VectorDouble& generationPoint, const VectorDouble& pointToCutOff, const NumericConstraintPtr& constraint);

    // The points to move toward, the ones deepest inside the constraints first
    std::vector<VectorDouble> getFinitePointCandidates();

    // Whether the hyperplane is in the list of generated ones, regardless of the solution strategy used
    bool isHyperplaneInGeneratedList(const std::pair<double, double>& hashes, int constraintIndex);
};

} // namespace SHOT