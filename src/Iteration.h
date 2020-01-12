/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Structs.h"
#include "Enums.h"
#include "Environment.h"

namespace SHOT
{
class Iteration
{
public:
    Iteration(EnvironmentPtr envPtr);
    ~Iteration();

    E_DualProblemClass dualProblemClass;
    bool isDualProblemDiscrete = false;

    E_ProblemSolutionStatus solutionStatus;

    std::vector<SolutionPoint> solutionPoints;

    double objectiveValue;
    PairDouble currentObjectiveBounds;

    VectorDouble constraintDeviations;
    double maxDeviation = 0.0;
    int maxDeviationConstraint = -1;

    double usedConstraintTolerance;

    int usedMIPSolutionLimit;
    bool MIPSolutionLimitUpdated;

    int iterationNumber;

    int numHyperplanesAdded = 0;
    int totNumHyperplanes;
    int relaxedLazyHyperplanesAdded = 0;

    int numberOfExploredNodes = 0;
    int numberOfOpenNodes = 0;

    double boundaryDistance;

    bool isMIP();
    bool isSolved = false;

    double solutionTime;
    bool hasInfeasibilityRepairBeenPerformed = false;
    bool wasInfeasibilityRepairSuccessful = false;
    int numberOfInfeasibilityRepairedConstraints = 0;

    bool forceObjectiveReductionCut = false;

    std::vector<VectorDouble> hyperplanePoints;

    SolutionPoint getSolutionPointWithSmallestDeviation();
    int getSolutionPointWithSmallestDeviationIndex();

private:
    EnvironmentPtr env;
};
} // namespace SHOT