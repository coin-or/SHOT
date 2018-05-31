/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Enums.h"
#include "Structs.h"
#include "SHOTSettings.h"
#include "ProcessInfo.h"

class Iteration
{
  public:
    Iteration();
    ~Iteration();

    E_IterationProblemType type;
    E_ProblemSolutionStatus solutionStatus;

    std::vector<SolutionPoint> solutionPoints;

    double objectiveValue;
    std::pair<double, double> currentObjectiveBounds;

    std::vector<double> constraintDeviations;
    double maxDeviation;
    int maxDeviationConstraint;

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

    std::vector<std::vector<double>> hyperplanePoints;

    SolutionPoint getSolutionPointWithSmallestDeviation();
    int getSolutionPointWithSmallestDeviationIndex();
};
