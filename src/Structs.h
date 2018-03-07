/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include <vector>
#include "Enums.h"
#include "OSGeneral.h"

struct SolutionPoint
{
    std::vector<double> point;
    double objectiveValue;
    int iterFound;
    IndexValuePair maxDeviation;
};

struct InteriorPoint
{
    std::vector<double> point;
    ES_NLPSolver NLPSolver;
    IndexValuePair maxDevatingConstraint;
};

struct PrimalSolution
{
    std::vector<double> point;
    E_PrimalSolutionSource sourceType;
    std::string sourceDescription;
    double objValue;
    int iterFound;
    IndexValuePair maxDevatingConstraintNonlinear;
    IndexValuePair maxDevatingConstraintLinear;
    double maxIntegerToleranceError;       // The maximum integer error before rounding
    bool boundProjectionPerformed = false; // Has the variable bounds been corrected to either upper or lower bounds?
    bool integerRoundingPerformed = false; // Has the integers been rounded?
    bool displayed = false;                // Has the primal solution been displayed on console?
};

struct PrimalFixedNLPCandidate
{
    std::vector<double> point;
    E_PrimalNLPSource sourceType;
    double objValue;
    int iterFound;
    IndexValuePair maxDevatingConstraint;
};

struct DualSolution
{
    std::vector<double> point;
    E_DualSolutionSource sourceType;
    double objValue;
    int iterFound;
    bool displayed; // Has the dual solution been displayed on console?
};

struct Hyperplane
{
    int sourceConstraintIndex;
    std::vector<double> generatedPoint;
    E_HyperplaneSource source;
};

struct GeneratedHyperplane
{
    int generatedConstraintIndex;
    int sourceConstraintIndex;
    std::vector<double> generatedPoint;
    E_HyperplaneSource source;
    bool isLazy;
    bool isRemoved;
    int generatedIter;
    int removedIter;
};