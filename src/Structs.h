/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
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
    ES_InteriorPointStrategy NLPSolver;
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

struct OptimizationProblemStatistics
{
    E_ProblemType problemType = E_ProblemType::None;
    E_ObjectiveFunctionType objectiveFunctionType = E_ObjectiveFunctionType::None;

    bool isMinimizationProblem = false;
    bool isDiscreteProblem = false;

    int numberOfConstraints = 0;
    int numberOfLinearConstraints = 0;
    int numberOfNonlinearConstraints = 0;
    int numberOfQuadraticConstraints = 0;

    bool quadraticTermsReformulatedAsNonlinear = false;
    int numberOfQuadraticTerms = 0;

    int numberOfVariables = 0;
    int numberOfContinousVariables = 0;
    int numberOfIntegerVariables = 0;
    int numberOfBinaryVariables = 0;
    int numberOfSemicontinuousVariables = 0;

    bool isObjectiveNonlinear()
    {
        return (objectiveFunctionType == E_ObjectiveFunctionType::Nonlinear || objectiveFunctionType == E_ObjectiveFunctionType::QuadraticConsideredAsNonlinear);
    };
};

struct SolutionStatistics
{
    int numberOfIterations = 0;
    int numberOfProblemsLP = 0;
    int numberOfProblemsQP = 0;
    int numberOfProblemsQCQP = 0;
    int numberOfProblemsFeasibleMILP = 0;
    int numberOfProblemsOptimalMILP = 0;
    int numberOfProblemsFeasibleMIQP = 0;
    int numberOfProblemsOptimalMIQP = 0;
    int numberOfProblemsFeasibleMIQCQP = 0;
    int numberOfProblemsOptimalMIQCQP = 0;

    int numberOfFunctionEvalutions = 0;
    int numberOfGradientEvaluations = 0;

    int numberOfProblemsMinimaxLP = 0;

    int numberOfProblemsNLPInteriorPointSearch = 0;
    int numberOfProblemsFixedNLP = 0;

    int numberOfConstraintsRemovedInPresolve = 0;
    int numberOfVariableBoundsTightenedInPresolve = 0;

    int numberOfIntegerCuts = 0;

    int numberOfIterationsWithStagnationMIP = 0;
    int numberOfIterationsWithSignificantObjectiveUpdate = 0;
    int numberOfIterationsWithoutNLPCallMIP = 0;

    int iterationLastPrimalBoundUpdate = 0;
    int iterationLastDualBoundUpdate = 0;
    int iterationLastLazyAdded = 0;

    double timeLastDualBoundUpdate = 0;
    double timeLastFixedNLPCall = 0;

    int numberOfOriginalInteriorPoints = 0;

    int numberOfExploredNodes = 0;
    int numberOfOpenNodes = 0;

    int getNumberOfTotalDualProblems()
    {
        return (numberOfProblemsLP +
                numberOfProblemsQP +
                numberOfProblemsFeasibleMILP +
                numberOfProblemsOptimalMILP +
                numberOfProblemsFeasibleMIQP +
                numberOfProblemsOptimalMIQP +
                numberOfProblemsOptimalMIQCQP +
                numberOfProblemsFeasibleMIQCQP);
    };

    int getNumberOfTotalNLPProblems()
    {
        return (numberOfProblemsNLPInteriorPointSearch +
                numberOfProblemsFixedNLP);
    };
};
