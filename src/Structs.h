/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Enums.h"
#include <utility>
#include <vector>
#include <memory>
#include <list>
#include <string>

namespace SHOT
{
class ProcessInfo;
class Settings;
class Model;
class IMIPSolver;
class Output;
class Report;
class TaskHandler;
class OptProblemOriginal;

typedef std::shared_ptr<ProcessInfo> ProcessPtr;
typedef std::shared_ptr<Settings> SettingsPtr;
typedef std::shared_ptr<Model> ModelPtr;
typedef std::shared_ptr<IMIPSolver> MIPSolverPtr;
typedef std::shared_ptr<Output> OutputPtr;
typedef std::shared_ptr<OptProblemOriginal> OriginalProblemPtr;
typedef std::shared_ptr<Report> ReportPtr;
typedef std::shared_ptr<TaskHandler> TaskHandlerPtr;

typedef std::pair<int, int> PairInteger;
typedef std::pair<double, double> PairDouble;
typedef std::vector<double> VectorDouble;
typedef std::vector<int> VectorInteger;
typedef std::vector<std::string> VectorString;

struct PairIndexValue
{
    int index;
    double value;
};

struct PairCoordinateValue
{
    PairInteger indexes;
    double value;
};

struct SolutionPoint
{
    VectorDouble point;
    double objectiveValue;
    int iterFound;
    PairIndexValue maxDeviation;
    bool isRelaxedPoint = false;
}; // namespace structSolutionPoint

struct InteriorPoint
{
    VectorDouble point;
    ES_InteriorPointStrategy NLPSolver;
    PairIndexValue maxDevatingConstraint;
};

struct PrimalSolution
{
    VectorDouble point;
    E_PrimalSolutionSource sourceType;
    std::string sourceDescription;
    double objValue;
    int iterFound;
    PairIndexValue maxDevatingConstraintNonlinear;
    PairIndexValue maxDevatingConstraintLinear;
    double maxIntegerToleranceError;       // The maximum integer error before rounding
    bool boundProjectionPerformed = false; // Has the variable bounds been corrected to either upper or lower bounds?
    bool integerRoundingPerformed = false; // Has the integers been rounded?
    bool displayed = false;                // Has the primal solution been displayed on console?
};

struct PrimalFixedNLPCandidate
{
    VectorDouble point;
    E_PrimalNLPSource sourceType;
    double objValue;
    int iterFound;
    PairIndexValue maxDevatingConstraint;
};

struct DualSolution
{
    VectorDouble point;
    E_DualSolutionSource sourceType;
    double objValue;
    int iterFound;
    bool displayed; // Has the dual solution been displayed on console?
};

struct Hyperplane
{
    int sourceConstraintIndex;
    VectorDouble generatedPoint;
    E_HyperplaneSource source;
};

struct GeneratedHyperplane
{
    int generatedConstraintIndex;
    int sourceConstraintIndex;
    VectorDouble generatedPoint;
    E_HyperplaneSource source;
    bool isLazy;
    bool isRemoved;
    int generatedIter;
    int removedIter;
};

struct ModelStatistics
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
}; // namespace SHOT