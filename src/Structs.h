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
const double SHOT_DBL_MIN = std::numeric_limits<double>::lowest();
const double SHOT_DBL_MAX = std::numeric_limits<double>::max();
const double SHOT_DBL_INF = std::numeric_limits<double>::infinity();
const double SHOT_DBL_EPS = std::numeric_limits<double>::epsilon();
const int SHOT_INT_MAX = std::numeric_limits<int>::max();

class Results;
class Settings;
class Model;
class Problem;
class ReformulatedProblem;
class IModelingSystem;
class IMIPSolver;
class Output;
class Report;
class TaskHandler;
class EventHandler;
class Timing;
class Iteration;
class DualSolver;
class PrimalSolver;

class Constraint;
class NumericConstraint;

typedef std::shared_ptr<Results> ResultsPtr;
typedef std::shared_ptr<Settings> SettingsPtr;
typedef std::shared_ptr<Model> ModelPtr;
typedef std::shared_ptr<Problem> ProblemPtr;
typedef std::shared_ptr<ReformulatedProblem> ReformulatedProblemPtr;
typedef std::shared_ptr<IModelingSystem> ModelingSystemPtr;
typedef std::shared_ptr<IMIPSolver> MIPSolverPtr;
typedef std::shared_ptr<Output> OutputPtr;
typedef std::shared_ptr<EventHandler> EventHandlerPtr;
typedef std::shared_ptr<Report> ReportPtr;
typedef std::shared_ptr<TaskHandler> TaskHandlerPtr;
typedef std::shared_ptr<Timing> TimingPtr;
typedef std::shared_ptr<DualSolver> DualSolverPtr;
typedef std::shared_ptr<PrimalSolver> PrimalSolverPtr;
typedef std::shared_ptr<Iteration> IterationPtr;

typedef std::shared_ptr<Constraint> ConstraintPtr;
typedef std::shared_ptr<NumericConstraint> NumericConstraintPtr;

typedef std::pair<int, int> PairInteger;
typedef std::pair<double, double> PairDouble;
typedef std::pair<std::string, std::string> PairString;

typedef std::vector<double> VectorDouble;
typedef std::vector<int> VectorInteger;
typedef std::vector<std::string> VectorString;

struct PairIndexValue
{
    int index;
    double value;

public:
    PairIndexValue(){};
    PairIndexValue(int index, double value) : index(index), value(value){};
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
};

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
    PairIndexValue maxDevatingConstraintLinear;
    PairIndexValue maxDevatingConstraintQuadratic;
    PairIndexValue maxDevatingConstraintNonlinear;
    double maxIntegerToleranceError; // The maximum integer error before rounding
    bool boundProjectionPerformed = false; // Has the variable bounds been corrected to either upper or lower bounds?
    bool integerRoundingPerformed = false; // Has the integers been rounded?
    bool displayed = false; // Has the primal solution been displayed on console?
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
    NumericConstraintPtr sourceConstraint;
    int sourceConstraintIndex; // TODO remove
    VectorDouble generatedPoint;
    double objectiveFunctionValue; // Used for the objective cuts only
    E_HyperplaneSource source;
    bool isObjectiveHyperplane = false;
};

struct GeneratedHyperplane
{
    int sourceConstraintIndex;
    E_HyperplaneSource source;
    bool isLazy;
    bool isRemoved;
    int iterationGenerated;

    std::size_t pointHash;
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

    int numberOfIterationsWithDualStagnation = 0;
    int lastIterationWithSignificantDualUpdate = 0;
    int numberOfIterationsWithPrimalStagnation = 0;
    int lastIterationWithSignificantPrimalUpdate = 0;
    int numberOfIterationsWithoutNLPCallMIP = 0;

    int iterationLastPrimalBoundUpdate = 0;
    int iterationLastDualBoundUpdate = 0;
    int iterationLastLazyAdded = 0;

    double timeLastDualBoundUpdate = 0;
    double timeLastFixedNLPCall = 0;

    int numberOfOriginalInteriorPoints = 0;

    int numberOfExploredNodes = 0;
    int numberOfOpenNodes = 0;

    int numberOfPrimalReductionCutsUpdatesWithoutEffect = 0;
    int numberOfDualRepairsSinceLastPrimalUpdate = 0;

    int getNumberOfTotalDualProblems()
    {
        return (numberOfProblemsLP + numberOfProblemsQP + numberOfProblemsFeasibleMILP + numberOfProblemsOptimalMILP
            + numberOfProblemsFeasibleMIQP + numberOfProblemsOptimalMIQP + numberOfProblemsOptimalMIQCQP
            + numberOfProblemsFeasibleMIQCQP);
    };

    int getNumberOfTotalNLPProblems() { return (numberOfProblemsNLPInteriorPointSearch + numberOfProblemsFixedNLP); };
};

class Error
{
public:
    Error(std::string message) : message(message){};

    std::string message;
};
}; // namespace SHOT