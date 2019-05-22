/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "Enums.h"

#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// Fix for missing NAN i Visual Studio
#ifdef WIN32
#ifndef NAN
static const unsigned long __nan[2] = { 0xffffffff, 0x7fffffff };
#define NAN (*(const float*)__nan)
#endif
#else
#include <cmath> // For NAN
#endif

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

using ResultsPtr = std::shared_ptr<Results>;
using SettingsPtr = std::shared_ptr<Settings>;
using ModelPtr = std::shared_ptr<Model>;
using ProblemPtr = std::shared_ptr<Problem>;
using ReformulatedProblemPtr = std::shared_ptr<ReformulatedProblem>;
using ModelingSystemPtr = std::shared_ptr<IModelingSystem>;
using MIPSolverPtr = std::shared_ptr<IMIPSolver>;
using OutputPtr = std::shared_ptr<Output>;
using EventHandlerPtr = std::shared_ptr<EventHandler>;
using ReportPtr = std::shared_ptr<Report>;
using TaskHandlerPtr = std::shared_ptr<TaskHandler>;
using TimingPtr = std::shared_ptr<Timing>;
using DualSolverPtr = std::shared_ptr<DualSolver>;
using PrimalSolverPtr = std::shared_ptr<PrimalSolver>;
using IterationPtr = std::shared_ptr<Iteration>;

using ConstraintPtr = std::shared_ptr<Constraint>;
using NumericConstraintPtr = std::shared_ptr<NumericConstraint>;

using PairInteger = std::pair<int, int>;
using PairDouble = std::pair<double, double>;
using PairString = std::pair<std::string, std::string>;

using VectorDouble = std::vector<double>;
using VectorInteger = std::vector<int>;
using VectorString = std::vector<std::string>;

struct PairIndexValue
{
    int index;
    double value;

public:
    PairIndexValue() = default;
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
    bool isSourceConvex;
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

    int numberOfFoundPrimalSolutions = 0;

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

class VariableNotFoundException : public std::exception
{
private:
    std::string errorMessage;

public:
    VariableNotFoundException(std::string message) : errorMessage(message) {}

    inline const char* what() const throw() override
    {
        std::stringstream message;
        message << "Could not find variable ";
        message << errorMessage;

        return (message.str().c_str());
    }
};

class ConstraintNotFoundException : public std::exception
{
private:
    std::string errorMessage;

public:
    ConstraintNotFoundException(std::string message) : errorMessage(message) {}

    inline const char* what() const throw() override
    {
        std::stringstream message;
        message << "Could not find constraint ";
        message << errorMessage;

        return (message.str().c_str());
    }
};

class OperationNotImplementedException : public std::exception
{
private:
    std::string errorMessage;

public:
    OperationNotImplementedException(std::string message) : errorMessage(message) {}

    inline const char* what() const throw() override
    {
        std::stringstream message;
        message << "The following operation is not implemented: ";
        message << errorMessage;

        return (message.str().c_str());
    }
};
}; // namespace SHOT