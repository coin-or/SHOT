/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "NLPSolverBase.h"

#include "../Model/Problem.h"

#include <map>
#include <string>
#include <utility>

namespace SHOT
{

/**
   An NLP solver using Uno (https://github.com/cvanaret/Uno) through its C API.

   Uno bundles several NLP algorithms (interior point and SQP) behind a common option set, so which method is
   actually run is decided by the Subsolver.Uno.Preset setting rather than by the class.

   No Uno header is included here on purpose: the two handles below are kept opaque so that every Uno call stays
   inside NLPSolverUno.cpp. Should Uno gain a supported C++ interface later, only that file needs to change.
*/
class NLPSolverUno : public NLPSolverBase
{
private:
    ProblemPtr sourceProblem;

    void* unoModel = nullptr;
    void* unoSolver = nullptr;

    VectorDouble lowerBounds;
    VectorDouble upperBounds;

    VectorDouble lowerBoundsBeforeFix;
    VectorDouble upperBoundsBeforeFix;

    VectorInteger fixedVariableIndexes;
    VectorDouble fixedVariableValues;

    VectorInteger startingPointVariableIndexes;
    VectorDouble startingPointVariableValues;

    VectorDouble variableSolution;
    double objectiveValue = 0.0;

    /* Maps a (constraint index, variable index) pair onto its position in the Jacobian value array, and a
       (variable index, variable index) pair onto its position in the Lagrangian Hessian value array. Unlike Ipopt,
       Uno is given the sparsity pattern once when the model is created rather than through a structure pass of the
       evaluation callback, so these are built in the constructor and then reused by every evaluation. */
    std::map<std::pair<int, int>, int> jacobianCounterPlacement;
    std::map<std::pair<int, int>, int> lagrangianHessianCounterPlacement;

    int numberOfJacobianNonzeros = 0;
    int numberOfHessianNonzeros = 0;

    std::string solverDescription;

    // Holds the part of Uno's output that has not yet been terminated by a newline
    std::string loggerBuffer;

    double divergingIterativesTolerance = 1e20;

    void createModel();
    void setInitialSettings();
    void pushBoundsToModel();
    void pushStartingPointToModel();

    /* Uno interprets a bound as unbounded only when it is an actual infinity, whereas SHOT uses SHOT_DBL_MIN and
       SHOT_DBL_MAX. Passing those through unchanged would make every variable look bounded, which in particular
       makes the barrier methods place slack terms on bounds of magnitude 1e308. */
    static double toUnoBound(double bound);

public:
    NLPSolverUno(EnvironmentPtr envPtr, ProblemPtr source);

    ~NLPSolverUno() override;

    void setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues) override;
    void clearStartingPoint() override;

    void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues) override;

    void unfixVariables() override;

    void saveOptionsToFile(std::string fileName) override;

    void saveProblemToFile(std::string fileName) override;

    VectorDouble getSolution() override;
    double getSolution(int i) override;

    double getObjectiveValue() override;

    void updateVariableLowerBound(int variableIndex, double bound) override;
    void updateVariableUpperBound(int variableIndex, double bound) override;

    std::string getSolverDescription() override;

    /* The following are called from the Uno callbacks in NLPSolverUno.cpp. They are public only so that the
       file-static trampolines that Uno is given can reach them; they are not part of the INLPSolver interface. */
    bool evaluateObjective(int numberOfVariables, const double* x, double* value);
    bool evaluateObjectiveGradient(int numberOfVariables, const double* x, double* gradient);
    bool evaluateConstraints(int numberOfVariables, int numberOfConstraints, const double* x, double* values);
    bool evaluateJacobian(int numberOfVariables, int numberOfNonzeros, const double* x, double* values);
    bool evaluateLagrangianHessian(int numberOfVariables, int numberOfConstraints, int numberOfNonzeros,
        const double* x, double objectiveMultiplier, const double* multipliers, double* values);
    void writeLoggerOutput(const char* buffer, int length);
    void flushLoggerOutput();

protected:
    E_NLPSolutionStatus solveProblemInstance() override;

    VectorDouble getVariableLowerBounds() override;
    VectorDouble getVariableUpperBounds() override;
};
} // namespace SHOT
