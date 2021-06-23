
/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "../Structs.h"
#include "../Environment.h"

#include "Variables.h"
#include "AuxiliaryVariables.h"
#include "ObjectiveFunction.h"
#include "Constraints.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "cppad/cppad.hpp"

namespace SHOT
{

struct ProblemProperties
{
    bool isValid = false; // Whether the values here are valid anymore

    E_ProblemConvexity convexity = E_ProblemConvexity::NotSet;

    bool isNonlinear = false;
    bool isDiscrete = false;

    bool isMINLPProblem = false;
    bool isNLPProblem = false;
    bool isMIQPProblem = false;
    bool isQPProblem = false;
    bool isMIQCQPProblem = false;
    bool isQCQPProblem = false;
    bool isMILPProblem = false;
    bool isLPProblem = false;

    int numberOfVariables = 0;
    int numberOfRealVariables = 0;
    int numberOfDiscreteVariables = 0; // Binary and integer variables
    int numberOfBinaryVariables = 0;
    int numberOfIntegerVariables = 0; // Not including binary or semiinteger variables
    int numberOfSemicontinuousVariables = 0;
    int numberOfSemiintegerVariables = 0;
    int numberOfNonlinearVariables = 0;
    int numberOfVariablesInNonlinearExpressions = 0;
    int numberOfAuxiliaryVariables = 0;

    int numberOfNumericConstraints = 0; // This include the linearizations in the POA step
    int numberOfLinearConstraints = 0; // This include the linearizations in the POA step
    int numberOfQuadraticConstraints = 0;
    int numberOfConvexQuadraticConstraints = 0;
    int numberOfNonconvexQuadraticConstraints = 0;
    int numberOfNonlinearConstraints = 0;
    int numberOfConvexNonlinearConstraints = 0;
    int numberOfNonconvexNonlinearConstraints = 0;
    int numberOfNonlinearExpressions = 0; // This includes a possible nonlinear objective

    int numberOfSpecialOrderedSets = 0;

    int numberOfAddedLinearizations = 0; // In the initial POA step

    std::string name = "";
    std::string description = "";
    bool isReformulated = false; // True if this is the reformulated problem
};

struct SpecialOrderedSet
{
    E_SOSType type = E_SOSType::One;
    Variables variables;

    VectorDouble weights;

    SpecialOrderedSet() = default;

    SpecialOrderedSet(E_SOSType SOSType, Variables vars, VectorDouble w = {})
    {
        type = SOSType;
        variables = vars;
        weights = w;
    };

    void takeOwnership([[maybe_unused]] ProblemPtr owner) {};
};

using SpecialOrderedSetPtr = std::shared_ptr<SpecialOrderedSet>;
using SpecialOrderedSets = std::vector<SpecialOrderedSetPtr>;

class DllExport Problem : public std::enable_shared_from_this<Problem>
{
private:
    bool variablesUpdated = false;
    bool constraintsUpdated = false;
    bool objectiveUpdated = false;

    std::shared_ptr<std::vector<std::pair<NumericConstraintPtr, Variables>>> constraintGradientSparsityPattern;
    std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> constraintsHessianSparsityPattern;
    std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> lagrangianHessianSparsityPattern;

    NonlinearConstraints constraintsWithNonlinearExpressions;

    void updateVariableBounds(); // This is called by updateVariables()
    void updateVariables();
    void updateConstraints();
    void updateConvexity();
    void updateFactorableFunctions();

    bool verifyOwnership();

public:
    EnvironmentPtr env;

    Problem(EnvironmentPtr env);

    virtual ~Problem();

    ProblemProperties properties;
    std::string name = "";

    Variables allVariables;
    Variables realVariables;
    Variables binaryVariables;
    Variables integerVariables;
    Variables semicontinuousVariables;
    Variables semiintegerVariables;
    Variables nonlinearVariables; // All nonlinear variables, including in quadratic, signomial or monomial terms
    Variables nonlinearExpressionVariables; // Variables in general nonlinear expressions

    AuxiliaryVariables auxiliaryVariables;
    AuxiliaryVariablePtr auxiliaryObjectiveVariable; // This is not the same as one created in the dual problem

    VariablePtr antiEpigraphObjectiveVariable; // This is the objective variable used before anti epigraph formulation

    VectorDouble variableLowerBounds;
    VectorDouble variableUpperBounds;
    IntervalVector variableBounds;

    ObjectiveFunctionPtr objectiveFunction;

    NumericConstraints numericConstraints;
    LinearConstraints linearConstraints;
    QuadraticConstraints quadraticConstraints;
    NonlinearConstraints nonlinearConstraints;

    SpecialOrderedSets specialOrderedSets;

    std::vector<CppAD::AD<double>> factorableFunctionVariables;
    std::vector<CppAD::AD<double>> factorableFunctions;
    CppAD::ADFun<double> ADFunctions;

    void updateProperties();

    // This also updates the problem properties
    void finalize();

    void add(VariablePtr variable);
    void add(Variables variables);

    void add(AuxiliaryVariablePtr variable);
    void add(AuxiliaryVariables variables);

    void add(LinearConstraintPtr constraint);
    void add(QuadraticConstraintPtr constraint);
    void add(NonlinearConstraintPtr constraint);
    void add(NumericConstraintPtr constraint);

    void add(ObjectiveFunctionPtr objective);
    void add(LinearObjectiveFunctionPtr objective);
    void add(QuadraticObjectiveFunctionPtr objective);
    void add(NonlinearObjectiveFunctionPtr objective);

    void add(SpecialOrderedSetPtr orderedSet);

    template <class T> void add(std::vector<T> elements);

    VariablePtr getVariable(int variableIndex);
    ConstraintPtr getConstraint(int constraintIndex);

    double getVariableLowerBound(int variableIndex);
    double getVariableUpperBound(int variableIndex);

    VectorDouble getVariableLowerBounds();
    VectorDouble getVariableUpperBounds();

    IntervalVector getVariableBounds();

    AuxiliaryVariables getAuxiliaryVariablesOfType(E_AuxiliaryVariableType type);

    void setVariableLowerBound(int variableIndex, double bound);
    void setVariableUpperBound(int variableIndex, double bound);
    void setVariableBounds(int variableIndex, double lowerBound, double upperBound);

    std::shared_ptr<std::vector<std::pair<NumericConstraintPtr, Variables>>> getConstraintsJacobianSparsityPattern();
    std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> getConstraintsHessianSparsityPattern();
    std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> getLagrangianHessianSparsityPattern();

    std::optional<NumericConstraintValue> getMostDeviatingNumericConstraint(const VectorDouble& point);
    std::optional<NumericConstraintValue> getMostDeviatingNonlinearOrQuadraticConstraint(const VectorDouble& point);
    std::optional<NumericConstraintValue> getMostDeviatingNonlinearConstraint(const VectorDouble& point);

    template <typename T>
    std::optional<NumericConstraintValue> getMostDeviatingNumericConstraint(
        const VectorDouble& point, std::vector<T> constraintSelection);

    template <typename T>
    std::optional<NumericConstraintValue> getMostDeviatingNumericConstraint(const VectorDouble& point,
        std::vector<std::shared_ptr<T>> constraintSelection, std::vector<T*>& activeConstraints);

    template <typename T>
    std::optional<NumericConstraintValue> getMostDeviatingNumericConstraint(const VectorDouble& point,
        std::vector<std::shared_ptr<T>> constraintSelection, std::vector<std::shared_ptr<T>>& activeConstraints);

    NumericConstraintValue getMaxNumericConstraintValue(
        const VectorDouble& point, const LinearConstraints constraintSelection);
    NumericConstraintValue getMaxNumericConstraintValue(
        const VectorDouble& point, const QuadraticConstraints constraintSelection);
    NumericConstraintValue getMaxNumericConstraintValue(
        const VectorDouble& point, const NonlinearConstraints constraintSelection, double correction = 0.0);
    NumericConstraintValue getMaxNumericConstraintValue(
        const VectorDouble& point, const NumericConstraints constraintSelection);

    template <typename T>
    NumericConstraintValue getMaxNumericConstraintValue(
        const VectorDouble& point, const std::vector<T*>& constraintSelection, std::vector<T*>& activeConstraints);

    NumericConstraintValue getMaxNumericConstraintValue(const VectorDouble& point,
        const std::vector<NumericConstraint*>& constraintSelection, std::vector<NumericConstraint*>& activeConstraints);

    template <typename T>
    NumericConstraintValues getAllDeviatingConstraints(
        const VectorDouble& point, double tolerance, std::vector<T> constraintSelection, double correction = 0.0);

    NumericConstraintValues getFractionOfDeviatingNonlinearConstraints(
        const VectorDouble& point, double tolerance, double fraction, double correction = 0.0);

    virtual NumericConstraintValues getAllDeviatingNumericConstraints(const VectorDouble& point, double tolerance);

    virtual NumericConstraintValues getAllDeviatingLinearConstraints(const VectorDouble& point, double tolerance);
    virtual NumericConstraintValues getAllDeviatingQuadraticConstraints(const VectorDouble& point, double tolerance);

    virtual NumericConstraintValues getAllDeviatingNonlinearConstraints(const VectorDouble& point, double tolerance);

    virtual bool areLinearConstraintsFulfilled(VectorDouble point, double tolerance);

    virtual bool areQuadraticConstraintsFulfilled(VectorDouble point, double tolerance);

    virtual bool areNonlinearConstraintsFulfilled(VectorDouble point, double tolerance);

    virtual bool areNumericConstraintsFulfilled(VectorDouble point, double tolerance);

    virtual bool areIntegralityConstraintsFulfilled(VectorDouble point, double tolerance);

    virtual bool areSpecialOrderedSetsFulfilled(VectorDouble point, double tolerance);

    bool areVariableBoundsFulfilled(VectorDouble point, double tolerance);

    void saveProblemToFile(std::string filename);

    void doFBBT();
    bool doFBBTOnConstraint(NumericConstraintPtr constraint, double timeLimit);

    void augmentAuxiliaryVariableValues(VectorDouble& point);

    friend std::ostream& operator<<(std::ostream& stream, const Problem& problem);

    ProblemPtr createCopy(EnvironmentPtr destinationEnv, bool integerRelaxed = false, bool convexityRelaxed = false,
        bool copyAuxiliary = false);
};

inline std::ostream& operator<<(std::ostream& stream, ProblemPtr problem)
{
    stream << *problem;
    return stream;
}

inline std::ostream& operator<<(std::ostream& stream, FactorableFunctionPtr function)
{
    stream << *function;
    return stream;
}

} // namespace SHOT