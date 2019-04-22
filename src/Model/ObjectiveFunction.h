/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Structs.h"
#include "../Enums.h"
#include "Variables.h"
#include "Terms.h"
#include "NonlinearExpressions.h"

#include <vector>

namespace SHOT
{

typedef mc::Interval Interval;
typedef std::vector<Interval> IntervalVector;

enum class E_ObjectiveFunctionDirection
{
    Maximize,
    Minimize,
    None
};

enum class E_ObjectiveFunctionClassification
{
    None,
    Linear,
    Quadratic,
    QuadraticConsideredAsNonlinear,
    Signomial,
    Nonlinear,
    GeneralizedSignomial,
    Nonalgebraic
};

struct ObjectiveFunctionProperties
{
    bool isValid = false; // Whether the values here are valid anymore

    bool isMinimize = false;
    bool isMaximize = false;

    E_Convexity convexity = E_Convexity::Convex;

    bool isReformulated = false;

    E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::None;

    bool hasLinearTerms = false;
    bool hasQuadraticTerms = false;
    bool hasBinaryBilinearTerms = false;
    bool hasNonBinaryBilinearTerms = false;
    bool hasBinarySquareTerms = false;
    bool hasNonBinarySquareTerms = false;
    bool hasMonomialTerms = false;
    bool hasSignomialTerms = false;
    bool hasNonlinearExpression = false;
    bool hasNonalgebraicPart = false; // E.g for external functions
};

class ObjectiveFunction
{

public:
    ~ObjectiveFunction() {}

    ObjectiveFunctionProperties properties;

    E_ObjectiveFunctionDirection direction;
    double constant = 0.0;

    std::weak_ptr<Problem> ownerProblem;

    std::shared_ptr<Variables> gradientSparsityPattern;
    std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> hessianSparsityPattern;

    void takeOwnership(ProblemPtr owner);

    virtual void updateProperties();

    virtual double calculateValue(const VectorDouble& point) = 0;
    virtual Interval calculateValue(const IntervalVector& intervalVector) = 0;

    virtual SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) = 0;
    virtual std::shared_ptr<Variables> getGradientSparsityPattern();

    virtual SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) = 0;
    virtual std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> getHessianSparsityPattern();

    virtual std::ostream& print(std::ostream&) const = 0;

protected:
    virtual void initializeGradientSparsityPattern() = 0;
    virtual void initializeHessianSparsityPattern() = 0;
};

typedef std::shared_ptr<ObjectiveFunction> ObjectiveFunctionPtr;

std::ostream& operator<<(std::ostream& stream, ObjectiveFunctionPtr objective);

class LinearObjectiveFunction : public ObjectiveFunction
{
public:
    LinearObjectiveFunction() { linearTerms = LinearTerms(); }

    LinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection)
    {
        linearTerms = LinearTerms();

        direction = objectiveDirection;
        updateProperties();
    }

    LinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection, LinearTerms linTerms, double constValue)
    {
        linearTerms = LinearTerms();

        direction = objectiveDirection;
        linearTerms = linTerms;
        constant = constValue;

        updateProperties();
    }

    virtual ~LinearObjectiveFunction(){};

    LinearTerms linearTerms;

    void add(LinearTerms terms);

    void add(LinearTermPtr term);
    virtual void updateProperties() override;

    virtual double calculateValue(const VectorDouble& point) override;
    virtual Interval calculateValue(const IntervalVector& intervalVector) override;

    virtual SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) override;

    virtual SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    virtual void initializeGradientSparsityPattern();
    virtual void initializeHessianSparsityPattern();
};

typedef std::shared_ptr<LinearObjectiveFunction> LinearObjectiveFunctionPtr;

class QuadraticObjectiveFunction : public LinearObjectiveFunction
{
public:
    QuadraticObjectiveFunction()
    {
        linearTerms = LinearTerms();
        quadraticTerms = QuadraticTerms();
    }

    QuadraticObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection)
    {
        linearTerms = LinearTerms();
        quadraticTerms = QuadraticTerms();

        direction = objectiveDirection;
        updateProperties();
    }

    QuadraticObjectiveFunction(
        E_ObjectiveFunctionDirection objectiveDirection, QuadraticTerms quadTerms, double constValue)
    {
        linearTerms = LinearTerms();

        direction = objectiveDirection;
        quadraticTerms = quadTerms;
        constant = constValue;
        updateProperties();
    }

    QuadraticObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection, LinearTerms linTerms,
        QuadraticTerms quadTerms, double constValue)
    {
        direction = objectiveDirection;
        linearTerms = linTerms;
        quadraticTerms = quadTerms;
        constant = constValue;
        updateProperties();
    }

    virtual ~QuadraticObjectiveFunction(){};

    QuadraticTerms quadraticTerms;

    void add(LinearTerms terms) { LinearObjectiveFunction::add(terms); }

    void add(LinearTermPtr term) { LinearObjectiveFunction::add(term); }

    void add(QuadraticTerms terms);
    void add(QuadraticTermPtr term);

    virtual void updateProperties() override;

    virtual double calculateValue(const VectorDouble& point) override;
    virtual Interval calculateValue(const IntervalVector& intervalVector) override;

    virtual SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) override;
    virtual SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    virtual void initializeGradientSparsityPattern();
    virtual void initializeHessianSparsityPattern();
};

typedef std::shared_ptr<QuadraticObjectiveFunction> QuadraticObjectiveFunctionPtr;

class NonlinearObjectiveFunction : public QuadraticObjectiveFunction
{
public:
    NonlinearObjectiveFunction()
    {
        this->linearTerms = LinearTerms();
        this->quadraticTerms = QuadraticTerms();
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection)
    {
        linearTerms = LinearTerms();
        quadraticTerms = QuadraticTerms();

        direction = objectiveDirection;
        updateProperties();
    }

    NonlinearObjectiveFunction(
        E_ObjectiveFunctionDirection objectiveDirection, NonlinearExpressionPtr nonlinExpr, double constValue)
    {
        linearTerms = LinearTerms();
        quadraticTerms = QuadraticTerms();

        direction = objectiveDirection;
        nonlinearExpression = nonlinExpr;
        constant = constValue;
        updateProperties();
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection, LinearTerms linTerms,
        NonlinearExpressionPtr nonlinExpr, double constValue)
    {
        quadraticTerms = QuadraticTerms();

        direction = objectiveDirection;
        linearTerms = linTerms;
        nonlinearExpression = nonlinExpr;
        constant = constValue;
        updateProperties();
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection, LinearTerms linTerms,
        QuadraticTerms quadTerms, NonlinearExpressionPtr nonlinExpr, double constValue)
    {
        direction = objectiveDirection;
        linearTerms = linTerms;
        quadraticTerms = quadTerms;
        nonlinearExpression = nonlinExpr;
        constant = constValue;
        updateProperties();
    }

    virtual ~NonlinearObjectiveFunction(){};

    MonomialTerms monomialTerms;
    SignomialTerms signomialTerms;

    NonlinearExpressionPtr nonlinearExpression;
    FactorableFunctionPtr factorableFunction;
    std::vector<std::pair<VariablePtr, FactorableFunction>> symbolicSparseJacobian;
    std::vector<std::pair<std::pair<VariablePtr, VariablePtr>, FactorableFunction>> symbolicSparseHessian;

    Variables variablesInMonomialTerms;
    Variables variablesInSignomialTerms;
    Variables variablesInNonlinearExpression;

    int factorableFunctionIndex;

    void add(LinearTerms terms) { LinearObjectiveFunction::add(terms); }

    void add(LinearTermPtr term) { LinearObjectiveFunction::add(term); }

    void add(QuadraticTerms terms) { QuadraticObjectiveFunction::add(terms); }

    void add(QuadraticTermPtr term) { QuadraticObjectiveFunction::add(term); }

    void add(MonomialTerms terms);
    void add(MonomialTermPtr term);

    void add(SignomialTerms terms);
    void add(SignomialTermPtr term);

    void add(NonlinearExpressionPtr expression);

    void updateFactorableFunction();

    virtual void updateProperties() override;

    virtual double calculateValue(const VectorDouble& point) override;
    virtual Interval calculateValue(const IntervalVector& intervalVector) override;

    virtual SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) override;
    virtual SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    virtual void initializeGradientSparsityPattern();
    virtual void initializeHessianSparsityPattern();
};

typedef std::shared_ptr<NonlinearObjectiveFunction> NonlinearObjectiveFunctionPtr;

std::ostream& operator<<(std::ostream& stream, NonlinearObjectiveFunctionPtr objective);

} // namespace SHOT
