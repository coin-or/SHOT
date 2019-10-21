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

#include "cppad/cppad.hpp"
#include "cppad/utility.hpp"

namespace SHOT
{

using Interval = mc::Interval;
using IntervalVector = std::vector<Interval>;

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
    ~ObjectiveFunction() = default;

    ObjectiveFunctionProperties properties;

    E_ObjectiveFunctionDirection direction;
    double constant = 0.0;

    std::weak_ptr<Problem> ownerProblem;

    std::shared_ptr<Variables> gradientSparsityPattern;
    std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> hessianSparsityPattern;

    virtual void takeOwnership(ProblemPtr owner) = 0;

    virtual void updateProperties();

    virtual double calculateValue(const VectorDouble& point) = 0;
    virtual Interval calculateValue(const IntervalVector& intervalVector) = 0;

    virtual Interval getBounds();

    virtual SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) = 0;
    virtual std::shared_ptr<Variables> getGradientSparsityPattern();

    virtual SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) = 0;
    virtual std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> getHessianSparsityPattern();

    virtual std::ostream& print(std::ostream&) const = 0;

protected:
    virtual void initializeGradientSparsityPattern() = 0;
    virtual void initializeHessianSparsityPattern() = 0;
};

using ObjectiveFunctionPtr = std::shared_ptr<ObjectiveFunction>;

std::ostream& operator<<(std::ostream& stream, ObjectiveFunctionPtr objective);

class LinearObjectiveFunction : public ObjectiveFunction
{
public:
    LinearObjectiveFunction()
    {
        linearTerms = LinearTerms();
        direction = E_ObjectiveFunctionDirection::Minimize;
    }

    LinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection)
    {
        linearTerms = LinearTerms();

        direction = objectiveDirection;
        updateProperties();
    }

    LinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection, double constValue)
    {
        linearTerms = LinearTerms();
        constant = constValue;

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

    virtual ~LinearObjectiveFunction() = default;

    LinearTerms linearTerms;

    void add(LinearTerms terms);

    void add(LinearTermPtr term);
    void updateProperties() override;

    virtual bool isDualUnbounded();

    void takeOwnership(ProblemPtr owner) override;

    double calculateValue(const VectorDouble& point) override;
    Interval calculateValue(const IntervalVector& intervalVector) override;

    SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) override;

    SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    void initializeGradientSparsityPattern() override;
    void initializeHessianSparsityPattern() override;
};

using LinearObjectiveFunctionPtr = std::shared_ptr<LinearObjectiveFunction>;

class QuadraticObjectiveFunction : public LinearObjectiveFunction
{
public:
    QuadraticObjectiveFunction()
    {
        linearTerms = LinearTerms();
        quadraticTerms = QuadraticTerms();
        direction = E_ObjectiveFunctionDirection::Minimize;
    }

    QuadraticObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection)
    {
        linearTerms = LinearTerms();
        quadraticTerms = QuadraticTerms();

        direction = objectiveDirection;
        updateProperties();
    }

    QuadraticObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection, double constValue)
    {
        linearTerms = LinearTerms();
        quadraticTerms = QuadraticTerms();

        direction = objectiveDirection;
        constant = constValue;
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

    ~QuadraticObjectiveFunction() override = default;

    QuadraticTerms quadraticTerms;

    void add(LinearTerms terms) { LinearObjectiveFunction::add(terms); }

    void add(LinearTermPtr term) { LinearObjectiveFunction::add(term); }

    void add(QuadraticTerms terms);
    void add(QuadraticTermPtr term);

    void updateProperties() override;

    virtual bool isDualUnbounded() override;

    double calculateValue(const VectorDouble& point) override;
    Interval calculateValue(const IntervalVector& intervalVector) override;

    void takeOwnership(ProblemPtr owner) override;

    SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) override;
    SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    void initializeGradientSparsityPattern() override;
    void initializeHessianSparsityPattern() override;
};

using QuadraticObjectiveFunctionPtr = std::shared_ptr<QuadraticObjectiveFunction>;

class NonlinearObjectiveFunction : public QuadraticObjectiveFunction
{
public:
    NonlinearObjectiveFunction()
    {
        this->linearTerms = LinearTerms();
        this->quadraticTerms = QuadraticTerms();
        this->monomialTerms = MonomialTerms();
        this->signomialTerms = SignomialTerms();
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection)
    {
        linearTerms = LinearTerms();
        quadraticTerms = QuadraticTerms();
        this->monomialTerms = MonomialTerms();
        this->signomialTerms = SignomialTerms();

        direction = objectiveDirection;
        updateProperties();
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection, double constValue)
    {
        linearTerms = LinearTerms();
        quadraticTerms = QuadraticTerms();
        this->monomialTerms = MonomialTerms();
        this->signomialTerms = SignomialTerms();

        direction = objectiveDirection;
        constant = constValue;
        updateProperties();
    }

    NonlinearObjectiveFunction(
        E_ObjectiveFunctionDirection objectiveDirection, NonlinearExpressionPtr nonlinExpr, double constValue)
    {
        linearTerms = LinearTerms();
        quadraticTerms = QuadraticTerms();
        this->monomialTerms = MonomialTerms();
        this->signomialTerms = SignomialTerms();

        direction = objectiveDirection;
        nonlinearExpression = nonlinExpr;
        constant = constValue;
        updateProperties();
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection, LinearTerms linTerms,
        NonlinearExpressionPtr nonlinExpr, double constValue)
    {
        quadraticTerms = QuadraticTerms();
        this->monomialTerms = MonomialTerms();
        this->signomialTerms = SignomialTerms();

        direction = objectiveDirection;
        linearTerms = linTerms;
        nonlinearExpression = nonlinExpr;
        constant = constValue;
        updateProperties();
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection objectiveDirection, LinearTerms linTerms,
        QuadraticTerms quadTerms, NonlinearExpressionPtr nonlinExpr, double constValue)
    {
        this->monomialTerms = MonomialTerms();
        this->signomialTerms = SignomialTerms();

        direction = objectiveDirection;
        linearTerms = linTerms;
        quadraticTerms = quadTerms;
        nonlinearExpression = nonlinExpr;
        constant = constValue;
        updateProperties();
    }

    ~NonlinearObjectiveFunction() override = default;

    MonomialTerms monomialTerms;
    SignomialTerms signomialTerms;

    NonlinearExpressionPtr nonlinearExpression;
    FactorableFunctionPtr factorableFunction;

    CppAD::sparse_rc<std::vector<size_t>> nonlinearGradientSparsityPattern;
    CppAD::sparse_rc<std::vector<size_t>> nonlinearHessianSparsityPattern;

    bool nonlinearGradientSparsityMapGenerated = false;
    bool nonlinearHessianSparsityMapGenerated = false;

    Variables variablesInMonomialTerms;
    Variables variablesInSignomialTerms;
    Variables variablesInNonlinearExpression;

    int nonlinearExpressionIndex = -1;

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

    void updateProperties() override;

    void takeOwnership(ProblemPtr owner) override;

    double calculateValue(const VectorDouble& point) override;
    Interval calculateValue(const IntervalVector& intervalVector) override;

    SparseVariableVector calculateGradient(const VectorDouble& point, bool eraseZeroes) override;
    SparseVariableMatrix calculateHessian(const VectorDouble& point, bool eraseZeroes) override;

    std::ostream& print(std::ostream& stream) const override;

protected:
    void initializeGradientSparsityPattern() override;
    void initializeHessianSparsityPattern() override;
};

using NonlinearObjectiveFunctionPtr = std::shared_ptr<NonlinearObjectiveFunction>;

std::ostream& operator<<(std::ostream& stream, NonlinearObjectiveFunctionPtr objective);

} // namespace SHOT
