/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "ModelShared.h"
#include "../UtilityFunctions.h"
#include "Variables.h"
#include "Terms.h"
#include "NonlinearExpressions.h"
#include "OptimizationProblem.h"

#include <vector>
#include <string>
#include <memory>

namespace SHOT
{

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

    E_ObjectiveFunctionDirection direction;
    bool isMinimize = false;
    bool isMaximize = false;

    E_Curvature curvature = E_Curvature::Convex;

    bool isReformulated = false;

    E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::None;

    bool hasLinearTerms = false;
    bool hasSignomialTerms = false;
    bool hasQuadraticTerms = false;
    bool hasNonlinearExpression = false;
    bool hasNonalgebraicPart = false; // E.g for external functions
};

class ObjectiveFunction
{

  public:
    ~ObjectiveFunction()
    {
    }

    ObjectiveFunctionProperties properties;

    double constant = 0.0;

    OptimizationProblemPtr ownerProblem;

    void takeOwnership(OptimizationProblemPtr owner);

    E_Curvature checkConvexity();

    virtual void updateProperties();

    virtual double calculateValue(const VectorDouble &point) = 0;
    virtual Interval calculateValue(const IntervalVector &intervalVector) = 0;

    virtual SparseVariableVector calculateGradient(const VectorDouble &point) = 0;

    virtual std::ostream &print(std::ostream &) const = 0;
};

std::ostream &operator<<(std::ostream &stream, ObjectiveFunctionPtr objective);

class LinearObjectiveFunction : public ObjectiveFunction
{
  public:
    LinearObjectiveFunction()
    {
    }

    LinearObjectiveFunction(E_ObjectiveFunctionDirection direction)
    {
        properties.direction = direction;
        updateProperties();
    }

    LinearObjectiveFunction(E_ObjectiveFunctionDirection direction, LinearTerms linTerms, double constValue)
    {
        properties.direction = direction;
        linearTerms = linTerms;
        constant = constValue;
        updateProperties();
    }

    virtual ~LinearObjectiveFunction(){};

    LinearTerms linearTerms;

    void add(LinearTerms terms);

    void add(LinearTermPtr term);
    virtual void updateProperties() override;

    virtual double calculateValue(const VectorDouble &point) override;
    virtual Interval calculateValue(const IntervalVector &intervalVector) override;

    virtual SparseVariableVector calculateGradient(const VectorDouble &point) override;

    std::ostream &print(std::ostream &stream) const override;
};

class QuadraticObjectiveFunction : public LinearObjectiveFunction
{
  public:
    QuadraticObjectiveFunction()
    {
    }

    QuadraticObjectiveFunction(E_ObjectiveFunctionDirection direction)
    {
        properties.direction = direction;
        updateProperties();
    }

    QuadraticObjectiveFunction(E_ObjectiveFunctionDirection direction, QuadraticTerms quadTerms, double constValue)
    {
        properties.direction = direction;
        quadraticTerms = quadTerms;
        constant = constValue;
        updateProperties();
    }

    QuadraticObjectiveFunction(E_ObjectiveFunctionDirection direction, LinearTerms linTerms, QuadraticTerms quadTerms, double constValue)
    {
        properties.direction = direction;
        linearTerms = linTerms;
        quadraticTerms = quadTerms;
        constant = constValue;
        updateProperties();
    }

    virtual ~QuadraticObjectiveFunction(){};

    QuadraticTerms quadraticTerms;

    void add(LinearTerms terms);
    void add(LinearTermPtr term);
    void add(QuadraticTerms terms);
    void add(QuadraticTermPtr term);

    virtual void updateProperties() override;

    virtual double calculateValue(const VectorDouble &point) override;
    virtual Interval calculateValue(const IntervalVector &intervalVector) override;

    virtual SparseVariableVector calculateGradient(const VectorDouble &point) override;

    std::ostream &print(std::ostream &stream) const override;
};

std::ostream &operator<<(std::ostream &stream, QuadraticObjectiveFunctionPtr objective);

class NonlinearObjectiveFunction : public QuadraticObjectiveFunction
{
  public:
    NonlinearObjectiveFunction()
    {
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection direction)
    {
        properties.direction = direction;
        updateProperties();
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection direction, NonlinearExpressionPtr nonlinExpr, double constValue)
    {
        properties.direction = direction;
        nonlinearExpression = nonlinExpr;
        constant = constValue;
        updateProperties();
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection direction, LinearTerms linTerms, NonlinearExpressionPtr nonlinExpr, double constValue)
    {
        properties.direction = direction;
        linearTerms = linTerms;
        nonlinearExpression = nonlinExpr;
        constant = constValue;
        updateProperties();
    }

    NonlinearObjectiveFunction(E_ObjectiveFunctionDirection direction, LinearTerms linTerms, QuadraticTerms quadTerms, NonlinearExpressionPtr nonlinExpr, double constValue)
    {
        properties.direction = direction;
        linearTerms = linTerms;
        quadraticTerms = quadTerms;
        nonlinearExpression = nonlinExpr;
        constant = constValue;
        updateProperties();
    }

    virtual ~NonlinearObjectiveFunction(){};

    NonlinearExpressionPtr nonlinearExpression;
    FactorableFunctionPtr factorableFunction;
    std::vector<std::pair<VariablePtr, FactorableFunction>> symbolicSparseJacobian;

    int factorableFunctionIndex;

    void add(LinearTerms terms)
    {
        LinearObjectiveFunction::add(terms);
    }

    void add(LinearTermPtr term)
    {
        LinearObjectiveFunction::add(term);
    }

    void add(QuadraticTerms terms)
    {
        QuadraticObjectiveFunction::add(terms);
    }

    void add(QuadraticTermPtr term)
    {
        QuadraticObjectiveFunction::add(term);
    }

    void add(NonlinearExpressionPtr expression);

    void updateFactorableFunction();

    virtual void updateProperties() override;

    virtual double calculateValue(const VectorDouble &point) override;
    virtual Interval calculateValue(const IntervalVector &intervalVector) override;

    virtual SparseVariableVector calculateGradient(const VectorDouble &point) override;

    std::ostream &print(std::ostream &stream) const override;
};

std::ostream &operator<<(std::ostream &stream, NonlinearObjectiveFunctionPtr objective);

} // namespace SHOT
