
/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "ModelStructs.h"
#include "Terms.h"
#include "NonlinearExpressions.h"

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

    void takeOwnership(OptimizationProblemPtr owner)
    {
        ownerProblem = owner;
    }

    E_Curvature checkConvexity()
    {
        return E_Curvature::Convex;
    }

    virtual void updateProperties()
    {
        if (properties.direction == E_ObjectiveFunctionDirection::Minimize)
        {
            properties.isMinimize = true;
            properties.isMaximize = false;
        }

        properties.curvature = checkConvexity();
    }

    virtual double calculateNumericValue(const VectorDouble &point) = 0;

    virtual std::ostream &print(std::ostream &) const = 0;
    friend std::ostream &operator<<(std::ostream &stream, const ObjectiveFunction &objective)
    {
        return objective.print(stream); // polymorphic print via reference
    };
};

typedef std::shared_ptr<ObjectiveFunction> ObjectiveFunctionPtr;

std::ostream &operator<<(std::ostream &stream, ObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
}

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

    void add(LinearTerms terms)
    {
        if (linearTerms.terms.size() == 0)
        {
            linearTerms = terms;
            properties.isValid = false;
        }
        else
        {
            for (auto T : terms.terms)
            {
                add(T);
            }
        }
    };

    void add(LinearTermPtr term)
    {
        linearTerms.terms.push_back(term);
        properties.isValid = false;
    };

    virtual void updateProperties() override
    {
        if (linearTerms.terms.size() > 0)
        {
            properties.hasLinearTerms = true;
        }

        if (!(properties.hasNonlinearExpression || properties.hasSignomialTerms || properties.hasNonalgebraicPart || properties.hasQuadraticTerms))
            E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::Linear;

        ObjectiveFunction::updateProperties();
    };

    virtual double calculateNumericValue(const VectorDouble &point) override
    {
        double value = linearTerms.calculate(point);
        return value;
    };

    std::ostream &print(std::ostream &stream) const override
    {
        if (properties.isMinimize)
            stream << "minimize: ";
        else if (properties.isMaximize)
            stream << "maximize: ";

        if (constant != 0.0)
            stream << constant;

        if (properties.hasLinearTerms)
            stream << linearTerms;

        return stream;
    };
};

typedef std::shared_ptr<LinearObjectiveFunction> LinearObjectiveFunctionPtr;

std::ostream &operator<<(std::ostream &stream, LinearObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
}

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

    void add(QuadraticTerms terms)
    {
        if (quadraticTerms.terms.size() == 0)
        {
            quadraticTerms = terms;
            properties.isValid = false;
        }
        else
        {
            for (auto T : terms.terms)
            {
                add(T);
            }
        }
    };

    void add(QuadraticTermPtr term)
    {
        quadraticTerms.terms.push_back(term);
        properties.isValid = false;
    };

    virtual void updateProperties() override
    {
        if (quadraticTerms.terms.size() > 0)
        {
            properties.hasQuadraticTerms = true;

            if (!(properties.hasNonlinearExpression || properties.hasSignomialTerms || properties.hasNonalgebraicPart))
                E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::Quadratic;
        }

        LinearObjectiveFunction::updateProperties();
    };

    virtual double calculateNumericValue(const VectorDouble &point) override
    {
        double value = LinearObjectiveFunction::calculateNumericValue(point);
        value += quadraticTerms.calculate(point);
        return value;
    };

    std::ostream &print(std::ostream &stream) const override
    {
        LinearObjectiveFunction::print(stream);

        if (properties.hasQuadraticTerms)
            stream << quadraticTerms;

        return stream;
    };
};

typedef std::shared_ptr<QuadraticObjectiveFunction> QuadraticObjectiveFunctionPtr;

std::ostream &operator<<(std::ostream &stream, QuadraticObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
};

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

    void add(NonlinearExpressionPtr expression)
    {
        if (nonlinearExpression != nullptr)
        {
            auto tmpExpr = nonlinearExpression;
            auto nonlinearExpression(new ExpressionPlus(tmpExpr, expression));
        }
        else
        {
            nonlinearExpression = expression;
        }

        properties.isValid = false;
    }

    virtual void updateProperties() override
    {
        if (nonlinearExpression != nullptr)
        {
            properties.hasNonlinearExpression = true;

            if (properties.hasNonalgebraicPart)
            {
                E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::Nonalgebraic;
            }
            else if (properties.hasSignomialTerms)
            {
                E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::GeneralizedSignomial;
            }
            else
            {
                E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::Nonlinear;
            }
        }

        QuadraticObjectiveFunction::updateProperties();
    }

    virtual double calculateNumericValue(const VectorDouble &point) override
    {
        double value = QuadraticObjectiveFunction::calculateNumericValue(point);
        value += nonlinearExpression->calculate(point);
        return value;
    };

    std::ostream &print(std::ostream &stream) const override
    {
        QuadraticObjectiveFunction::print(stream);

        if (properties.hasNonlinearExpression)
            stream << " +(" << nonlinearExpression << ')';

        return stream;
    };
};

typedef std::shared_ptr<NonlinearObjectiveFunction> NonlinearObjectiveFunctionPtr;

std::ostream &operator<<(std::ostream &stream, NonlinearObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
};

} // namespace SHOT
