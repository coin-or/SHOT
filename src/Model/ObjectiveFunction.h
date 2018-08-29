
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

    E_Curvature curvature;

    bool isReformulated = false;

    E_ObjectiveFunctionClassification classification;

    bool hasLinearTerms = false;
    bool hasSignomialTerms = false;
    bool hasQuadraticTerms = false;
    bool hasNonlinearExpression = false;
    bool hasNonalgebraicPart = false; // E.g for external functions
};

class ObjectiveFunction
{
  public:
    virtual ~ObjectiveFunction() = 0;

    ObjectiveFunctionProperties properties;
    virtual void updateProperties();

    virtual double calculateNumericValue(VectorDouble point) = 0;
};

class LinearObjectiveFunction : ObjectiveFunction
{
  public:
    LinearObjectiveFunction()
    {
    }

    virtual ~LinearObjectiveFunction();

    LinearTerms linearTerms;

    virtual void updateProperties();
};

class QuadraticObjectiveFunction : ObjectiveFunction
{
  public:
    QuadraticObjectiveFunction()
    {
    }

    virtual ~QuadraticObjectiveFunction();

    LinearTerms linearTerms;
    QuadraticTerms quadraticTerms;

    virtual void updateProperties();
};

class NonlinearObjectiveFunction : ObjectiveFunction
{
  public:
    NonlinearObjectiveFunction()
    {
    }

    virtual ~NonlinearObjectiveFunction();

    LinearTerms linearTerms;
    QuadraticTerms quadraticTerms;
    NonlinearExpressionPtr nonlinearExpression;

    virtual void updateProperties();
};
} // namespace SHOT