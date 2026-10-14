/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "Interval.h"
#include "Variables.h"
#include "Terms.h"
#include "NonlinearExpressions.h"

#include <map>
#include <utility>

namespace SHOT
{

using SparseVariableIntervalVector = std::map<VariablePtr, Interval, VariableIndexComparator>;
using SparseVariableIntervalMatrix = std::map<std::pair<VariablePtr, VariablePtr>, Interval, VariableIndexComparator>;

// Bounds of the value, the gradient and the Hessian of a function over a box of variable bounds, calculated with
// interval arithmetic by differentiating the terms and the expression tree of the function. The Hessian only has its
// upper triangle, i.e., the elements (V1, V2) with V1->getIndex() <= V2->getIndex(), and elements that are exactly
// zero are left out of both the gradient and the Hessian.
//
// The bounds enclose the classical derivatives over the box only when isDefinedOnBox is true and differentiableOrder
// is at least the order of the derivative. Otherwise they are not reliable, e.g., [-1, 1] is the generalized gradient
// of |x| at x = 0, not a gradient. A bound that is not finite is SHOT_DBL_MIN or SHOT_DBL_MAX, and an element
// calculated from it is not finite either, unless it is multiplied by an exact zero.
//
// The bounds are not rounded outwards, so they are numerical bounds, not verified enclosures of the floating-point
// calculations.
struct IntervalDerivatives
{
    Interval value = Interval(0.0);
    SparseVariableIntervalVector gradient;
    SparseVariableIntervalMatrix hessian;

    // Whether the function is defined everywhere in the box, e.g., not if the argument of a logarithm can be zero
    bool isDefinedOnBox = true;

    // The order (0, 1 or 2) up to which the function is continuously differentiable everywhere in the box, e.g., 0 for
    // |x| or sqrt(x) when x = 0 is in the box
    int differentiableOrder = 2;

    // Whether the bounds of the value and of the calculated derivatives are finite
    bool isBounded = true;
};

// The bounds over the box of the derivatives up to the order (0 for the value only, 1 for the gradient as well and 2
// for the Hessian as well). The box has an interval for each variable, indexed by the indexes of the variables. Throws
// std::invalid_argument for another order.
IntervalDerivatives calculateIntervalDerivatives(const LinearTerms& terms, const IntervalVector& box, int order);
IntervalDerivatives calculateIntervalDerivatives(const QuadraticTerms& terms, const IntervalVector& box, int order);
IntervalDerivatives calculateIntervalDerivatives(const MonomialTerms& terms, const IntervalVector& box, int order);
IntervalDerivatives calculateIntervalDerivatives(const SignomialTerms& terms, const IntervalVector& box, int order);
IntervalDerivatives calculateIntervalDerivatives(
    const NonlinearExpressionPtr& expression, const IntervalVector& box, int order);

// Adds the bounds of a part of a function, e.g., its quadratic terms, to those of the function. The elements that
// cancel to exactly zero are removed, and isBounded is updated.
void addIntervalDerivatives(IntervalDerivatives& sum, const IntervalDerivatives& part, int order);

// Throws std::invalid_argument for an order other than 0, 1 or 2
void checkIntervalDerivativeOrder(int order);

} // namespace SHOT
