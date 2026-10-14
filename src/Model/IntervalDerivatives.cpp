/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "IntervalDerivatives.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace SHOT
{

namespace
{
    using Derivatives = IntervalDerivatives;

    // The derivatives of the expressions that are used more than once in the expression tree, e.g., e in e + e, so
    // that they are calculated once for each evaluation. The uses are counted in the tree before it is evaluated, since
    // the number of references to an expression also counts those from outside the tree, e.g., from Python.
    struct Memo
    {
        std::unordered_map<const NonlinearExpression*, int> remainingUses;
        std::unordered_map<const NonlinearExpression*, Derivatives> results;
    };

    // ── Interval arithmetic ─────────────────────────────────────────────────────────────────────────────────────
    // SHOT_DBL_MIN and SHOT_DBL_MAX are the ends of an unbounded interval, and are treated as infinities here: the
    // interval arithmetic would otherwise use them as finite numbers, e.g., 0.5 * SHOT_DBL_MAX, and the sum
    // SHOT_DBL_MAX + SHOT_DBL_MIN would be zero.

    inline bool isInfiniteEnd(double value) { return (!(value > SHOT_DBL_MIN && value < SHOT_DBL_MAX)); }

    inline bool isUnbounded(const Interval& interval)
    {
        return (isInfiniteEnd(interval.l()) || isInfiniteEnd(interval.u()));
    }

    inline bool isExactlyZero(const Interval& interval) { return (interval.l() == 0.0 && interval.u() == 0.0); }

    inline Interval unbounded() { return (Interval(SHOT_DBL_MIN, SHOT_DBL_MAX)); }

    // An interval with the ends of a result, where a NaN gives no information and an infinity becomes the bound used
    // for one
    inline Interval normalize(double lower, double upper)
    {
        if(std::isnan(lower) || std::isnan(upper))
            return (unbounded());

        return (Interval(std::clamp(lower, SHOT_DBL_MIN, SHOT_DBL_MAX), std::clamp(upper, SHOT_DBL_MIN, SHOT_DBL_MAX)));
    }

    inline Interval add(const Interval& first, const Interval& second)
    {
        double lower
            = (first.l() <= SHOT_DBL_MIN || second.l() <= SHOT_DBL_MIN) ? SHOT_DBL_MIN : first.l() + second.l();
        double upper
            = (first.u() >= SHOT_DBL_MAX || second.u() >= SHOT_DBL_MAX) ? SHOT_DBL_MAX : first.u() + second.u();
        return (normalize(lower, upper));
    }

    inline Interval negate(const Interval& interval) { return (Interval(-interval.u(), -interval.l())); }

    // The product of two ends, where zero times an infinity is zero, since the ends of the intervals are attained
    inline double multiplyEnds(double first, double second)
    {
        if(first == 0.0 || second == 0.0)
            return (0.0);

        if(isInfiniteEnd(first) || isInfiniteEnd(second))
            return ((first > 0.0) == (second > 0.0) ? SHOT_DBL_MAX : SHOT_DBL_MIN);

        return (first * second);
    }

    inline Interval multiply(const Interval& first, const Interval& second)
    {
        if(isExactlyZero(first) || isExactlyZero(second))
            return (Interval(0.0));

        double products[4] = { multiplyEnds(first.l(), second.l()), multiplyEnds(first.l(), second.u()),
            multiplyEnds(first.u(), second.l()), multiplyEnds(first.u(), second.u()) };

        return (normalize(*std::min_element(products, products + 4), *std::max_element(products, products + 4)));
    }

    inline Interval multiply(const Interval& interval, double factor) { return (multiply(interval, Interval(factor))); }

    inline Interval square(const Interval& interval)
    {
        double lower = multiplyEnds(interval.l(), interval.l());
        double upper = multiplyEnds(interval.u(), interval.u());

        if(interval.l() >= 0.0)
            return (normalize(lower, upper));

        if(interval.u() <= 0.0)
            return (normalize(upper, lower));

        return (normalize(0.0, std::max(lower, upper)));
    }

    // 1 / interval for an interval without zero, where 1 / infinity is zero
    inline Interval inverse(const Interval& interval)
    {
        auto inverseEnd = [](double value) { return (isInfiniteEnd(value) ? 0.0 : 1.0 / value); };
        return (normalize(inverseEnd(interval.u()), inverseEnd(interval.l())));
    }

    inline Interval increasing(const Interval& interval, const std::function<double(double)>& function)
    {
        return (normalize(function(interval.l()), function(interval.u())));
    }

    inline bool isInteger(double value) { return (std::trunc(value) == value); }

    // base^exponent for an integer exponent, or for a base without negative values when the exponent is not an
    // integer. The base has no zero when the exponent is negative. The power is monotone on each side of zero, so it is
    // calculated at the ends of the base, and a base near zero is not moved, e.g., to avoid the logarithm of zero.
    inline Interval power(const Interval& base, double exponent)
    {
        if(exponent == 0.0)
            return (Interval(1.0));

        if(isInteger(exponent) && base.l() < 0.0)
        {
            // An even power is the power of the absolute value
            if(std::fmod(exponent, 2.0) == 0.0)
            {
                auto absolute = base.u() <= 0.0 ? negate(base) : Interval(0.0, std::max(-base.l(), base.u()));
                return (power(absolute, exponent));
            }

            // An odd power is increasing for a positive exponent, and decreasing on each side of zero otherwise
            if(exponent > 0.0)
                return (normalize(std::pow(base.l(), exponent), std::pow(base.u(), exponent)));

            return (normalize(std::pow(base.u(), exponent), std::pow(base.l(), exponent)));
        }

        // The power of a base without negative values is increasing for a positive exponent and decreasing otherwise
        if(exponent > 0.0)
            return (normalize(std::pow(base.l(), exponent), std::pow(base.u(), exponent)));

        return (normalize(std::pow(base.u(), exponent), std::pow(base.l(), exponent)));
    }

    // The bounds of |x| for x in the interval
    inline Interval absoluteValues(const Interval& interval)
    {
        if(interval.l() >= 0.0)
            return (interval);

        if(interval.u() <= 0.0)
            return (negate(interval));

        return (Interval(0.0, std::max(-interval.l(), interval.u())));
    }

    // ── Operations on the derivatives ───────────────────────────────────────────────────────────────────────────

    inline std::pair<VariablePtr, VariablePtr> getHessianKey(const VariablePtr& first, const VariablePtr& second)
    {
        return (
            first->getIndex() <= second->getIndex() ? std::make_pair(first, second) : std::make_pair(second, first));
    }

    template <typename Map, typename Key> inline void addElement(Map& map, const Key& key, const Interval& value)
    {
        auto element = map.emplace(key, value);

        if(!element.second)
            element.first->second = add(element.first->second, value);
    }

    inline void combineStatus(Derivatives& target, const Derivatives& source)
    {
        target.isDefinedOnBox = target.isDefinedOnBox && source.isDefinedOnBox;
        target.differentiableOrder = std::min(target.differentiableOrder, source.differentiableOrder);
    }

    inline void restrictStatus(Derivatives& derivatives, bool isDefinedOnBox, int differentiableOrder)
    {
        derivatives.isDefinedOnBox = derivatives.isDefinedOnBox && isDefinedOnBox;
        derivatives.differentiableOrder = std::min(derivatives.differentiableOrder, differentiableOrder);
    }

    Derivatives constant(double value)
    {
        Derivatives result;
        result.value = Interval(value);
        return (result);
    }

    Derivatives variable(const VariablePtr& variable, const IntervalVector& box, int order)
    {
        Derivatives result;
        const auto& bounds = box[variable->getIndex()];
        result.value = normalize(bounds.l(), bounds.u());

        if(order >= 1)
            result.gradient.emplace(variable, Interval(1.0));

        return (result);
    }

    // Adds the elements of a map to those of another, where the nodes of the keys that are only in the source are moved
    template <typename Map> void addElements(Map& target, Map&& source)
    {
        if(target.empty())
        {
            target = std::move(source);
            return;
        }

        target.merge(source);

        // The elements left in the source have keys that are also in the target
        for(auto& [K, E] : source)
            target[K] = add(target[K], E);
    }

    void accumulate(Derivatives& sum, Derivatives&& term, int order)
    {
        sum.value = add(sum.value, term.value);

        if(order >= 1)
            addElements(sum.gradient, std::move(term.gradient));

        if(order >= 2)
            addElements(sum.hessian, std::move(term.hessian));

        combineStatus(sum, term);
    }

    void accumulate(Derivatives& sum, const Derivatives& term, int order)
    {
        sum.value = add(sum.value, term.value);

        if(order >= 1)
        {
            for(auto& [V, G] : term.gradient)
                addElement(sum.gradient, V, G);
        }

        if(order >= 2)
        {
            for(auto& [K, H] : term.hessian)
                addElement(sum.hessian, K, H);
        }

        combineStatus(sum, term);
    }

    // The derivatives are scaled in place, so that a temporary passed here is not copied
    Derivatives scale(Derivatives derivatives, double factor, int order)
    {
        derivatives.value = multiply(derivatives.value, factor);

        if(order >= 1)
        {
            for(auto& [V, G] : derivatives.gradient)
                G = multiply(G, factor);
        }

        if(order >= 2)
        {
            for(auto& [K, H] : derivatives.hessian)
                H = multiply(H, factor);
        }

        return (derivatives);
    }

    // The product rule: (ab)' = a'b + ab' and (ab)'' = a''b + ab'' + a'b'^T + b'a'^T
    Derivatives product(const Derivatives& first, const Derivatives& second, int order)
    {
        Derivatives result;
        result.value = multiply(first.value, second.value);

        if(order >= 1)
        {
            for(auto& [V, G] : first.gradient)
                addElement(result.gradient, V, multiply(G, second.value));

            for(auto& [V, G] : second.gradient)
                addElement(result.gradient, V, multiply(G, first.value));
        }

        if(order >= 2)
        {
            for(auto& [K, H] : first.hessian)
                addElement(result.hessian, K, multiply(H, second.value));

            for(auto& [K, H] : second.hessian)
                addElement(result.hessian, K, multiply(H, first.value));

            // The element (i, j) of a'b'^T + b'a'^T is a'_i b'_j + a'_j b'_i, so each product a'_i b'_j is added once
            // to the element of the upper triangle, and twice to a diagonal element
            for(auto& [V1, G1] : first.gradient)
            {
                for(auto& [V2, G2] : second.gradient)
                {
                    auto value = multiply(G1, G2);
                    addElement(result.hessian, getHessianKey(V1, V2), V1 == V2 ? multiply(value, 2.0) : value);
                }
            }
        }

        combineStatus(result, first);
        combineStatus(result, second);
        return (result);
    }

    // The product rule for several factors: (f_1 ... f_n)' = sum_k P_k f_k' and
    // (f_1 ... f_n)'' = sum_k P_k f_k'' + sum_{k != l} P_kl f_k' f_l'^T, where P_k is the product of the factors other
    // than f_k and P_kl that of the factors other than f_k and f_l. The products are calculated from the products of
    // the factors before and after each factor, without dividing, so the work is quadratic in the number of factors,
    // while multiplying the factors one at a time rebuilds the growing Hessian for each factor.
    Derivatives product(std::vector<Derivatives> factors, int order)
    {
        if(factors.empty())
            return (constant(1.0));

        if(factors.size() == 1)
            return (std::move(factors[0]));

        size_t numberOfFactors = factors.size();
        std::vector<Interval> before(numberOfFactors + 1, Interval(1.0));
        std::vector<Interval> after(numberOfFactors + 1, Interval(1.0));

        for(size_t k = 0; k < numberOfFactors; k++)
            before[k + 1] = multiply(before[k], factors[k].value);

        for(size_t k = numberOfFactors; k-- > 0;)
            after[k] = multiply(after[k + 1], factors[k].value);

        Derivatives result;
        result.value = before[numberOfFactors];

        for(size_t k = 0; k < numberOfFactors; k++)
        {
            const auto& factor = factors[k];
            auto others = multiply(before[k], after[k + 1]);

            if(order >= 1)
            {
                for(auto& [V, G] : factor.gradient)
                    addElement(result.gradient, V, multiply(G, others));
            }

            if(order >= 2)
            {
                for(auto& [K, H] : factor.hessian)
                    addElement(result.hessian, K, multiply(H, others));

                // The product of the factors between f_k and f_l is extended by one factor for each l
                Interval between(1.0);

                for(size_t l = k + 1; l < numberOfFactors; l++)
                {
                    auto weight = multiply(multiply(before[k], between), after[l + 1]);
                    between = multiply(between, factors[l].value);

                    if(isExactlyZero(weight))
                        continue;

                    for(auto& [V1, G1] : factor.gradient)
                    {
                        for(auto& [V2, G2] : factors[l].gradient)
                        {
                            auto value = multiply(multiply(G1, G2), weight);
                            addElement(result.hessian, getHessianKey(V1, V2), V1 == V2 ? multiply(value, 2.0) : value);
                        }
                    }
                }
            }

            combineStatus(result, factor);
        }

        return (result);
    }

    // The bounds of f(u), f'(u) and f''(u) over the bounds of an argument u
    struct UnaryBounds
    {
        Interval value;
        Interval firstDerivative;
        Interval secondDerivative;
        bool isDefinedOnBox = true;
        int differentiableOrder = 2;
    };

    // The chain rule: f(u)' = f'(u) u' and f(u)'' = f''(u) u'u'^T + f'(u) u''. The derivatives of the argument are
    // changed in place into those of the function.
    Derivatives compose(Derivatives argument, const UnaryBounds& bounds, int order)
    {
        argument.value = bounds.value;

        // The Hessian is calculated first, since it uses the gradient of the argument
        if(order >= 2)
        {
            for(auto& [K, H] : argument.hessian)
                H = multiply(H, bounds.firstDerivative);

            for(auto first = argument.gradient.begin(); first != argument.gradient.end(); ++first)
            {
                // The square of an element is used on the diagonal, since it is tighter than the product
                addElement(argument.hessian, getHessianKey(first->first, first->first),
                    multiply(square(first->second), bounds.secondDerivative));

                for(auto second = std::next(first); second != argument.gradient.end(); ++second)
                {
                    addElement(argument.hessian, getHessianKey(first->first, second->first),
                        multiply(multiply(first->second, second->second), bounds.secondDerivative));
                }
            }
        }

        if(order >= 1)
        {
            for(auto& [V, G] : argument.gradient)
                G = multiply(G, bounds.firstDerivative);
        }

        restrictStatus(argument, bounds.isDefinedOnBox, bounds.differentiableOrder);
        return (argument);
    }

    // A function that is not defined anywhere in the box, or whose bounds cannot be calculated
    UnaryBounds unboundedBounds(bool isDefinedOnBox, int differentiableOrder)
    {
        UnaryBounds bounds { unbounded(), unbounded(), unbounded(), isDefinedOnBox, differentiableOrder };
        return (bounds);
    }

    // ── The elementary functions ────────────────────────────────────────────────────────────────────────────────
    // The functions below are called with an argument with finite bounds, since the bounds of a function of an
    // unbounded argument are not calculated, but are unbounded.

    UnaryBounds inverseBounds(const Interval& argument)
    {
        if(argument.l() <= 0.0 && argument.u() >= 0.0)
            return (unboundedBounds(false, 0));

        auto inverted = inverse(argument);
        return (UnaryBounds { inverted, negate(square(inverted)), multiply(power(inverted, 3.0), 2.0) });
    }

    UnaryBounds squareRootBounds(const Interval& argument)
    {
        if(argument.u() < 0.0)
            return (unboundedBounds(false, 0));

        bool isDefinedOnBox = argument.l() >= 0.0;
        Interval domain(std::max(argument.l(), 0.0), argument.u());
        auto value = increasing(domain, [](double x) { return (std::sqrt(x)); });

        // The derivative 1 / (2 sqrt(x)) is not finite in zero
        if(domain.l() == 0.0)
            return (UnaryBounds { value, unbounded(), unbounded(), isDefinedOnBox, 0 });

        auto inverseRoot = inverse(value);
        return (UnaryBounds { value, multiply(inverseRoot, 0.5), multiply(power(inverseRoot, 3.0), -0.25) });
    }

    UnaryBounds logarithmBounds(const Interval& argument)
    {
        if(argument.u() <= 0.0)
            return (unboundedBounds(false, 0));

        if(argument.l() <= 0.0)
        {
            return (UnaryBounds { Interval(SHOT_DBL_MIN, std::log(argument.u())), unbounded(), unbounded(), false, 0 });
        }

        auto inverted = inverse(argument);
        return (UnaryBounds {
            increasing(argument, [](double x) { return (std::log(x)); }), inverted, negate(square(inverted)) });
    }

    UnaryBounds exponentialBounds(const Interval& argument)
    {
        auto value = increasing(argument, [](double x) { return (std::exp(x)); });
        return (UnaryBounds { value, value, value });
    }

    UnaryBounds sineBounds(const Interval& argument)
    {
        auto sine = sin(argument);
        auto cosine = cos(argument);
        return (UnaryBounds { sine, cosine, negate(sine) });
    }

    UnaryBounds cosineBounds(const Interval& argument)
    {
        auto sine = sin(argument);
        auto cosine = cos(argument);
        return (UnaryBounds { cosine, negate(sine), negate(cosine) });
    }

    UnaryBounds tangentBounds(const Interval& argument)
    {
        Interval tangent;

        try
        {
            // Throws if the argument contains a pole
            tangent = tan(argument);
        }
        catch(IntervalError&)
        {
            return (unboundedBounds(false, 0));
        }

        tangent = normalize(tangent.l(), tangent.u());
        auto derivative = add(Interval(1.0), square(tangent));
        return (UnaryBounds { tangent, derivative, multiply(multiply(tangent, derivative), 2.0) });
    }

    // arcsin(x) and arccos(x) = pi/2 - arcsin(x), whose derivatives are +-1 / sqrt(1 - x^2) and +-x / (1 - x^2)^1.5
    UnaryBounds arcSineBounds(const Interval& argument, bool isCosine)
    {
        if(argument.u() < -1.0 || argument.l() > 1.0)
            return (unboundedBounds(false, 0));

        bool isDefinedOnBox = argument.l() >= -1.0 && argument.u() <= 1.0;
        Interval domain(std::max(argument.l(), -1.0), std::min(argument.u(), 1.0));

        auto value = isCosine ? normalize(std::acos(domain.u()), std::acos(domain.l()))
                              : increasing(domain, [](double x) { return (std::asin(x)); });

        // The derivatives are not finite in -1 and 1
        if(domain.l() <= -1.0 || domain.u() >= 1.0)
            return (UnaryBounds { value, unbounded(), unbounded(), isDefinedOnBox, 0 });

        auto remainder = add(Interval(1.0), negate(square(domain)));
        auto inverseRoot = inverse(increasing(remainder, [](double x) { return (std::sqrt(x)); }));
        auto firstDerivative = inverseRoot;
        auto secondDerivative = multiply(domain, power(inverseRoot, 3.0));

        if(isCosine)
            return (UnaryBounds { value, negate(firstDerivative), negate(secondDerivative) });

        return (UnaryBounds { value, firstDerivative, secondDerivative });
    }

    // The derivatives of arctan(x) are 1 / (1 + x^2) and -2x / (1 + x^2)^2. They are calculated from 1 / x for a large
    // x, where x^2 would overflow, or the small result would be lost by squaring 1 / x first.
    UnaryBounds arcTangentBounds(const Interval& argument)
    {
        auto firstDerivative = [](double x) { return (std::abs(x) > 1e8 ? (1.0 / x) / x : 1.0 / (1.0 + x * x)); };

        auto secondDerivative = [](double x)
        { return (std::abs(x) > 1e8 ? ((-2.0 / x) / x) / x : -2.0 * x / ((1.0 + x * x) * (1.0 + x * x))); };

        // The first derivative decreases with |x|, and the second has its extrema in -1/sqrt(3) and 1/sqrt(3)
        auto absolute = absoluteValues(argument);
        Interval first = normalize(firstDerivative(absolute.u()), firstDerivative(absolute.l()));

        std::vector<double> values = { secondDerivative(argument.l()), secondDerivative(argument.u()) };

        for(double criticalPoint : { -1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0) })
        {
            if(argument.l() <= criticalPoint && criticalPoint <= argument.u())
                values.push_back(secondDerivative(criticalPoint));
        }

        return (UnaryBounds { increasing(argument, [](double x) { return (std::atan(x)); }), first,
            normalize(
                *std::min_element(values.begin(), values.end()), *std::max_element(values.begin(), values.end())) });
    }

    UnaryBounds absoluteValueBounds(const Interval& argument)
    {
        if(argument.l() > 0.0)
            return (UnaryBounds { argument, Interval(1.0), Interval(0.0) });

        if(argument.u() < 0.0)
            return (UnaryBounds { negate(argument), Interval(-1.0), Interval(0.0) });

        // |x| is not differentiable in zero, where its generalized gradient is [-1, 1]
        return (UnaryBounds { absoluteValues(argument), Interval(-1.0, 1.0), unbounded(), true, 0 });
    }

    Interval hyperbolicCosine(const Interval& argument)
    {
        double atLower = std::cosh(argument.l());
        double atUpper = std::cosh(argument.u());
        double minimum = (argument.l() <= 0.0 && argument.u() >= 0.0) ? 1.0 : std::min(atLower, atUpper);
        return (normalize(minimum, std::max(atLower, atUpper)));
    }

    UnaryBounds hyperbolicSineBounds(const Interval& argument)
    {
        auto sine = increasing(argument, [](double x) { return (std::sinh(x)); });
        return (UnaryBounds { sine, hyperbolicCosine(argument), sine });
    }

    UnaryBounds hyperbolicCosineBounds(const Interval& argument)
    {
        auto cosine = hyperbolicCosine(argument);
        return (UnaryBounds { cosine, increasing(argument, [](double x) { return (std::sinh(x)); }), cosine });
    }

    // The derivatives of tanh(x) are sech^2(x) and -2 tanh(x) sech^2(x). sech^2(x) = 4z / (1 + z)^2 with z =
    // exp(-2|x|), since 1 - tanh^2(x) is zero when tanh(x) is rounded to one, e.g., for x = 20.
    UnaryBounds hyperbolicTangentBounds(const Interval& argument)
    {
        auto squaredSecant = [](double x)
        {
            double z = std::exp(-2.0 * x);
            return (4.0 * z / ((1.0 + z) * (1.0 + z)));
        };

        // sech^2(x) decreases with |x|
        auto absolute = absoluteValues(argument);
        Interval derivative = normalize(squaredSecant(absolute.u()), squaredSecant(absolute.l()));
        auto tangent = increasing(argument, [](double x) { return (std::tanh(x)); });

        return (UnaryBounds { tangent, derivative, multiply(multiply(tangent, derivative), -2.0) });
    }

    // The standard normal cumulative distribution function, whose derivative is the density phi(x) = exp(-x^2 / 2) /
    // sqrt(2 pi) and second derivative -x phi(x)
    UnaryBounds errorFunctionBounds(const Interval& argument)
    {
        auto value = increasing(argument, [](double x) { return (0.5 * std::erfc(-x / std::sqrt(2.0))); });
        auto density
            = multiply(increasing(negate(multiply(square(argument), 0.5)), [](double x) { return (std::exp(x)); }),
                1.0 / std::sqrt(2.0 * std::acos(-1.0)));
        return (UnaryBounds { value, density, multiply(negate(argument), density) });
    }

    // sign(x) |x|^c with c > 0, whose derivatives are c |x|^(c-1) and c (c-1) sign(x) |x|^(c-2)
    UnaryBounds signPowerBounds(const Interval& argument, double exponent)
    {
        auto signPower = [exponent](double x) { return (x >= 0.0 ? std::pow(x, exponent) : -std::pow(-x, exponent)); };
        auto value = increasing(argument, signPower);

        if(exponent == 1.0)
            return (UnaryBounds { value, Interval(1.0), Interval(0.0) });

        if(argument.l() > 0.0)
        {
            return (UnaryBounds { value, multiply(power(argument, exponent - 1.0), exponent),
                multiply(power(argument, exponent - 2.0), exponent * (exponent - 1.0)) });
        }

        if(argument.u() < 0.0)
        {
            auto absolute = negate(argument);
            return (UnaryBounds { value, multiply(power(absolute, exponent - 1.0), exponent),
                multiply(power(absolute, exponent - 2.0), -exponent * (exponent - 1.0)) });
        }

        // Zero is in the argument, where the derivatives of |x|^c are finite only for c >= 1 and c >= 2, and the second
        // derivative is continuous only for c > 2
        Interval absolute(0.0, std::max(-argument.l(), argument.u()));

        if(exponent < 1.0)
            return (UnaryBounds { value, unbounded(), unbounded(), true, 0 });

        auto firstDerivative = multiply(power(absolute, exponent - 1.0), exponent);

        if(exponent < 2.0)
            return (UnaryBounds { value, firstDerivative, unbounded(), true, 1 });

        double largest = exponent * (exponent - 1.0) * power(absolute, exponent - 2.0).u();
        return (UnaryBounds { value, firstDerivative, normalize(-largest, largest), true, exponent > 2.0 ? 2 : 1 });
    }

    // base^power for a constant power. The powers 0, 1 and 2 are handled first, so that no zero factor multiplies a
    // power of the base that is not defined, e.g., x^(p-2) in x = 0.
    UnaryBounds powerBounds(const Interval& base, double exponent)
    {
        if(exponent == 0.0)
            return (UnaryBounds { Interval(1.0), Interval(0.0), Interval(0.0) });

        if(exponent == 1.0)
            return (UnaryBounds { base, Interval(1.0), Interval(0.0) });

        if(exponent == 2.0)
            return (UnaryBounds { square(base), multiply(base, 2.0), Interval(2.0) });

        auto derivatives = [exponent](const Interval& domain)
        {
            return (UnaryBounds { power(domain, exponent), multiply(power(domain, exponent - 1.0), exponent),
                multiply(power(domain, exponent - 2.0), exponent * (exponent - 1.0)) });
        };

        if(isInteger(exponent))
        {
            // A negative integer power is not defined in zero
            if(exponent < 0.0 && base.l() <= 0.0 && base.u() >= 0.0)
                return (unboundedBounds(false, 0));

            return (derivatives(base));
        }

        // A noninteger power is only defined for a nonnegative base, and a negative one only for a positive base
        if(base.u() < 0.0 || (exponent < 0.0 && base.u() <= 0.0))
            return (unboundedBounds(false, 0));

        bool isDefinedOnBox = exponent > 0.0 ? base.l() >= 0.0 : base.l() > 0.0;
        Interval domain(std::max(base.l(), 0.0), base.u());

        if(domain.l() > 0.0)
        {
            auto bounds = derivatives(domain);
            bounds.isDefinedOnBox = isDefinedOnBox;
            return (bounds);
        }

        // Zero is in the domain, where the derivatives of x^p are finite only for p >= 1 and p >= 2
        if(exponent < 0.0)
            return (unboundedBounds(false, 0));

        auto value = power(domain, exponent);

        if(exponent < 1.0)
            return (UnaryBounds { value, unbounded(), unbounded(), isDefinedOnBox, 0 });

        auto firstDerivative = multiply(power(domain, exponent - 1.0), exponent);

        if(exponent < 2.0)
            return (UnaryBounds { value, firstDerivative, unbounded(), isDefinedOnBox, 1 });

        return (UnaryBounds { value, firstDerivative,
            multiply(power(domain, exponent - 2.0), exponent * (exponent - 1.0)), isDefinedOnBox, 2 });
    }

    // Applies a function to an argument. The bounds of a function of an argument without finite bounds are not
    // calculated, since the ends of the argument are then not numbers, e.g., sqrt(SHOT_DBL_MAX) is not an upper bound
    // of sqrt(x) for an unbounded x. Whether the function is defined and differentiable is still decided from the
    // ends of the argument.
    Derivatives applyFunction(
        Derivatives argument, int order, const std::function<UnaryBounds(const Interval&)>& calculateBounds)
    {
        UnaryBounds bounds;

        try
        {
            bounds = calculateBounds(argument.value);
        }
        catch(IntervalError&)
        {
            bounds = unboundedBounds(false, 0);
        }

        if(isUnbounded(argument.value))
            bounds = unboundedBounds(bounds.isDefinedOnBox, bounds.differentiableOrder);

        return (compose(std::move(argument), bounds, order));
    }

    Derivatives power(Derivatives base, double exponent, int order)
    {
        if(exponent == 1.0)
            return (base);

        return (applyFunction(std::move(base), order,
            [exponent](const Interval& interval) { return (powerBounds(interval, exponent)); }));
    }

    Derivatives invert(Derivatives argument, int order)
    {
        return (applyFunction(std::move(argument), order, inverseBounds));
    }

    // ── The expression tree ─────────────────────────────────────────────────────────────────────────────────────

    Derivatives calculate(const NonlinearExpression& expression, const IntervalVector& box, int order, Memo& memo);

    Derivatives calculateChild(const NonlinearExpressionPtr& child, const IntervalVector& box, int order, Memo& memo);

    // Counts the uses of each expression in the tree below an expression, where the expressions below one that is used
    // more than once are only counted once, since it is only evaluated once
    void countUses(const NonlinearExpression& expression, Memo& memo)
    {
        auto countChild = [&memo](const NonlinearExpressionPtr& child)
        {
            if(++memo.remainingUses[child.get()] == 1)
                countUses(*child, memo);
        };

        if(auto unary = dynamic_cast<const ExpressionUnary*>(&expression))
        {
            countChild(unary->child);
        }
        else if(auto binary = dynamic_cast<const ExpressionBinary*>(&expression))
        {
            countChild(binary->firstChild);
            countChild(binary->secondChild);
        }
        else if(auto general = dynamic_cast<const ExpressionGeneral*>(&expression))
        {
            for(auto& C : general->children)
                countChild(C);
        }
    }

    inline bool isUsedOnce(const NonlinearExpressionPtr& expression, const Memo& memo)
    {
        auto uses = memo.remainingUses.find(expression.get());
        return (uses == memo.remainingUses.end() || uses->second <= 1);
    }

    // The factors of a product, where a product in a product, e.g., from (xy)z, is replaced by its factors, unless it
    // is used more than once in the tree
    void collectFactors(const ExpressionGeneral& productExpression, const IntervalVector& box, int order, Memo& memo,
        std::vector<Derivatives>& factors)
    {
        for(auto& C : productExpression.children)
        {
            if(C->getType() == E_NonlinearExpressionTypes::Product && isUsedOnce(C, memo))
                collectFactors(static_cast<const ExpressionGeneral&>(*C), box, order, memo, factors);
            else
                factors.push_back(calculateChild(C, box, order, memo));
        }
    }

    Derivatives calculateChild(const NonlinearExpressionPtr& child, const IntervalVector& box, int order, Memo& memo)
    {
        auto uses = memo.remainingUses.find(child.get());

        if(uses == memo.remainingUses.end() || uses->second <= 1)
        {
            // The last use of an expression used more than once takes its derivatives from the memo without copying
            auto found = memo.results.find(child.get());

            if(found == memo.results.end())
                return (calculate(*child, box, order, memo));

            auto result = std::move(found->second);
            memo.results.erase(found);
            return (result);
        }

        uses->second--;
        auto found = memo.results.find(child.get());

        if(found != memo.results.end())
            return (found->second);

        auto result = calculate(*child, box, order, memo);
        memo.results.emplace(child.get(), result);
        return (result);
    }

    Derivatives calculate(const NonlinearExpression& expression, const IntervalVector& box, int order, Memo& memo)
    {
        auto unaryChild = [&]()
        { return (calculateChild(static_cast<const ExpressionUnary&>(expression).child, box, order, memo)); };

        switch(expression.getType())
        {
        case E_NonlinearExpressionTypes::Constant:
            return (constant(static_cast<const ExpressionConstant&>(expression).constant));

        case E_NonlinearExpressionTypes::Variable:
            return (variable(static_cast<const ExpressionVariable&>(expression).variable, box, order));

        case E_NonlinearExpressionTypes::Negate:
            return (scale(unaryChild(), -1.0, order));

        case E_NonlinearExpressionTypes::Invert:
            return (invert(unaryChild(), order));

        case E_NonlinearExpressionTypes::SquareRoot:
            return (applyFunction(unaryChild(), order, squareRootBounds));

        case E_NonlinearExpressionTypes::Log:
            return (applyFunction(unaryChild(), order, logarithmBounds));

        case E_NonlinearExpressionTypes::Exp:
            return (applyFunction(unaryChild(), order, exponentialBounds));

        case E_NonlinearExpressionTypes::Square:
            return (power(unaryChild(), 2.0, order));

        case E_NonlinearExpressionTypes::Sin:
            return (applyFunction(unaryChild(), order, sineBounds));

        case E_NonlinearExpressionTypes::Cos:
            return (applyFunction(unaryChild(), order, cosineBounds));

        case E_NonlinearExpressionTypes::Tan:
            return (applyFunction(unaryChild(), order, tangentBounds));

        case E_NonlinearExpressionTypes::ArcSin:
        case E_NonlinearExpressionTypes::ArcCos:
        {
            bool isCosine = expression.getType() == E_NonlinearExpressionTypes::ArcCos;
            return (applyFunction(unaryChild(), order,
                [isCosine](const Interval& interval) { return (arcSineBounds(interval, isCosine)); }));
        }

        case E_NonlinearExpressionTypes::ArcTan:
            return (applyFunction(unaryChild(), order, arcTangentBounds));

        case E_NonlinearExpressionTypes::Abs:
            return (applyFunction(unaryChild(), order, absoluteValueBounds));

        case E_NonlinearExpressionTypes::ErrorFunction:
            return (applyFunction(unaryChild(), order, errorFunctionBounds));

        case E_NonlinearExpressionTypes::SignPower:
        {
            double exponent = static_cast<const ExpressionSignPower&>(expression).exponent;
            return (applyFunction(unaryChild(), order,
                [exponent](const Interval& interval) { return (signPowerBounds(interval, exponent)); }));
        }

        case E_NonlinearExpressionTypes::Sinh:
            return (applyFunction(unaryChild(), order, hyperbolicSineBounds));

        case E_NonlinearExpressionTypes::Cosh:
            return (applyFunction(unaryChild(), order, hyperbolicCosineBounds));

        case E_NonlinearExpressionTypes::Tanh:
            return (applyFunction(unaryChild(), order, hyperbolicTangentBounds));

        case E_NonlinearExpressionTypes::Divide:
        {
            const auto& divide = static_cast<const ExpressionBinary&>(expression);
            auto numerator = calculateChild(divide.firstChild, box, order, memo);
            auto denominator = calculateChild(divide.secondChild, box, order, memo);
            return (product(numerator, invert(denominator, order), order));
        }

        case E_NonlinearExpressionTypes::Power:
        {
            const auto& powerExpression = static_cast<const ExpressionBinary&>(expression);
            auto base = calculateChild(powerExpression.firstChild, box, order, memo);

            if(powerExpression.secondChild->getType() == E_NonlinearExpressionTypes::Constant)
            {
                return (
                    power(base, static_cast<const ExpressionConstant&>(*powerExpression.secondChild).constant, order));
            }

            // u^v = exp(v log(u)), which is only defined for a positive base here
            auto exponent = calculateChild(powerExpression.secondChild, box, order, memo);
            auto logarithm = applyFunction(std::move(base), order, logarithmBounds);
            return (applyFunction(product(exponent, logarithm, order), order, exponentialBounds));
        }

        case E_NonlinearExpressionTypes::Sum:
        {
            Derivatives sum;

            for(auto& C : static_cast<const ExpressionGeneral&>(expression).children)
                accumulate(sum, calculateChild(C, box, order, memo), order);

            return (sum);
        }

        case E_NonlinearExpressionTypes::Product:
        {
            std::vector<Derivatives> factors;
            collectFactors(static_cast<const ExpressionGeneral&>(expression), box, order, memo, factors);
            return (product(std::move(factors), order));
        }
        }

        throw std::logic_error("Interval derivatives are not implemented for the expression type.");
    }

    // Removes the derivatives that are exactly zero and decides whether the bounds are finite
    void finish(Derivatives& derivatives, int order)
    {
        if(order < 1)
            derivatives.gradient.clear();

        if(order < 2)
            derivatives.hessian.clear();

        for(auto element = derivatives.gradient.begin(); element != derivatives.gradient.end();)
            element = isExactlyZero(element->second) ? derivatives.gradient.erase(element) : std::next(element);

        for(auto element = derivatives.hessian.begin(); element != derivatives.hessian.end();)
            element = isExactlyZero(element->second) ? derivatives.hessian.erase(element) : std::next(element);

        derivatives.isBounded = !isUnbounded(derivatives.value);

        for(auto& [V, G] : derivatives.gradient)
            derivatives.isBounded = derivatives.isBounded && !isUnbounded(G);

        for(auto& [K, H] : derivatives.hessian)
            derivatives.isBounded = derivatives.isBounded && !isUnbounded(H);
    }

} // namespace

void checkIntervalDerivativeOrder(int order)
{
    if(order < 0 || order > 2)
        throw std::invalid_argument("The order of the interval derivatives must be 0, 1 or 2.");
}

static IntervalDerivatives calculateUnfinished(const LinearTerms& terms, const IntervalVector& box, int order)
{
    Derivatives sum;

    for(auto& T : terms)
        accumulate(sum, scale(variable(T->variable, box, order), T->coefficient, order), order);

    return (sum);
}

static IntervalDerivatives calculateUnfinished(const QuadraticTerms& terms, const IntervalVector& box, int order)
{
    Derivatives sum;

    for(auto& T : terms)
    {
        auto first = variable(T->firstVariable, box, order);

        // The square of a variable is tighter than the product of the variable with itself
        auto term = (T->firstVariable == T->secondVariable)
            ? power(first, 2.0, order)
            : product(first, variable(T->secondVariable, box, order), order);

        accumulate(sum, scale(std::move(term), T->coefficient, order), order);
    }

    return (sum);
}

static IntervalDerivatives calculateUnfinished(const MonomialTerms& terms, const IntervalVector& box, int order)
{
    Derivatives sum;

    for(auto& T : terms)
    {
        // A variable that is repeated in the product is a power of it, e.g., x*x*y = x^2 y, which is tighter
        std::map<VariablePtr, int, VariableIndexComparator> powers;

        for(auto& V : T->variables)
            powers[V]++;

        std::vector<Derivatives> factors;

        for(auto& [V, P] : powers)
            factors.push_back(power(variable(V, box, order), P, order));

        accumulate(sum, scale(product(std::move(factors), order), T->coefficient, order), order);
    }

    return (sum);
}

static IntervalDerivatives calculateUnfinished(const SignomialTerms& terms, const IntervalVector& box, int order)
{
    Derivatives sum;

    for(auto& T : terms)
    {
        // The elements are not combined, since each power of a variable is only defined on its own domain
        std::vector<Derivatives> factors;

        for(auto& E : T->elements)
            factors.push_back(power(variable(E->variable, box, order), E->power, order));

        accumulate(sum, scale(product(std::move(factors), order), T->coefficient, order), order);
    }

    return (sum);
}

static IntervalDerivatives calculateUnfinished(
    const NonlinearExpressionPtr& expression, const IntervalVector& box, int order)
{
    Memo memo;
    countUses(*expression, memo);
    return (calculate(*expression, box, order, memo));
}

// The public functions check the order and return the finished derivatives
template <typename Function>
static IntervalDerivatives calculateFinished(const Function& function, const IntervalVector& box, int order)
{
    checkIntervalDerivativeOrder(order);

    auto derivatives = calculateUnfinished(function, box, order);
    finish(derivatives, order);
    return (derivatives);
}

IntervalDerivatives calculateIntervalDerivatives(const LinearTerms& terms, const IntervalVector& box, int order)
{
    return (calculateFinished(terms, box, order));
}

IntervalDerivatives calculateIntervalDerivatives(const QuadraticTerms& terms, const IntervalVector& box, int order)
{
    return (calculateFinished(terms, box, order));
}

IntervalDerivatives calculateIntervalDerivatives(const MonomialTerms& terms, const IntervalVector& box, int order)
{
    return (calculateFinished(terms, box, order));
}

IntervalDerivatives calculateIntervalDerivatives(const SignomialTerms& terms, const IntervalVector& box, int order)
{
    return (calculateFinished(terms, box, order));
}

IntervalDerivatives calculateIntervalDerivatives(
    const NonlinearExpressionPtr& expression, const IntervalVector& box, int order)
{
    return (calculateFinished(expression, box, order));
}

void addIntervalDerivatives(IntervalDerivatives& sum, const IntervalDerivatives& part, int order)
{
    checkIntervalDerivativeOrder(order);

    accumulate(sum, part, order);
    finish(sum, order);
}

} // namespace SHOT
