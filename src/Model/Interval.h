/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "interval.hpp"

namespace SHOT
{

// The interval type of SHOT, currently MC++'s. Code outside this header should only use the construction from one or
// two values, l() and u(), the arithmetic operators and the elementary functions, so that the implementation can be
// replaced. The bounds are not rounded outwards, so an interval is not a verified enclosure of the floating-point
// calculations.
using Interval = mc::Interval;
using IntervalVector = std::vector<Interval>;

// Thrown when an interval cannot be calculated, e.g., the logarithm of an interval with only negative values
using IntervalError = mc::Interval::Exceptions;

// The interval functions of the expressions restrict an argument to the domain of the function, e.g., the logarithm
// to positive values, so an argument entirely outside the domain gives bounds that are not numbers or reversed
inline bool isValidInterval(const Interval& interval)
{
    return (!std::isnan(interval.l()) && !std::isnan(interval.u()) && interval.l() <= interval.u());
}

// The description of an interval error, without the name of the implementation that MC++ starts it with
inline std::string getIntervalErrorMessage(IntervalError& error)
{
    std::string message = error.what();

    if(auto position = message.find_first_not_of(" \t", message.find('\t')); position != std::string::npos)
        message = message.substr(position);

    return (message);
}

} // namespace SHOT
