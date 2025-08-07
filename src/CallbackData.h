/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "Enums.h"
#include "Structs.h"
#include "Utilities.h"

#include <string>

namespace SHOT
{

/**
 * @brief Data structure for dual bound callback events
 *
 * Contains all relevant information that might be useful for external
 * dual bound providers to make informed decisions.
 */
struct DualBoundCallbackData
{
    bool isMinimization; ///< True if minimization problem, false if maximization problem
    double currentDualBound; ///< Current dual bound value
    double currentPrimalBound; ///< Current primal bound value
    double relativeGap; ///< Current relative gap
    double absoluteGap; ///< Current absolute gap
    int iterationNumber; ///< Current iteration number
    SolutionStatistics solutionStatistics; ///< Statistics about the current solution state

    DualBoundCallbackData(bool minimize, double dualBound, double primalBound, int iteration, double relGap,
        double absGap, SolutionStatistics stats)
        : isMinimization(minimize)
        , currentDualBound(dualBound)
        , currentPrimalBound(primalBound)
        , iterationNumber(iteration)
        , relativeGap(relGap)
        , absoluteGap(absGap)
        , solutionStatistics(stats)
    {
    }
};

/**
 * @brief Data structure for user termination check callback events
 *
 * Provides context information for user termination decisions.
 */
struct TerminationCallbackData
{
    int iterationNumber; ///< Current iteration number
    double currentDualBound; ///< Current dual bound
    double currentPrimalBound; ///< Current primal bound
    double relativeGap; ///< Current relative gap
    double absoluteGap; ///< Current absolute gap
    double timeElapsed; ///< Time elapsed since start
    SolutionStatistics solutionStatistics; ///< Statistics about the current solution state

    TerminationCallbackData(int iteration, double time, double dualBound, double primalBound, double relGap,
        double absGap, SolutionStatistics stats)
        : iterationNumber(iteration)
        , currentDualBound(dualBound)
        , currentPrimalBound(primalBound)
        , relativeGap(relGap)
        , absoluteGap(absGap)
        , timeElapsed(time)
        , solutionStatistics(stats)
    {
    }
};

} // namespace SHOT
