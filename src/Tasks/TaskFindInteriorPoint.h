/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include <memory>
#include <optional>
#include <vector>

namespace SHOT
{

class INLPSolver;

class TaskFindInteriorPoint : public TaskBase
{
public:
    TaskFindInteriorPoint(EnvironmentPtr envPtr);
    ~TaskFindInteriorPoint() override;

    void run() override;
    std::string getType() override;

private:
    std::vector<std::unique_ptr<INLPSolver>> NLPSolvers;

    VectorString variableNames;

    // A usable interior point found by moving from a candidate toward the center of the variable box
    std::shared_ptr<InteriorPoint> retreatToUsableInteriorPoint(const VectorDouble& candidatePoint);

    // Calls the InteriorPointSearch callbacks with the current interior points; returns the points they replace them
    // with, or nullopt if they keep them
    std::optional<std::vector<VectorDouble>> invokeInteriorPointCallbacks();

    // Replaces the interior points with those given by a callback, keeping only those that are interior points
    void setCallbackInteriorPoints(const std::vector<VectorDouble>& points);
};
} // namespace SHOT