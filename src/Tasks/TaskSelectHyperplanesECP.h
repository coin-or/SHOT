/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

#include "../Structs.h"

namespace SHOT
{
class TaskSelectHyperplanesECP : public TaskBase
{
public:
    TaskSelectHyperplanesECP(EnvironmentPtr envPtr);
    ~TaskSelectHyperplanesECP() override;

    void run() override;
    virtual void run(std::vector<SolutionPoint> solPoints);

    std::string getType() override;

private:
    // The largest number of values in the points of the hyperplanes added for convex constraints in an iteration,
    // above which at most Dual.HyperplaneCuts.MaxPerIteration are added (800 MB)
    static constexpr size_t maximumNumberOfHyperplanePointValues = 100000000;
};
} // namespace SHOT