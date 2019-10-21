/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "RelaxationStrategyBase.h"
#include "../Environment.h"
#include "../Enums.h"

namespace SHOT
{
class RelaxationStrategyStandard : public IRelaxationStrategy, public RelaxationStrategyBase
{
public:
    RelaxationStrategyStandard(EnvironmentPtr envPtr);
    ~RelaxationStrategyStandard() override;

    void executeStrategy() override;

    void setActive() override;

    void setInactive() override;

    void setInitial() override;

private:
    bool isIterationLimitReached();
    bool isTimeLimitReached();
    bool isLPStepFinished();
    bool isObjectiveStagnant();

    bool LPFinished;
};

} // namespace SHOT