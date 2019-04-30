/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Environment.h"

namespace SHOT
{
class IMIPSolutionLimitStrategy
{
public:
    virtual ~IMIPSolutionLimitStrategy() = default;

    virtual bool updateLimit() = 0;

    virtual int getNewLimit() = 0;

    virtual int getInitialLimit() = 0;

    EnvironmentPtr env;

protected:
};
} // namespace SHOT