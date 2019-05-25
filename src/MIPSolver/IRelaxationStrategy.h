/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "../Enums.h"

namespace SHOT
{
class IRelaxationStrategy
{
public:
    virtual ~IRelaxationStrategy() = default;

    virtual void executeStrategy() = 0;

    virtual void setActive() = 0;
    virtual void setInactive() = 0;

    virtual void setInitial() = 0;
};
} // namespace SHOT