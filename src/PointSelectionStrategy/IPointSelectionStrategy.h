/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Structs.h"

namespace SHOT
{
class IPointSelectionStrategy
{
  public:
    IPointSelectionStrategy();
    ~IPointSelectionStrategy();

    virtual VectorDouble selectPoint() = 0;
    virtual std::vector<VectorDouble> selectPoints(int maxNumPts) = 0;
};
} // namespace SHOT