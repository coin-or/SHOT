/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "SHOTSettings.h"
#include "../ProcessInfo.h"

class PointSelectionStrategyBase
{
  public:
    PointSelectionStrategyBase();
    ~PointSelectionStrategyBase();

    DoubleVector selectPoint();
    virtual std::vector<DoubleVector> selectPoints(int maxNumPts) = 0;

  protected:
};
