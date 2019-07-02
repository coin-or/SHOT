/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"

namespace SHOT
{
class TaskUpdateInteriorPoint : public TaskBase
{
public:
    TaskUpdateInteriorPoint(EnvironmentPtr envPtr);
    ~TaskUpdateInteriorPoint() override;
    void run() override;
    std::string getType() override;

private:
};
} // namespace SHOT