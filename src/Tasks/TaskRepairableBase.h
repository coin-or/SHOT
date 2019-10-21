/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskBase.h"

namespace SHOT
{
class TaskRepairableBase : public TaskBase
{
public:
    ~TaskRepairableBase() override;

    virtual void repair();
    std::string getType() override;

private:
};
} // namespace SHOT