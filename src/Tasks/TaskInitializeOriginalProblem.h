/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "TaskBase.h"
#include "../ProcessInfo.h"
#include "../UtilityFunctions.h"
#include "../OptProblems/OptProblem.h"
#include "../OptProblems/OptProblemOriginalNonlinearObjective.h"
#include "../OptProblems/OptProblemOriginalQuadraticObjective.h"
#include "../OptProblems/OptProblemOriginalLinearObjective.h"
#include <OSInstance.h>
#include "../MIPSolver/IMIPSolver.h"

class TaskInitializeOriginalProblem : public TaskBase
{
  public:
    TaskInitializeOriginalProblem(OSInstance *originalInstance);
    virtual ~TaskInitializeOriginalProblem();

    void run();
    virtual std::string getType();

  private:
    OptProblem *problem;
    OSInstance *instance;
};
