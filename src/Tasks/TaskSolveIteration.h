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

#include "../MIPSolver/IMIPSolver.h"

class TaskSolveIteration : public TaskBase
{
  public:
	TaskSolveIteration(IMIPSolver *MIPSolver);
	virtual ~TaskSolveIteration();

	virtual void run();
	virtual std::string getType();

  private:
	IMIPSolver *MIPSolver;
};
