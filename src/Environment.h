/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Enums.h"
#include "Structs.h"

namespace SHOT
{
class Environment
{
  public:
    Environment();
    ~Environment();

    ProcessPtr process;
    SettingsPtr settings;
    ModelPtr model;
    MIPSolverPtr dualSolver;
    OutputPtr output;
    ReportPtr report;
    TaskHandlerPtr tasks;

    SolutionStatistics solutionStatistics;

  private:
};

typedef std::shared_ptr<Environment> EnvironmentPtr;
} // namespace SHOT

#include "TaskHandler.h"