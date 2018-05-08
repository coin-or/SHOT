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

class ProcessInfo;
class Settings;
class IMIPSolver;
class Output;

typedef std::shared_ptr<ProcessInfo> ProcessPtr;
typedef std::shared_ptr<Settings> SettingsPtr;
typedef std::shared_ptr<IMIPSolver> MIPSolverPtr;
typedef std::shared_ptr<Output> OutputPtr;

class Environment
{
  public:
    Environment();
    ~Environment();

    ProcessPtr process;
    SettingsPtr settings;
    MIPSolverPtr dualSolver;
    OutputPtr output;

  private:
};

typedef std::shared_ptr<Environment> EnvironmentPtr;