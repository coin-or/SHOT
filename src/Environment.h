/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Shared.h"

namespace SHOT
{

class ILinesearchMethod;

class Environment
{
public:
    inline Environment(){};
    inline ~Environment(){};

    ResultsPtr results;
    SettingsPtr settings;
    ProblemPtr problem;
    ProblemPtr reformulatedProblem;
    ModelingSystemPtr modelingSystem;
    DualSolverPtr dualSolver;
    PrimalSolverPtr primalSolver;
    OutputPtr output;
    ReportPtr report;
    TaskHandlerPtr tasks;
    TimingPtr timing;

    std::shared_ptr<ILinesearchMethod> rootsearchMethod;

    SolutionStatistics solutionStatistics;

    using TCallback = std::function<void(EnvironmentPtr)>;
    using CallbackVector = std::vector<TCallback>;

private:
};

typedef std::shared_ptr<Environment> EnvironmentPtr;
} // namespace SHOT

#include "TaskHandler.h"