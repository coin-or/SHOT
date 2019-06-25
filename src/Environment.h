/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <memory>

#include "Structs.h"

namespace SHOT
{

class IRootsearchMethod;

class DllExport Environment
{
public:
    inline Environment() = default;
    inline ~Environment() = default;

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
    EventHandlerPtr events;

    std::shared_ptr<IRootsearchMethod> rootsearchMethod;

    SolutionStatistics solutionStatistics;

private:
};

using EnvironmentPtr = std::shared_ptr<Environment>;
} // namespace SHOT
