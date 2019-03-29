/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "Shared.h"

#include "ModelingSystem/IModelingSystem.h"
#include "SolutionStrategy/ISolutionStrategy.h"

namespace SHOT
{
class Solver
{
private:
    std::unique_ptr<ISolutionStrategy> solutionStrategy;

    void initializeSettings();
    void verifySettings();

    void initializeDebugMode();

    bool selectStrategy();

    bool isProblemInitialized = false;
    bool isProblemSolved = false;

    EnvironmentPtr env;

public:
    Solver(std::shared_ptr<spdlog::sinks::sink> consoleSink = NULL);
    Solver(EnvironmentPtr environment);
    ~Solver();

    inline EnvironmentPtr getEnvironment() { return env; };

    bool setOptionsFromFile(std::string fileName);
    bool setOptionsFromString(std::string options);
    bool setOptionsFromOSoL(std::string options);

    bool setProblem(std::string fileName);
    bool setProblem(ProblemPtr problem, ModelingSystemPtr modelingSystem);

    bool solveProblem();

    template <typename Callback> inline void registerCallback(const E_EventType& event, Callback&& callback)
    {
        env->events->registerCallback(event, callback);
    }

    std::string getOptionsOSoL();
    std::string getOptions();

    std::string getResultsOSrL();
    std::string getResultsTrace();

    template <typename T> void updateSetting(std::string name, std::string category, T value);

    double getDualBound();
    double getPrimalBound();
    double getAbsoluteObjectiveGap();
    double getRelativeObjectiveGap();

    PrimalSolution getPrimalSolution();
    std::vector<PrimalSolution> getPrimalSolutions();

    E_TerminationReason getTerminationReason();
    E_ModelReturnStatus getModelReturnStatus();
};
} // namespace SHOT