/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <memory>
#include <string>

#include "Environment.h"
#include "Enums.h"
#include "EventHandler.h"
#include "Structs.h"

#include "ModelingSystem/IModelingSystem.h"
#include "SolutionStrategy/ISolutionStrategy.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

namespace SHOT
{
class DllExport Solver
{
private:
    std::unique_ptr<ISolutionStrategy> solutionStrategy;

    void initializeSettings();
    void verifySettings();

    void setConvexityBasedSettingsPreReformulation();
    void setConvexityBasedSettings();

    void initializeDebugMode();

    bool selectStrategy();

    bool isProblemInitialized = false;
    bool isProblemSolved = false;

    EnvironmentPtr env;

public:
    Solver();
    Solver(std::shared_ptr<spdlog::sinks::sink> consoleSink);
    Solver(EnvironmentPtr environment);
    ~Solver();

    EnvironmentPtr getEnvironment();

    bool setOptionsFromFile(std::string fileName);
    bool setOptionsFromString(std::string options);
    bool setOptionsFromOSoL(std::string options);

    bool setLogFile(std::string filename);

    bool setProblem(std::string fileName);
    bool setProblem(ProblemPtr problem, ProblemPtr reformulatedProblem, ModelingSystemPtr modelingSystem = nullptr);
    bool setProblem(ProblemPtr problem, ModelingSystemPtr modelingSystem = nullptr)
    {
        return setProblem(problem, nullptr, modelingSystem);
    };

    ProblemPtr getOriginalProblem() { return (env->problem); };
    ProblemPtr getReformulatedProblem() { return (env->reformulatedProblem); };

    bool solveProblem();

    void finalizeSolution();

    template <typename Callback> inline void registerCallback(const E_EventType& event, Callback&& callback)
    {
        env->events->registerCallback(event, callback);
    }

    std::string getOptionsOSoL();
    std::string getOptions();

    std::string getResultsOSrL();
    std::string getResultsTrace();
    std::string getResultsSol();

    void updateSetting(std::string name, std::string category, int value);
    void updateSetting(std::string name, std::string category, std::string value);
    void updateSetting(std::string name, std::string category, double value);
    void updateSetting(std::string name, std::string category, bool value);

    VectorString getSettingIdentifiers(E_SettingType type);

    double getCurrentDualBound();
    double getPrimalBound();
    double getAbsoluteObjectiveGap();
    double getRelativeObjectiveGap();

    bool hasPrimalSolution();
    PrimalSolution getPrimalSolution();
    std::vector<PrimalSolution> getPrimalSolutions();

    E_TerminationReason getTerminationReason();
    E_ModelReturnStatus getModelReturnStatus();
};
} // namespace SHOT