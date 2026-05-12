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
#include <vector>

#include "Environment.h"
#include "Enums.h"
#include "EventHandler.h"
#include "Settings.h"
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

    void finalizeSolution();

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

    std::string getSettingsAsMarkup();

    bool setLogFile(std::string filename);
    void updateLogLevels();

    bool setProblem(std::string fileName);
    bool setProblem(ProblemPtr problem, ProblemPtr reformulatedProblem, ModelingSystemPtr modelingSystem = nullptr);
    bool setProblem(ProblemPtr problem, ModelingSystemPtr modelingSystem = nullptr)
    {
        return setProblem(problem, nullptr, modelingSystem);
    };

    ProblemPtr getOriginalProblem() { return (env->problem); };
    ProblemPtr getReformulatedProblem() { return (env->reformulatedProblem); };

    bool solveProblem();

    void outputSolverHeader();
    void outputOptionsReport();
    void outputProblemInstanceReport();
    void outputSolutionReport();

    /**
     * @brief Callback registration method
     *
     * This method automatically detects whether the callback is a notification callback
     * or a data provider based on its return type:
     * - Returns void: Notification callback
     * - Returns a value: Data provider
     *
     * Examples:
     * // Data provider for dual bound
     * solver.registerCallback(E_EventType::ExternalDualBound, []() {
     *     return computeDualBound(); // Returns double -> data provider
     * });
     *
     * // User termination check
     * solver.registerCallback(E_EventType::UserTerminationCheck, []() {
     *     return shouldTerminate(); // Returns bool -> data provider
     * });
     *
     * // Notification callback
     * solver.registerCallback(E_EventType::NewPrimalSolution, [](std::any solution) {
     *     processSolution(solution); // Returns void -> notification
     * });
     *
     * @tparam Callback The callback function type
     * @param event The event type to register for
     * @param callback The callback function
     */
    template <typename Callback> inline void registerCallback(const E_EventType& event, Callback&& callback)
    {
        env->events->registerCallback(event, std::forward<Callback>(callback));
    }

    std::string getOptionsOSoL();
    std::string getOptions();

    std::string getResultsOSrL();
    std::string getResultsTrace();
    std::string getResultsSol();

    void updateSetting(std::string settingName, int value)
    { env->settings->updateSetting(settingName, value, E_SettingPriority::UserAPI); }
    void updateSetting(std::string settingName, std::string value)
    { env->settings->updateSetting<std::string>(settingName, value, E_SettingPriority::UserAPI); }
    void updateSetting(std::string settingName, double value)
    { env->settings->updateSetting(settingName, value, E_SettingPriority::UserAPI); }
    void updateSetting(std::string settingName, bool value)
    { env->settings->updateSetting(settingName, value, E_SettingPriority::UserAPI); }

    void updateSetting(std::string settingName, int value, E_SettingPriority priority)
    { env->settings->updateSetting(settingName, value, priority); }
    void updateSetting(std::string settingName, std::string value, E_SettingPriority priority)
    { env->settings->updateSetting<std::string>(settingName, value, priority); }
    void updateSetting(std::string settingName, double value, E_SettingPriority priority)
    { env->settings->updateSetting(settingName, value, priority); }
    void updateSetting(std::string settingName, bool value, E_SettingPriority priority)
    { env->settings->updateSetting(settingName, value, priority); }

    template <typename T> T getSetting(std::string settingName)
    {
        return (env->settings->getSetting<T>(settingName));
    }

    E_SettingPriority getSettingPriority(std::string settingName)
    {
        return (env->settings->getSettingPriority(settingName));
    }

    VectorString getSettingIdentifiers(E_SettingType type);

    double getCurrentDualBound();
    double getPrimalBound();
    double getAbsoluteObjectiveGap();
    double getRelativeObjectiveGap();

    bool hasPrimalSolution();
    PrimalSolution getPrimalSolution();
    std::vector<PrimalSolution> getPrimalSolutions();

    SolutionStatistics getSolutionStatistics() { return env->solutionStatistics; };

    E_TerminationReason getTerminationReason();
    E_ModelReturnStatus getModelReturnStatus();

    // Static methods to query available solvers and modeling systems
    static std::vector<ES_ModelingSystem> getSupportedModelingSystems();
    static std::vector<ES_MIPSolver> getSupportedMIPSolvers();
    static std::vector<ES_PrimalNLPSolver> getSupportedNLPSolvers();

    // Static methods to check availability of specific components
    static bool hasModelingSystem(ES_ModelingSystem format);
    static bool hasMIPSolver(ES_MIPSolver solver);
    static bool hasNLPSolver(ES_PrimalNLPSolver solver);
};
} // namespace SHOT