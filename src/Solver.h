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
#include "Callback.h"
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
    bool isSolving = false;

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
     * @brief Registers a callback for one or several locations in the solution process
     *
     * The callback is given a context with the state of the solver and the actions that are available at the
     * location. The context is of the class for the location, which CallbackContext::as<T>() returns:
     *
     * solver.registerCallback(E_CallbackLocation::PrimalCandidateCheck | E_CallbackLocation::TerminationCheck,
     *     [](CallbackContext& context) {
     *         auto candidate = context.as<PrimalCandidateCheckContext>();
     *
     *         if(candidate != nullptr && reject(candidate->getPoint()))
     *             candidate->rejectCandidate();
     *
     *         if(context.getElapsedTime() > 60)
     *             context.terminate();
     *     });
     *
     * Callbacks can be registered before or after setProblem(), but not while solveProblem() runs. If a callback
     * throws, SHOT terminates and solveProblem() rethrows the exception. See docs/Callbacks.md for the locations.
     *
     * @param locations The locations to call the callback at, combined with operator|
     * @param callback The callback
     * @return A handle for removeCallback()
     */
    int registerCallback(E_CallbackLocation locations, CallbackFunction callback);

    /**
     * @brief Registers a callback for the location of a context class, e.g., PrimalCandidateCheckContext
     *
     * solver.registerCallback<TerminationCheckContext>([](TerminationCheckContext& context) { context.terminate(); });
     */
    template <typename T> int registerCallback(std::function<void(T&)> callback)
    {
        return (registerCallback(
            T::Location, [callback = std::move(callback)](CallbackContext& context) { callback(*context.as<T>()); }));
    }

    /// Removes a registered callback; returns false if there is no callback with the handle
    bool removeCallback(int handle);

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
    double getGlobalDualBound();
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