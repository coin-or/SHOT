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
#include "ModelingSystem/ModelingSystemOS.h"

#include "SolutionStrategy/ISolutionStrategy.h"
#include "SolutionStrategy/SolutionStrategySingleTree.h"
#include "SolutionStrategy/SolutionStrategyMultiTree.h"
#include "SolutionStrategy/SolutionStrategyMIQCQP.h"
#include "SolutionStrategy/SolutionStrategyNLP.h"

#include "TaskHandler.h"

#ifdef HAS_GAMS
#include "ModelingSystem/ModelingSystemGAMS.h"
#endif

namespace SHOT
{
class SHOTSolver
{
private:
    std::unique_ptr<ISolutionStrategy> solutionStrategy;

    void initializeSettings();
    void verifySettings();

    void initializeDebugMode();

    bool isProblemInitialized = false;
    bool isProblemSolved = false;

    EnvironmentPtr env;

public:
    SHOTSolver(std::shared_ptr<spdlog::sinks::sink> consoleSink = NULL);
    SHOTSolver(EnvironmentPtr environment);
    ~SHOTSolver();

    inline EnvironmentPtr getEnvironment() { return env; };

    bool setOptions(std::string fileName);

    bool setProblem(std::string fileName);
    bool setProblem(ProblemPtr problem, ModelingSystemPtr modelingSystem);

    bool selectStrategy();

    bool solveProblem();

    template <typename Callback> inline void registerCallback(const E_EventType& event, Callback&& callback)
    {
        env->events->registerCallback(event, callback);
    }

    // extern template void registerCallback(const E_EventType& event, std::function&& callback);

    std::string getOSoL();
    std::string getGAMSOptFile();

    std::string getResultsOSrL();
    std::string getResultsTrace();

    void updateSetting(std::string name, std::string category, std::string value);
    void updateSetting(std::string name, std::string category, int value);
    void updateSetting(std::string name, std::string category, bool value);
    void updateSetting(std::string name, std::string category, double value);

    double getDualBound();
    double getPrimalBound();
    double getAbsoluteObjectiveGap();
    double getRelativeObjectiveGap();

    int getNumberOfPrimalSolutions();
    PrimalSolution getPrimalSolution();
    std::vector<PrimalSolution> getPrimalSolutions();

    E_TerminationReason getTerminationReason();
};
} // namespace SHOT