/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Settings.h"
#include "../src/Solver.h"
#include "../src/Utilities.h"
#include "../src/Results.h"
#include "../src/Model/Problem.h"

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
#endif

#include <iostream>

using namespace SHOT;

bool SettingsTestOptions(bool useOSiL);
bool TestPriorityOrdering();
bool TestHistoryRetrieval();
bool TestGetSettingsAtPriority();
bool TestIsSettingAtDefault();
bool TestReadSettingsFromStringPriority();
bool TestSolverWrapperPriorityRouting();
bool TestUserOverridesConvexityRecommendation();
bool TestConditionalValues();
bool TestConditionalValueRegistration();
bool TestRecommendedSettingsForProblems();
bool TestSettingsOfAllBackends();

int SettingsTest(int argc, char* argv[])
{
    int defaultchoice = 1;

    int choice = defaultchoice;

    if(argc > 1)
    {
        if(sscanf(argv[1], "%d", &choice) != 1)
        {
            printf("Couldn't parse that input as a number\n");
            return -1;
        }
    }

    bool passed = true;

    switch(choice)
    {
    case 1:
        std::cout << "Starting test to read and write OSoL files:" << std::endl;
        passed = SettingsTestOptions(true);
        std::cout << "Finished test to read and write OSoL files." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to read and write opt files:" << std::endl;
        passed = SettingsTestOptions(false);
        std::cout << "Finished test to read and write opt files." << std::endl;
        break;
    case 3:
        passed = TestPriorityOrdering();
        break;
    case 4:
        passed = TestHistoryRetrieval();
        break;
    case 5:
        passed = TestGetSettingsAtPriority();
        break;
    case 6:
        passed = TestIsSettingAtDefault();
        break;
    case 7:
        passed = TestReadSettingsFromStringPriority();
        break;
    case 8:
        passed = TestSolverWrapperPriorityRouting();
        break;
    case 9:
        passed = TestUserOverridesConvexityRecommendation();
        break;
    case 10:
        passed = TestConditionalValues();
        break;
    case 11:
        passed = TestConditionalValueRegistration();
        break;
    case 12:
        passed = TestRecommendedSettingsForProblems();
        break;
    case 13:
        passed = TestSettingsOfAllBackends();
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
        break;
    }

    if(passed)
        return 0;
    else
        return -1;
}

bool TestPriorityOrdering()
{
    bool passed = true;
    std::cout << "Starting priority ordering test:" << std::endl;

    // Higher priority wins; lower-priority write is stored in history but does not change active value.
    auto settings = std::make_shared<Settings>(std::make_shared<Output>());
    settings->createSetting("Test.TestInt", 0, "A test integer setting", 0.0, 1000.0);

    settings->updateSetting("Test.TestInt", 42, E_SettingPriority::SolverInternal);
    if(settings->getSetting<int>("Test.TestInt") != 42)
    {
        std::cout << "  FAIL: SolverInternal write did not take effect." << std::endl;
        passed = false;
    }

    // A lower-priority write should NOT change the active value
    settings->updateSetting("Test.TestInt", 7, E_SettingPriority::OptionsFile);
    if(settings->getSetting<int>("Test.TestInt") != 42)
    {
        std::cout << "  FAIL: Lower-priority (OptionsFile) write incorrectly overrode SolverInternal." << std::endl;
        passed = false;
    }

    // But it should be stored in history
    if(!settings->hasSettingAtPriority("Test.TestInt", E_SettingPriority::OptionsFile))
    {
        std::cout << "  FAIL: OptionsFile value not found in history after lower-priority write." << std::endl;
        passed = false;
    }

    // Active priority should still be SolverInternal
    if(settings->getSettingPriority("Test.TestInt") != E_SettingPriority::SolverInternal)
    {
        std::cout << "  FAIL: Active priority should be SolverInternal." << std::endl;
        passed = false;
    }

    if(passed)
        std::cout << "  PASS: Priority ordering works correctly." << std::endl;
    std::cout << "Finished priority ordering test." << std::endl;
    return passed;
}

bool TestHistoryRetrieval()
{
    bool passed = true;
    std::cout << "Starting history retrieval test:" << std::endl;

    auto settings = std::make_shared<Settings>(std::make_shared<Output>());
    settings->createSetting("Test.HistDouble", 1.0, "A test double setting", 0.0, 100.0);

    settings->updateSetting("Test.HistDouble", 5.0, E_SettingPriority::OptionsFile);
    settings->updateSetting("Test.HistDouble", 9.0, E_SettingPriority::SolverCompatibility);
    settings->updateSetting("Test.HistDouble", 12.5, E_SettingPriority::SolverInternal);

    // getSettingAtPriority
    double atOptions = settings->getSettingAtPriority<double>("Test.HistDouble", E_SettingPriority::OptionsFile);
    if(atOptions != 5.0)
    {
        std::cout << "  FAIL: getSettingAtPriority<double> at OptionsFile returned " << atOptions
                  << " (expected 5.0)." << std::endl;
        passed = false;
    }

    double atSolverCompat
        = settings->getSettingAtPriority<double>("Test.HistDouble", E_SettingPriority::SolverCompatibility);
    if(atSolverCompat != 9.0)
    {
        std::cout << "  FAIL: getSettingAtPriority<double> at SolverCompatibility returned " << atSolverCompat
                  << " (expected 9.0)." << std::endl;
        passed = false;
    }

    // hasSettingAtPriority
    if(!settings->hasSettingAtPriority("Test.HistDouble", E_SettingPriority::Default))
    {
        std::cout << "  FAIL: hasSettingAtPriority should return true for Default (initial value)." << std::endl;
        passed = false;
    }
    if(settings->hasSettingAtPriority("Test.HistDouble", E_SettingPriority::UserAPI))
    {
        std::cout << "  FAIL: hasSettingAtPriority should return false for UserAPI (never written)." << std::endl;
        passed = false;
    }

    // getSettingPriorityHistory — should contain Default(0), OptionsFile(20), SolverInternal(40), SolverCompatibility(50)
    auto history = settings->getSettingPriorityHistory("Test.HistDouble");
    if(history.size() != 4)
    {
        std::cout << "  FAIL: getSettingPriorityHistory returned " << history.size() << " entries (expected 4)."
                  << std::endl;
        passed = false;
    }

    if(passed)
        std::cout << "  PASS: History retrieval methods work correctly." << std::endl;
    std::cout << "Finished history retrieval test." << std::endl;
    return passed;
}

bool TestGetSettingsAtPriority()
{
    bool passed = true;
    std::cout << "Starting getSettingsAtPriority test:" << std::endl;

    auto settings = std::make_shared<Settings>(std::make_shared<Output>());
    settings->createSetting("Cat.A", 1, "Setting A", 0.0, 100.0);
    settings->createSetting("Cat.B", 2, "Setting B", 0.0, 100.0);
    settings->createSetting("Cat.C", 3, "Setting C", 0.0, 100.0);

    settings->updateSetting("Cat.A", 10, E_SettingPriority::UserAPI);
    settings->updateSetting("Cat.B", 20, E_SettingPriority::UserAPI);
    // C is left at default

    auto userSettings = settings->getSettingsAtPriority(E_SettingPriority::UserAPI);
    if(userSettings.size() != 2)
    {
        std::cout << "  FAIL: getSettingsAtPriority(UserAPI) returned " << userSettings.size() << " (expected 2)."
                  << std::endl;
        passed = false;
    }

    auto defaultSettings = settings->getSettingsAtPriority(E_SettingPriority::Default);
    if(defaultSettings.size() != 1)
    {
        std::cout << "  FAIL: getSettingsAtPriority(Default) returned " << defaultSettings.size() << " (expected 1)."
                  << std::endl;
        passed = false;
    }

    if(passed)
        std::cout << "  PASS: getSettingsAtPriority works correctly." << std::endl;
    std::cout << "Finished getSettingsAtPriority test." << std::endl;
    return passed;
}

bool TestIsSettingAtDefault()
{
    bool passed = true;
    std::cout << "Starting isSettingAtDefault and getChangedSettings test:" << std::endl;

    auto settings = std::make_shared<Settings>(std::make_shared<Output>());
    settings->createSetting("Cat.X", 5, "Setting X", 0.0, 100.0);
    settings->createSetting("Cat.Y", true, "Setting Y");

    if(!settings->isSettingAtDefault("Cat.X"))
    {
        std::cout << "  FAIL: isSettingAtDefault should be true before any update." << std::endl;
        passed = false;
    }

    settings->updateSetting("Cat.X", 99, E_SettingPriority::UserAPI);

    if(settings->isSettingAtDefault("Cat.X"))
    {
        std::cout << "  FAIL: isSettingAtDefault should be false after UserAPI update." << std::endl;
        passed = false;
    }

    auto changed = settings->getChangedSettings();
    bool foundX = false;
    for(auto& s : changed)
        if(s.find("Cat.X") != std::string::npos)
            foundX = true;
    if(!foundX)
    {
        std::cout << "  FAIL: getChangedSettings did not include Cat.X." << std::endl;
        passed = false;
    }

    bool foundY = false;
    for(auto& s : changed)
        if(s.find("Cat.Y") != std::string::npos)
            foundY = true;
    if(foundY)
    {
        std::cout << "  FAIL: getChangedSettings should not include Cat.Y (still at default)." << std::endl;
        passed = false;
    }

    if(passed)
        std::cout << "  PASS: isSettingAtDefault and getChangedSettings work correctly." << std::endl;
    std::cout << "Finished isSettingAtDefault and getChangedSettings test." << std::endl;
    return passed;
}

bool TestReadSettingsFromStringPriority()
{
    bool passed = true;
    std::cout << "Starting readSettingsFromString priority test:" << std::endl;

    auto settings = std::make_shared<Settings>(std::make_shared<Output>());
    settings->createSetting("Termination.Dual.TimeLimit", 1000.0, "Time limit", 0.0, 1e12);

    std::string optStr = "Termination.Dual.TimeLimit=300\n";
    settings->readSettingsFromString(optStr);

    if(settings->getSettingPriority("Termination.Dual.TimeLimit") != E_SettingPriority::OptionsFile)
    {
        std::cout << "  FAIL: readSettingsFromString should set OptionsFile priority." << std::endl;
        passed = false;
    }

    if(settings->getSetting<double>("Termination.Dual.TimeLimit") != 300.0)
    {
        std::cout << "  FAIL: active value should be 300.0 after readSettingsFromString." << std::endl;
        passed = false;
    }

    // A SolverInternal write (default for bare updateSetting) should still win
    settings->updateSetting("Termination.Dual.TimeLimit", 999.0);
    if(settings->getSetting<double>("Termination.Dual.TimeLimit") != 999.0)
    {
        std::cout << "  FAIL: SolverInternal update should override OptionsFile." << std::endl;
        passed = false;
    }

    if(passed)
        std::cout << "  PASS: readSettingsFromString stores values at OptionsFile priority." << std::endl;
    std::cout << "Finished readSettingsFromString priority test." << std::endl;
    return passed;
}

bool TestSolverWrapperPriorityRouting()
{
    bool passed = true;
    std::cout << "Starting Solver wrapper priority routing test:" << std::endl;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();

    // Solver::updateSetting (no priority arg) must use UserAPI (30)
    solver->updateSetting("Termination.TimeLimit", 500.0);
    if(solver->getSetting<double>("Termination.TimeLimit") != 500.0)
    {
        std::cout << "  FAIL: Solver::updateSetting did not update the setting." << std::endl;
        passed = false;
    }

    auto prio = solver->getSettingPriority("Termination.TimeLimit");
    if(prio != E_SettingPriority::UserAPI)
    {
        std::cout << "  FAIL: Solver::updateSetting should route to UserAPI priority, got "
                  << static_cast<int>(prio) << "." << std::endl;
        passed = false;
    }

    // Explicit priority overload
    solver->updateSetting("Termination.TimeLimit", 1200.0, E_SettingPriority::SolverCompatibility);
    if(solver->getSetting<double>("Termination.TimeLimit") != 1200.0)
    {
        std::cout << "  FAIL: Solver::updateSetting with SolverCompatibility priority did not take effect."
                  << std::endl;
        passed = false;
    }

    if(passed)
        std::cout << "  PASS: Solver wrapper routes to correct priorities." << std::endl;
    std::cout << "Finished Solver wrapper priority routing test." << std::endl;
    return passed;
}

bool TestUserOverridesConvexityRecommendation()
{
    bool passed = true;
    std::cout << "Starting user-override-of-convexity-recommendation test:" << std::endl;

#ifndef HAS_AMPL
    std::cout << "  SKIP: AMPL (.nl) support not compiled in." << std::endl;
    return true;
#else
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // Short limits so the test runs quickly
    solver->updateSetting("Termination.IterationLimit", 50);
    solver->updateSetting("Termination.TimeLimit", 30.0);

    // Override BEFORE setProblem: setConvexityBasedSettings is called inside setProblem and will
    // try to set Relaxation.Use=false at RecommendedInternal (10). Our UserAPI (30) must win so
    // that the SolutionStrategy is constructed with relaxation enabled.
    solver->updateSetting("Dual.Relaxation.Use", true);

    try
    {
        if(!solver->setProblem("data/ncvx_min_div.nl"))
        {
            std::cout << "  FAIL: Could not load ncvx_min_div.nl." << std::endl;
            return false;
        }
    }
    catch(Exception& e)
    {
        std::cout << "  FAIL: Exception loading problem: " << e.what() << std::endl;
        return false;
    }

    // 1. Verify the problem is detected as nonconvex after loading
    if(env->problem->properties.convexity != E_ProblemConvexity::Nonconvex)
    {
        std::cout << "  FAIL: Expected ncvx_min_div.nl to be nonconvex, got convexity="
                  << static_cast<int>(env->problem->properties.convexity) << "." << std::endl;
        passed = false;
    }
    else
    {
        std::cout << "  OK: Problem is detected as nonconvex." << std::endl;
    }

    // 2. setConvexityBasedSettings (called inside setProblem) tried to write Relaxation.Use=false
    //    at RecommendedInternal (10). Our UserAPI (30) override should have won.
    if(solver->getSettingPriority("Dual.Relaxation.Use") != E_SettingPriority::UserAPI)
    {
        std::cout << "  FAIL: Relaxation.Use priority should be UserAPI after setProblem "
                     "(recommended-internal write should have lost)."
                  << std::endl;
        passed = false;
    }

    if(!solver->getSetting<bool>("Dual.Relaxation.Use"))
    {
        std::cout << "  FAIL: Relaxation.Use should still be true after setProblem." << std::endl;
        passed = false;
    }

    solver->solveProblem();

    // 3. After solving, UserAPI (30) should still win over any RecommendedInternal (10) writes
    //    that solveProblem/verifySettings may have triggered internally.
    if(solver->getSettingPriority("Dual.Relaxation.Use") != E_SettingPriority::UserAPI)
    {
        std::cout << "  FAIL: Relaxation.Use should still be at UserAPI priority after solveProblem."
                  << std::endl;
        passed = false;
    }

    // 4. The first SHOT iteration should be a relaxed (LP/QP) solve, not a MIP solve
    if(env->results->iterations.empty())
    {
        std::cout << "  FAIL: No iterations were recorded." << std::endl;
        passed = false;
    }
    else if(env->results->iterations[0]->isDualProblemDiscrete)
    {
        std::cout << "  FAIL: First iteration should be relaxed (LP/QP), but isDualProblemDiscrete=true."
                  << std::endl;
        passed = false;
    }
    else
    {
        std::cout << "  OK: First iteration is relaxed (isDualProblemDiscrete=false)." << std::endl;
    }

    if(passed)
        std::cout << "  PASS: User API override of convexity-based recommendation works correctly."
                  << std::endl;
    std::cout << "Finished user-override-of-convexity-recommendation test." << std::endl;
    return passed;
#endif
}

// Conditional values of settings: which values apply, the most specific value, the group prefix and the priorities
bool TestConditionalValues()
{
    bool passed = true;
    std::cout << "Starting conditional values test:" << std::endl;

    auto settings = std::make_shared<Settings>(std::make_shared<Output>());

    VectorString enumStrategy { "Zero", "One", "Two", "Three" };
    settings->createSetting("Test.Group.Strategy", 1, "A test enum setting", enumStrategy, 0);
    settings->createSetting("Test.Group.Limit", 10, "A test integer setting", 0.0, 1000.0);
    settings->createSetting("Test.GroupX.Use", true, "A test Boolean setting outside the group");
    settings->createSetting("Test.Other.Factor", 0.5, "A test double setting", 0.0, 1.0);

    // Two settings share the condition Nonconvex, and one of them also has a more specific value
    settings->addConditionalValue("Test.Group.Strategy", { E_SettingCondition::Nonconvex }, 2);
    settings->addConditionalValue(
        "Test.Group.Strategy", { E_SettingCondition::MIPSolverGurobi, E_SettingCondition::Nonconvex }, 3);
    settings->addConditionalValue("Test.Group.Limit", { E_SettingCondition::Nonconvex }, 50);
    settings->addConditionalValue("Test.GroupX.Use", { E_SettingCondition::Nonconvex }, false);
    settings->addConditionalValue("Test.Other.Factor", { E_SettingCondition::MIPSolverHighs }, 1);

    // Registering does not change the values
    if(settings->getSetting<int>("Test.Group.Strategy") != 1 || settings->getSetting<int>("Test.Group.Limit") != 10
        || !settings->isSettingAtDefault("Test.Group.Strategy"))
    {
        std::cout << "  FAIL: Registering conditional values changed the settings." << std::endl;
        passed = false;
    }

    // A value set by the user wins over the conditional values
    settings->updateSetting("Test.Group.Limit", 20, E_SettingPriority::UserAPI);

    // Inside the group, with the conditions Nonconvex and Gurobi: the most specific value of Strategy is used, and
    // the setting Test.GroupX.Use is not inside the group Test.Group
    settings->applyConditionalValues({ E_SettingCondition::Nonconvex, E_SettingCondition::MIPSolverGurobi },
        E_SettingPriority::RecommendedInternal, "Test.Group", true);

    if(settings->getSetting<int>("Test.Group.Strategy") != 3
        || settings->getSettingPriority("Test.Group.Strategy") != E_SettingPriority::RecommendedInternal)
    {
        std::cout << "  FAIL: The most specific conditional value was not used at the given priority." << std::endl;
        passed = false;
    }

    if(settings->getSetting<int>("Test.Group.Limit") != 20
        || settings->getSettingPriority("Test.Group.Limit") != E_SettingPriority::UserAPI
        || settings->getSettingAtPriority<int>("Group.Limit", "Test", E_SettingPriority::RecommendedInternal) != 50)
    {
        std::cout << "  FAIL: The conditional value should be stored but not override the user's value." << std::endl;
        passed = false;
    }

    if(!settings->getSetting<bool>("Test.GroupX.Use") || !settings->isSettingAtDefault("Test.GroupX.Use"))
    {
        std::cout << "  FAIL: A setting outside the group prefix was changed." << std::endl;
        passed = false;
    }

    // Outside the group, with only the condition Nonconvex: Use changes, but not Factor, which needs HiGHS
    settings->applyConditionalValues(
        { E_SettingCondition::Nonconvex }, E_SettingPriority::RecommendedInternal, "Test.Group", false);

    if(settings->getSetting<bool>("Test.GroupX.Use"))
    {
        std::cout << "  FAIL: The conditional value of a setting outside the group prefix was not used." << std::endl;
        passed = false;
    }

    if(settings->getSetting<double>("Test.Other.Factor") != 0.5)
    {
        std::cout << "  FAIL: A conditional value whose condition does not hold was used." << std::endl;
        passed = false;
    }

    // A second settings object only with the condition Nonconvex uses the less specific value
    auto otherSettings = std::make_shared<Settings>(std::make_shared<Output>());
    otherSettings->createSetting("Test.Group.Strategy", 1, "A test enum setting", enumStrategy, 0);
    otherSettings->addConditionalValue("Test.Group.Strategy", { E_SettingCondition::Nonconvex }, 2);
    otherSettings->addConditionalValue(
        "Test.Group.Strategy", { E_SettingCondition::MIPSolverGurobi, E_SettingCondition::Nonconvex }, 3);
    otherSettings->applyConditionalValues({ E_SettingCondition::Nonconvex, E_SettingCondition::MIPSolverCplex },
        E_SettingPriority::RecommendedInternal, "Test.Group", true);

    if(otherSettings->getSetting<int>("Test.Group.Strategy") != 2)
    {
        std::cout << "  FAIL: The value for only the condition Nonconvex was not used." << std::endl;
        passed = false;
    }

    // The values are documented in the setting descriptions
    auto markup = settings->getSettingsAsMarkup();

    if(markup.find("<br>Nonconvex default: 2 (Two)<br>Nonconvex default with Gurobi: 3 (Three)") == std::string::npos
        || markup.find("<br>Default with HiGHS: 1|") == std::string::npos)
    {
        std::cout << "  FAIL: The conditional values are not described correctly in the Markdown." << std::endl;
        passed = false;
    }

    if(passed)
        std::cout << "  PASS: Conditional values work correctly." << std::endl;
    std::cout << "Finished conditional values test." << std::endl;
    return passed;
}

// Invalid conditional values are rejected when they are registered
bool TestConditionalValueRegistration()
{
    bool passed = true;
    std::cout << "Starting conditional value registration test:" << std::endl;

    auto settings = std::make_shared<Settings>(std::make_shared<Output>());

    VectorString enumStrategy { "Zero", "One" };
    settings->createSetting("Test.Strategy", 0, "A test enum setting", enumStrategy, 0);
    settings->createSetting("Test.Use", true, "A test Boolean setting");
    settings->addConditionalValue("Test.Strategy", { E_SettingCondition::Nonconvex }, 1);

    auto isRejected = [&](const std::string& testCase, auto addValue)
    {
        try
        {
            addValue();
        }
        catch(std::exception&)
        {
            return;
        }

        std::cout << "  FAIL: " << testCase << " was not rejected." << std::endl;
        passed = false;
    };

    isRejected("An undefined setting",
        [&]() { settings->addConditionalValue("Test.Unknown", { E_SettingCondition::Nonconvex }, 1); });
    isRejected("A value of the wrong type",
        [&]() { settings->addConditionalValue("Test.Use", { E_SettingCondition::Nonconvex }, 1); });
    isRejected("An enum value that is not a choice",
        [&]() { settings->addConditionalValue("Test.Strategy", { E_SettingCondition::MIPSolverCbc }, 2); });
    isRejected("A second value for the same conditions",
        [&]()
        {
            settings->addConditionalValue(
                "Test.Strategy", { E_SettingCondition::Nonconvex, E_SettingCondition::Nonconvex }, 0);
        });

    if(settings->getSetting<int>("Test.Strategy") != 0 || !settings->isSettingAtDefault("Test.Strategy"))
    {
        std::cout << "  FAIL: A rejected conditional value changed the setting." << std::endl;
        passed = false;
    }

    if(passed)
        std::cout << "  PASS: Invalid conditional values are rejected." << std::endl;
    std::cout << "Finished conditional value registration test." << std::endl;
    return passed;
}

// The recommended values for nonconvex problems are applied after reading a problem, at the recommended priority, and
// not for a problem that the reformulation makes convex or with Strategy.UseRecommendedSettings = false
bool TestRecommendedSettingsForProblems()
{
    bool passed = true;
    std::cout << "Starting recommended settings test:" << std::endl;

#ifndef HAS_AMPL
    std::cout << "  SKIP: AMPL (.nl) support not compiled in." << std::endl;
    return true;
#else
    auto readProblem = [&](const std::string& problemFile, bool useRecommendedSettings)
    {
        auto solver = std::make_unique<Solver>();

        if(!useRecommendedSettings)
            solver->updateSetting("Strategy.UseRecommendedSettings", false);

        // Otherwise, nonconvex quadratic terms are kept with Gurobi, and the reformulation does not make the problem
        // convex
        solver->updateSetting("Model.Reformulation.Quadratics.Strategy",
            static_cast<int>(ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained));

        if(!solver->setProblem(problemFile))
        {
            std::cout << "  FAIL: Could not load " << problemFile << "." << std::endl;
            passed = false;
        }

        return solver;
    };

    // A nonconvex problem
    auto solver = readProblem("data/ncvx_min_div.nl", true);

    if(solver->getSetting<bool>("Dual.Relaxation.Use")
        || solver->getSettingPriority("Dual.Relaxation.Use") != E_SettingPriority::RecommendedInternal
        || solver->getSetting<int>("Dual.MIP.SolutionLimit.Initial") != SHOT_INT_MAX
        || solver->getSetting<int>("Primal.FixedInteger.CallStrategy")
            != static_cast<int>(ES_PrimalNLPStrategy::AlwaysUse))
    {
        std::cout << "  FAIL: The recommended values for a nonconvex problem were not used." << std::endl;
        passed = false;
    }

    // Without recommended settings, only the compatibility override of the tree strategy is applied
    solver = readProblem("data/ncvx_min_div.nl", false);

    if(!solver->getSetting<bool>("Dual.Relaxation.Use")
        || solver->getSettingPriority("Dual.Relaxation.Use") != E_SettingPriority::Default
        || solver->getSetting<int>("Dual.TreeStrategy") != static_cast<int>(ES_TreeStrategy::MultiTree)
        || solver->getSettingPriority("Dual.TreeStrategy") != E_SettingPriority::SolverCompatibility)
    {
        std::cout << "  FAIL: With Strategy.UseRecommendedSettings = false, only the compatibility override should "
                     "be applied."
                  << std::endl;
        passed = false;
    }

    // A nonconvex problem that the reformulation makes convex keeps the defaults for convex problems
    solver = readProblem("data/instances/MINLP-nonconvex/tln2.osil", true);

    if(solver->getEnvironment()->problem->properties.convexity == E_ProblemConvexity::Convex
        || solver->getEnvironment()->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
    {
        std::cout << "  FAIL: Expected a nonconvex problem that the reformulation makes convex." << std::endl;
        passed = false;
    }
    else if(!solver->getSetting<bool>("Dual.Relaxation.Use")
        || solver->getSettingPriority("Dual.Relaxation.Use") != E_SettingPriority::Default)
    {
        std::cout << "  FAIL: The recommended values for nonconvex problems were used for a convex reformulation."
                  << std::endl;
        passed = false;
    }

    if(passed)
        std::cout << "  PASS: The recommended settings are applied correctly." << std::endl;
    std::cout << "Finished recommended settings test." << std::endl;
    return passed;
#endif
}

// The settings of all backends are defined, also if SHOT is not compiled with them, but misspelled settings are not
bool TestSettingsOfAllBackends()
{
    bool passed = true;
    std::cout << "Starting test of the settings of all backends:" << std::endl;

    auto solver = std::make_unique<Solver>();
    auto markup = solver->getSettingsAsMarkup();

    for(auto& name : { "Subsolver.Cplex.MIPEmphasis", "Subsolver.Gurobi.MIPFocus", "Subsolver.Cbc.Strategy",
            "Subsolver.Highs.Presolve", "Subsolver.Conopt.FeasibilityTolerance", "Subsolver.Ipopt.LinearSolver",
            "Subsolver.Uno.Preset", "Subsolver.GAMS.NLP.Solver", "ModelingSystem.GAMS.QExtractAlg" })
    {
        if(markup.find(fmt::format("**{}**", name)) == std::string::npos)
        {
            std::cout << "  FAIL: The setting " << name << " is not documented." << std::endl;
            passed = false;
        }
    }

    try
    {
        solver->updateSetting("Subsolver.Gurobi.MIPFocus", 1);
    }
    catch(std::exception& e)
    {
        std::cout << "  FAIL: Setting a Gurobi setting raised an exception: " << e.what() << std::endl;
        passed = false;
    }

    try
    {
        solver->updateSetting("Subsolver.Gurobi.MIPFokus", 1);
        std::cout << "  FAIL: A misspelled setting was accepted." << std::endl;
        passed = false;
    }
    catch(std::exception&)
    {
    }

    if(passed)
        std::cout << "  PASS: The settings of all backends are defined." << std::endl;
    std::cout << "Finished test of the settings of all backends." << std::endl;
    return passed;
}

// Test the writing and reading of options files
bool SettingsTestOptions(bool useOSiL)
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();

    std::string filename;

    if(useOSiL)
    {
        filename = "SHOT.osol";
    }
    else
    {
        filename = "SHOT.opt";
    }

    if(fs::filesystem::exists(filename))
        std::remove(filename.c_str());

    try
    {
        if(useOSiL)
        {
            if(!Utilities::writeStringToFile(filename, solver->getOptionsOSoL()))
            {
                passed = false;
            }
        }
        else
        {
            if(!Utilities::writeStringToFile(filename, solver->getOptions()))
            {
                passed = false;
            }
        }

        if(passed && !solver->setOptionsFromFile(filename))
        {
            std::cout << "Could not read OSoL file." << std::endl;
            passed = false;
        }
    }
    catch(Exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        passed = false;
    }

    return passed;
}