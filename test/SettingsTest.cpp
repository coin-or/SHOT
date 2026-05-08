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
    settings->createSetting("TestInt", "Test", 0, "A test integer setting", 0.0, 1000.0);

    settings->updateSetting("TestInt", "Test", 42, E_SettingPriority::SolverInternal);
    if(settings->getSetting<int>("TestInt", "Test") != 42)
    {
        std::cout << "  FAIL: SolverInternal write did not take effect." << std::endl;
        passed = false;
    }

    // A lower-priority write should NOT change the active value
    settings->updateSetting("TestInt", "Test", 7, E_SettingPriority::OptionsFile);
    if(settings->getSetting<int>("TestInt", "Test") != 42)
    {
        std::cout << "  FAIL: Lower-priority (OptionsFile) write incorrectly overrode SolverInternal." << std::endl;
        passed = false;
    }

    // But it should be stored in history
    if(!settings->hasSettingAtPriority("TestInt", "Test", E_SettingPriority::OptionsFile))
    {
        std::cout << "  FAIL: OptionsFile value not found in history after lower-priority write." << std::endl;
        passed = false;
    }

    // Active priority should still be SolverInternal
    if(settings->getSettingPriority("TestInt", "Test") != E_SettingPriority::SolverInternal)
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
    settings->createSetting("HistDouble", "Test", 1.0, "A test double setting", 0.0, 100.0);

    settings->updateSetting("HistDouble", "Test", 5.0, E_SettingPriority::OptionsFile);
    settings->updateSetting("HistDouble", "Test", 9.0, E_SettingPriority::SolverCompatibility);
    settings->updateSetting("HistDouble", "Test", 12.5, E_SettingPriority::SolverInternal);

    // getSettingAtPriority
    double atOptions = settings->getSettingAtPriority<double>("HistDouble", "Test", E_SettingPriority::OptionsFile);
    if(atOptions != 5.0)
    {
        std::cout << "  FAIL: getSettingAtPriority<double> at OptionsFile returned " << atOptions
                  << " (expected 5.0)." << std::endl;
        passed = false;
    }

    double atSolverCompat
        = settings->getSettingAtPriority<double>("HistDouble", "Test", E_SettingPriority::SolverCompatibility);
    if(atSolverCompat != 9.0)
    {
        std::cout << "  FAIL: getSettingAtPriority<double> at SolverCompatibility returned " << atSolverCompat
                  << " (expected 9.0)." << std::endl;
        passed = false;
    }

    // hasSettingAtPriority
    if(!settings->hasSettingAtPriority("HistDouble", "Test", E_SettingPriority::Default))
    {
        std::cout << "  FAIL: hasSettingAtPriority should return true for Default (initial value)." << std::endl;
        passed = false;
    }
    if(settings->hasSettingAtPriority("HistDouble", "Test", E_SettingPriority::UserAPI))
    {
        std::cout << "  FAIL: hasSettingAtPriority should return false for UserAPI (never written)." << std::endl;
        passed = false;
    }

    // getSettingPriorityHistory — should contain Default(0), OptionsFile(20), SolverInternal(40), SolverCompatibility(50)
    auto history = settings->getSettingPriorityHistory("HistDouble", "Test");
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
    settings->createSetting("A", "Cat", 1, "Setting A", 0.0, 100.0);
    settings->createSetting("B", "Cat", 2, "Setting B", 0.0, 100.0);
    settings->createSetting("C", "Cat", 3, "Setting C", 0.0, 100.0);

    settings->updateSetting("A", "Cat", 10, E_SettingPriority::UserAPI);
    settings->updateSetting("B", "Cat", 20, E_SettingPriority::UserAPI);
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
    settings->createSetting("X", "Cat", 5, "Setting X", 0.0, 100.0);
    settings->createSetting("Y", "Cat", true, "Setting Y");

    if(!settings->isSettingAtDefault("X", "Cat"))
    {
        std::cout << "  FAIL: isSettingAtDefault should be true before any update." << std::endl;
        passed = false;
    }

    settings->updateSetting("X", "Cat", 99, E_SettingPriority::UserAPI);

    if(settings->isSettingAtDefault("X", "Cat"))
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
    settings->createSetting("Dual.TimeLimit", "Termination", 1000.0, "Time limit", 0.0, 1e12);

    std::string optStr = "Termination.Dual.TimeLimit=300\n";
    settings->readSettingsFromString(optStr);

    if(settings->getSettingPriority("Dual.TimeLimit", "Termination") != E_SettingPriority::OptionsFile)
    {
        std::cout << "  FAIL: readSettingsFromString should set OptionsFile priority." << std::endl;
        passed = false;
    }

    if(settings->getSetting<double>("Dual.TimeLimit", "Termination") != 300.0)
    {
        std::cout << "  FAIL: active value should be 300.0 after readSettingsFromString." << std::endl;
        passed = false;
    }

    // A SolverInternal write (default for bare updateSetting) should still win
    settings->updateSetting("Dual.TimeLimit", "Termination", 999.0);
    if(settings->getSetting<double>("Dual.TimeLimit", "Termination") != 999.0)
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
    solver->updateSetting("TimeLimit", "Termination", 500.0);
    if(solver->getSetting<double>("TimeLimit", "Termination") != 500.0)
    {
        std::cout << "  FAIL: Solver::updateSetting did not update the setting." << std::endl;
        passed = false;
    }

    auto prio = solver->getSettingPriority("TimeLimit", "Termination");
    if(prio != E_SettingPriority::UserAPI)
    {
        std::cout << "  FAIL: Solver::updateSetting should route to UserAPI priority, got "
                  << static_cast<int>(prio) << "." << std::endl;
        passed = false;
    }

    // Explicit priority overload
    solver->updateSetting("TimeLimit", "Termination", 1200.0, E_SettingPriority::SolverCompatibility);
    if(solver->getSetting<double>("TimeLimit", "Termination") != 1200.0)
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
    solver->updateSetting("IterationLimit", "Termination", 50);
    solver->updateSetting("TimeLimit", "Termination", 30.0);

    // Override BEFORE setProblem: setConvexityBasedSettings is called inside setProblem and will
    // try to set Relaxation.Use=false at RecommendedInternal (10). Our UserAPI (30) must win so
    // that the SolutionStrategy is constructed with relaxation enabled.
    solver->updateSetting("Relaxation.Use", "Dual", true);

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
    if(solver->getSettingPriority("Relaxation.Use", "Dual") != E_SettingPriority::UserAPI)
    {
        std::cout << "  FAIL: Relaxation.Use priority should be UserAPI after setProblem "
                     "(recommended-internal write should have lost)."
                  << std::endl;
        passed = false;
    }

    if(!solver->getSetting<bool>("Relaxation.Use", "Dual"))
    {
        std::cout << "  FAIL: Relaxation.Use should still be true after setProblem." << std::endl;
        passed = false;
    }

    solver->solveProblem();

    // 3. After solving, UserAPI (30) should still win over any RecommendedInternal (10) writes
    //    that solveProblem/verifySettings may have triggered internally.
    if(solver->getSettingPriority("Relaxation.Use", "Dual") != E_SettingPriority::UserAPI)
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