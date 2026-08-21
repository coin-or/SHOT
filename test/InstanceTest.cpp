/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Enums.h"
#include "../src/Results.h"
#include "../src/Solver.h"
#include "../src/Utilities.h"

#include "../src/Model/ObjectiveFunction.h"
#include "../src/Model/Problem.h"
#include "../src/Timing.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
#endif

using namespace SHOT;
using json = nlohmann::json;

enum class E_InstanceResult
{
    Pass,
    WarnObjective,
    WarnNoSolution,
    WarnFeasibility,
    FailCrash,
    Skipped
};

struct InstanceEntry
{
    std::string file;
    double expectedObjective = 0.0;
    bool isInfeasible = false;
    std::string description;
};

static std::vector<InstanceEntry> parseInstancesJson(const std::string& directory)
{
    std::vector<InstanceEntry> entries;
    std::string jsonPath = directory + "/instances.json";

    if(!fs::filesystem::exists(jsonPath))
        return entries;

    try
    {
        std::ifstream f(jsonPath);
        json j = json::parse(f);

        for(const auto& inst : j.at("instances"))
        {
            InstanceEntry e;
            e.file = inst.at("file").get<std::string>();

            if(inst.contains("status") && inst["status"].get<std::string>() == "infeasible")
            {
                e.isInfeasible = true;
            }
            else if(inst.contains("objective"))
            {
                e.expectedObjective = inst["objective"].get<double>();
            }

            if(inst.contains("description"))
                e.description = inst["description"].get<std::string>();

            entries.push_back(e);
        }
    }
    catch(const std::exception& ex)
    {
        std::cout << "  Error parsing " << jsonPath << ": " << ex.what() << '\n';
    }

    return entries;
}

static bool isFormatSupported(Solver& solver, const std::string& extension)
{
    if(extension == ".nl")
        return solver.hasModelingSystem(ES_ModelingSystem::AMPL);
    if(extension == ".gms")
        return solver.hasModelingSystem(ES_ModelingSystem::GAMS);
    // .osil and .xml are always available
    return (extension == ".osil" || extension == ".xml");
}

static E_InstanceResult solveInstance(const InstanceEntry& entry, const std::string& filepath, ES_MIPSolver mipSolver,
    ES_PrimalNLPSolver nlpSolver, bool verbose, const std::string& solverDesc)
{
    constexpr double tolerance = 1e-2;
    constexpr double timeLimit = 30.0;

    std::cout << fmt::format("  Solving {} [{}]...\n", entry.file, solverDesc);
    std::cout.flush();

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(verbose ? E_LogLevel::Info : E_LogLevel::Off));
    solver->updateLogLevels();
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));
    solver->updateSetting("Primal.FixedInteger.Solver", static_cast<int>(nlpSolver));
    solver->updateSetting("Termination.TimeLimit", timeLimit);

    try
    {
        if(!solver->setProblem(filepath))
        {
            std::cout << fmt::format("  [WARN] {}: failed to load\n", entry.file);
            return E_InstanceResult::WarnNoSolution;
        }

        solver->solveProblem();
    }
    catch(const std::exception& ex)
    {
        double elapsed = solver->getEnvironment()->timing->getElapsedTime("Total");
        std::cout << fmt::format("  [FAIL] {}: exception: {} ({:.1f}s)\n", entry.file, ex.what(), elapsed);
        return E_InstanceResult::FailCrash;
    }

    double elapsed = solver->getEnvironment()->timing->getElapsedTime("Total");
    auto status = solver->getModelReturnStatus();

    if(entry.isInfeasible)
    {
        bool confirmed
            = (status == E_ModelReturnStatus::InfeasibleGlobal || status == E_ModelReturnStatus::InfeasibleLocal);
        if(confirmed)
        {
            std::cout << fmt::format("  [PASS] {}: infeasible confirmed ({:.1f}s)\n", entry.file, elapsed);
            return E_InstanceResult::Pass;
        }
        else
        {
            std::cout << fmt::format("  [WARN] {}: expected infeasible, got status {} ({:.1f}s)\n", entry.file,
                static_cast<int>(status), elapsed);
            return E_InstanceResult::WarnFeasibility;
        }
    }

    if(!solver->hasPrimalSolution())
    {
        std::cout << fmt::format("  [WARN] {}: no primal solution ({:.1f}s)\n", entry.file, elapsed);
        return E_InstanceResult::WarnNoSolution;
    }

    double primal = solver->getPrimalBound();
    double dual = solver->getCurrentDualBound();
    double obj = entry.expectedObjective;

    bool isMin = solver->getOriginalProblem()->objectiveFunction->properties.isMinimize;
    bool withinBounds = isMin ? (obj <= primal + tolerance && obj >= dual - tolerance)
                              : (obj >= primal - tolerance && obj <= dual + tolerance);

    if(withinBounds)
    {
        std::cout << fmt::format(
            "  [PASS] {}: objective = {:.6g} (expected {:.6g}), {:.1f}s\n", entry.file, primal, obj, elapsed);
        return E_InstanceResult::Pass;
    }
    else
    {
        std::cout << fmt::format("  [WARN] {}: primal = {:.6g}, dual = {:.6g}, expected {:.6g}, {:.1f}s\n", entry.file,
            primal, dual, obj, elapsed);
        return E_InstanceResult::WarnObjective;
    }
}

static bool runInstanceTests(
    ES_MIPSolver mipSolver, ES_PrimalNLPSolver nlpSolver, bool verbose, const std::string& solverDesc)
{
    const std::string instancesRoot = "data/instances";

    if(!fs::filesystem::exists(instancesRoot))
    {
        std::cout << "  Instance test directory '" << instancesRoot << "' not found\n";
        return false;
    }

    int pass = 0, warn = 0, fail = 0, skip = 0;
    std::vector<std::pair<std::string, std::string>> warnList, failList;

    // probe once to check format support without loading a problem
    Solver probe;

    for(auto& subdir : fs::filesystem::directory_iterator(instancesRoot))
    {
        if(!subdir.is_directory())
            continue;

        std::string dir = subdir.path().string();
        auto entries = parseInstancesJson(dir);

        if(entries.empty())
            continue;

        std::cout << "\n  --- " << subdir.path().filename().string() << " ---\n";

        for(const auto& entry : entries)
        {
            std::string ext = fs::filesystem::path(entry.file).extension().string();

            if(!isFormatSupported(probe, ext))
            {
                std::cout << fmt::format("  [SKIP] {}: format '{}' not compiled in\n", entry.file, ext);
                skip++;
                continue;
            }

            std::string filepath = dir + "/" + entry.file;

            if(!fs::filesystem::exists(filepath))
            {
                std::cout << fmt::format("  [SKIP] {}: file not found\n", entry.file);
                skip++;
                continue;
            }

            auto result = solveInstance(entry, filepath, mipSolver, nlpSolver, verbose, solverDesc);

            switch(result)
            {
            case E_InstanceResult::Pass:
                pass++;
                break;
            case E_InstanceResult::FailCrash:
                fail++;
                failList.emplace_back(entry.file, "crash/exception");
                break;
            case E_InstanceResult::WarnObjective:
                warn++;
                warnList.emplace_back(entry.file, "objective mismatch");
                break;
            case E_InstanceResult::WarnNoSolution:
                warn++;
                warnList.emplace_back(entry.file, "no primal solution");
                break;
            case E_InstanceResult::WarnFeasibility:
                warn++;
                warnList.emplace_back(entry.file, "infeasibility not confirmed");
                break;
            case E_InstanceResult::Skipped:
                skip++;
                break;
            default:
                warn++;
                warnList.emplace_back(entry.file, "unknown");
                break;
            }
        }
    }

    std::cout << fmt::format("\n  Results: {} passed, {} warned, {} failed, {} skipped\n", pass, warn, fail, skip);

    if(!warnList.empty())
    {
        std::cout << "\n  Warned:\n";
        for(const auto& [f, reason] : warnList)
            std::cout << fmt::format("    - {} ({})", f, reason) << '\n';
    }

    if(!failList.empty())
    {
        std::cout << "\n  Failed:\n";
        for(const auto& [f, reason] : failList)
            std::cout << fmt::format("    - {} ({})", f, reason) << '\n';
    }

    return fail == 0;
}

int InstanceTest(int argc, char* argv[])
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

    bool verbose = false;
    for(int i = 2; i < argc; i++)
        if(std::string(argv[i]) == "-v")
            verbose = true;

    bool passed = false;

    switch(choice)
    {
    case 1:
        std::cout << "Instance tests: HiGHS + Ipopt\n";
        passed = runInstanceTests(ES_MIPSolver::Highs, ES_PrimalNLPSolver::Ipopt, verbose, "HiGHS + Ipopt");
        break;
    case 2:
        std::cout << "Instance tests: Gurobi + Ipopt\n";
        passed = runInstanceTests(ES_MIPSolver::Gurobi, ES_PrimalNLPSolver::Ipopt, verbose, "Gurobi + Ipopt");
        break;
    case 3:
        std::cout << "Instance tests: Cplex + Ipopt\n";
        passed = runInstanceTests(ES_MIPSolver::Cplex, ES_PrimalNLPSolver::Ipopt, verbose, "Cplex + Ipopt");
        break;
    case 4:
        std::cout << "Instance tests: Cbc + Ipopt\n";
        passed = runInstanceTests(ES_MIPSolver::Cbc, ES_PrimalNLPSolver::Ipopt, verbose, "Cbc + Ipopt");
        break;
    case 5:
        std::cout << "Instance tests: HiGHS + SHOT (NLP)\n";
        passed = runInstanceTests(ES_MIPSolver::Highs, ES_PrimalNLPSolver::SHOT, verbose, "HiGHS + SHOT");
        break;
    case 6:
        std::cout << "Instance tests: Gurobi + SHOT (NLP)\n";
        passed = runInstanceTests(ES_MIPSolver::Gurobi, ES_PrimalNLPSolver::SHOT, verbose, "Gurobi + SHOT");
        break;
    default:
        std::cout << "Test #" << choice << " does not exist!\n";
        return -1;
    }

    if(!passed)
        std::cout << "\nInstance tests FAILED (crashes occurred)\n";
    else
        std::cout << "\nInstance tests PASSED (no crashes)\n";

    return passed ? 0 : -1;
}
