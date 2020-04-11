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