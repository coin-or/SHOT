/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "SHOTSolver.h"
bool GAMSTest1();

int GAMSTest(int argc, char *argv[])
{

    int defaultchoice = 1;

    int choice = defaultchoice;

    if (argc > 1)
    {
        if (sscanf(argv[1], "%d", &choice) != 1)
        {
            printf("Couldn't parse that input as a number\n");
            return -1;
        }
    }

    bool passed = true;

    switch (choice)
    {
    case 1:
        passed = GAMSTest1();
        break;
    case 2:
        break;
    default:
        passed = false;
        cout << "Test #" << choice << " does not exist!\n";
    }

    if (passed)
        return 0;
    else
        return -1;
}

bool GAMSTest1()
{
    SHOTSolver testSolver;
    bool passed = true;

    if (testSolver.setProblem("ss"))
    {
        passed = true;
    }
    else
    {
        passed = false;
    }

    return passed;
}