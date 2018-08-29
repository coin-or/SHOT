/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/
#include <iostream>
#include "Terms.h"
#include "Constraints.h"
//#include "Model/NonlinearExpressions.h"
#include "ObjectiveFunction.h"

using namespace SHOT;

bool ModelTest1();

int ModelTest(int argc, char *argv[])
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
        passed = ModelTest1();
        break;
    case 2:
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    if (passed)
        return 0;
    else
        return -1;
}

bool ModelTest1()
{
    bool passed = true;

    std::cout << "Creating variable:" << std::endl;
    VariablePtr variable(new Variable());
    variable->name = "x";
    variable->index = 0;
    variable->type = E_VariableType::Real;
    variable->lowerBound = 0.0;
    variable->upperBound = 100.0;

    std::cout << "Variable " << variable->name << " created." << std::endl;

    VectorDouble point;
    point.push_back(2.0);

    double value = variable->calculate(point);

    std::cout << "Calculating variable value: " << value << " (should be equal to " << point.at(0) << ")." << std::endl;

    if (value != point.at(0))
        passed = false;

    std::cout << "Creating linear term:" << std::endl;
    LinearTermPtr linearTerm(new LinearTerm());
    linearTerm->coefficient = 1.1;
    linearTerm->variable = variable;

    std::cout << "Linear term created: " << linearTerm << std::endl;

    value = linearTerm->calculate(point);

    std::cout << "Calculating term value: " << value << " (should be equal to " << linearTerm->coefficient * point.at(0) << ")." << std::endl;

    if (value != linearTerm->coefficient * point.at(0))
        passed = false;

    return passed;
}