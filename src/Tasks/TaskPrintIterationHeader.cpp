/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskPrintIterationHeader.h"

TaskPrintIterationHeader::TaskPrintIterationHeader()
{
}

TaskPrintIterationHeader::~TaskPrintIterationHeader()
{
}

void TaskPrintIterationHeader::run()
{

    std::stringstream tmpLine;
    tmpLine << "                                                                                     \n";

    tmpLine << "    Iteration     │  Time  │  Dual cuts  │     Objective value     │   Objective gap   │     Current solution\r\n";
    tmpLine << "     #: type      │  tot.  │   + | tot.  │       dual | primal     │    abs. | rel.    │    obj.fn. | max.err.\r\n";
    tmpLine << "╶─────────────────┴────────┴─────────────┴─────────────────────────┴───────────────────┴───────────────────────────╴\r\n";

    Output::getInstance().outputSummary(tmpLine.str());
}

std::string TaskPrintIterationHeader::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
