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
    Output::getInstance().outputIterationDetailHeader();
}

std::string TaskPrintIterationHeader::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
