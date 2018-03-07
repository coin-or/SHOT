/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskPrintProblemStats.h"

TaskPrintProblemStats::TaskPrintProblemStats()
{
}

TaskPrintProblemStats::~TaskPrintProblemStats()
{
}

void TaskPrintProblemStats::run()
{
	ProcessInfo::getInstance().originalProblem->printProblemStatistics();
}

std::string TaskPrintProblemStats::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
