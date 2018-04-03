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

#ifdef _WIN32
	tmpLine << "ÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ\n";

	tmpLine
		<< boost::format("%|=14| %|=11| %|=14| %|=14| %|=14|  %s\n") % " Iteration" % "HPs" % "DB" % "OBJ" % "PB" % "max constr.";

	tmpLine << "ÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ\n";

#else
	tmpLine << "═════════════════════════════════════════════════════════════════════════════════════\n";

	tmpLine
		<< boost::format("%|=14| %|=11| %|=14| %|=14| %|=14|  %s\n") % " Iteration" % "HPs" % "DB" % "OBJ" % "PB" % "max constr.";

	tmpLine << "═════════════════════════════════════════════════════════════════════════════════════\n";

#endif

	Output::getInstance().outputSummary(tmpLine.str());
}

std::string TaskPrintIterationHeader::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
