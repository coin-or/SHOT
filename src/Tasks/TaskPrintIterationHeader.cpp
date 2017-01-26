/*
 * TaskPrintIterationHeader.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include <TaskPrintIterationHeader.h>

TaskPrintIterationHeader::TaskPrintIterationHeader()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskPrintIterationHeader::~TaskPrintIterationHeader()
{
	// TODO Auto-generated destructor stub
}

void TaskPrintIterationHeader::run()
{
	std::stringstream tmpLine;
	tmpLine << "\n═════════════════════════════════════════════════════════════════════════════════════\n";

	tmpLine
			<< boost::format("%|-14| %|=11| %|=14| %|=14| %|=14|  %s\n") % "Iteration" % "HPs" % "DB" % "OBJ" % "PB"
					% "max constr.";

	tmpLine << "═════════════════════════════════════════════════════════════════════════════════════\n";

	processInfo->outputSummary(tmpLine.str());
}

std::string TaskPrintIterationHeader::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
