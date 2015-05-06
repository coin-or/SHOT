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
	//if ()
	//{
	auto tmpLine = boost::format("%1% %|4t|%2% %|10t|%3% %|14t|%4% %|24t|%5% %|38t|%6% %|46t|%7%: %|54t|%8% %|70t|%9%")
			% "#" % "Type" % "SL" % "HPs" % "Obj. val." % "Tol." % "Max. error" % " " % "||HPP||";

	processInfo->logger.message(2)
			<< "==================================================================================" << CoinMessageEol
			<< tmpLine.str() << CoinMessageEol
			<< "==================================================================================" << CoinMessageEol;
	//}
}

std::string TaskPrintIterationHeader::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
