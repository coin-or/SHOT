/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeLinesearch.h"

TaskInitializeLinesearch::TaskInitializeLinesearch()
{
    ProcessInfo::getInstance().startTimer("HyperplaneLinesearch");

    if (Settings::getInstance().getIntSetting("Rootsearch.Method", "Subsolver") == static_cast<int>(ES_RootsearchMethod::Bisection))
    {
        ProcessInfo::getInstance().linesearchMethod = new LinesearchMethodBisection();
        ProcessInfo::getInstance().outputInfo("Bisection linesearch implementation selected.");
    }
    else
    {
        ProcessInfo::getInstance().linesearchMethod = new LinesearchMethodBoost();
        ProcessInfo::getInstance().outputInfo("Boost linesearch implementation selected.");
    }

    ProcessInfo::getInstance().stopTimer("HyperplaneLinesearch");
}

TaskInitializeLinesearch::~TaskInitializeLinesearch()
{
    delete ProcessInfo::getInstance().linesearchMethod;
}

void TaskInitializeLinesearch::run()
{
}

std::string TaskInitializeLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
