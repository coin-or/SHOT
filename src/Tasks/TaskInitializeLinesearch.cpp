/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeLinesearch.h"

TaskInitializeLinesearch::TaskInitializeLinesearch(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("DualCutGenerationRootSearch");

    /*if (env->settings->getIntSetting("Rootsearch.Method", "Subsolver") == static_cast<int>(ES_RootsearchMethod::Bisection))
    {
        env->process->linesearchMethod = new LinesearchMethodBisection(env);
        env->output->outputInfo("Bisection linesearch implementation selected.");
    }
    else
    {*/
    env->process->linesearchMethod = new LinesearchMethodBoost(env);
    env->output->outputInfo("Boost linesearch implementation selected.");
    //}

    env->process->stopTimer("DualCutGenerationRootSearch");
}

TaskInitializeLinesearch::~TaskInitializeLinesearch()
{
    delete env->process->linesearchMethod;
}

void TaskInitializeLinesearch::run()
{
}

std::string TaskInitializeLinesearch::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
