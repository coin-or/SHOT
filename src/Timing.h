/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Environment.h"
#include "Timer.h"

#include <algorithm>
#include <vector>

namespace SHOT
{

class Timing
{
public:
    inline Timing(EnvironmentPtr envPtr) { env = envPtr; };

    inline ~Timing() { timers.clear(); }

    inline void createTimer(std::string name, std::string description) { timers.emplace_back(name, description); }

    inline void startTimer(std::string name)
    {
        auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const& T) { return (T.name == name); });

        if(timer == timers.end())
        {
            // env->output->outputError("Timer with name  \"" + name + "\" not found!");
            return;
        }

        timer->start();
    }

    inline void stopTimer(std::string name)
    {
        auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const& T) { return (T.name == name); });

        if(timer == timers.end())
        {
            // env->output->outputError("Timer with name  \"" + name + "\" not found!");
            return;
        }

        timer->stop();
    }

    inline void restartTimer(std::string name)
    {
        auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const& T) { return (T.name == name); });

        if(timer == timers.end())
        {
            // env->output->outputError("Timer with name  \"" + name + "\" not found!");
            return;
        }

        timer->restart();
    }

    inline double getElapsedTime(std::string name)
    {
        auto timer = std::find_if(timers.begin(), timers.end(), [name](Timer const& T) { return (T.name == name); });

        if(timer == timers.end())
        {
            // env->output->outputError("Timer with name  \"" + name + "\" not found!");
            return (0.0);
        }

        return (timer->elapsed());
    }

    std::vector<Timer> timers;

private:
    EnvironmentPtr env;
};

} // namespace SHOT