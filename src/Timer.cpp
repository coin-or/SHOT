/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "Timer.h"

Timer::Timer(std::string timerName)
{
    restart();
    isRunning = false;
    description = "";
    name = timerName;
}

Timer::Timer(std::string timerName, std::string desc)
{
    restart();
    isRunning = false;
    description = desc;
    name = timerName;
}

Timer::~Timer()
{
}

double Timer::elapsed()
{
    if (isRunning)
    {
        std::chrono::duration<double> dur = std::chrono::high_resolution_clock::now() - lastStart;
        double tmpTime = dur.count();
        return (timeElapsed + tmpTime);
    }
    return (timeElapsed);
}

void Timer::restart()
{
    isRunning = true;
    timeElapsed = 0.0;
    lastStart = std::chrono::high_resolution_clock::now();
}

void Timer::stop()
{
    if (!isRunning)
        return;

    std::chrono::duration<double> dur = std::chrono::high_resolution_clock::now() - lastStart;
    double tmpTime = dur.count();
    timeElapsed = timeElapsed + tmpTime;
    isRunning = false;
}

void Timer::start()
{
    if (isRunning)
    {
        return;
    }

    isRunning = true;
    lastStart = std::chrono::high_resolution_clock::now();
}
