/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include <chrono>
#include <string>

class Timer
{
public:
    Timer() = delete;
    ~Timer() = default;

    Timer(std::string timerName)
    {
        restart();
        isRunning = false;
        description = "";
        name = timerName;
    }

    Timer(std::string timerName, std::string desc)
    {
        restart();
        isRunning = false;
        description = desc;
        name = timerName;
    }

    std::chrono::time_point<std::chrono::high_resolution_clock> lastStart;

    inline double elapsed()
    {
        if(isRunning)
        {
            std::chrono::duration<double> dur = std::chrono::high_resolution_clock::now() - lastStart;
            double tmpTime = dur.count();
            return (timeElapsed + tmpTime);
        }
        return (timeElapsed);
    }

    inline void restart()
    {
        isRunning = true;
        timeElapsed = 0.0;
        lastStart = std::chrono::high_resolution_clock::now();
    }

    inline void stop()
    {
        if(!isRunning)
            return;

        std::chrono::duration<double> dur = std::chrono::high_resolution_clock::now() - lastStart;
        double tmpTime = dur.count();
        timeElapsed = timeElapsed + tmpTime;
        isRunning = false;
    }

    inline void start()
    {
        if(isRunning)
        {
            return;
        }

        isRunning = true;
        lastStart = std::chrono::high_resolution_clock::now();
    }

    std::string description;
    std::string name;

private:
    double timeElapsed;
    bool isRunning;
};
