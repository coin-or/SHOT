/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "chrono"
#include "Enums.h"
#include "string"

class Timer
{
  public:
    Timer();
    Timer(std::string timerName);
    Timer(std::string timerName, std::string desc);
    ~Timer();

    std::chrono::time_point<std::chrono::high_resolution_clock> lastStart;

    double elapsed();
    void restart();
    void stop();
    void start();

    std::string description;
    std::string name;

  private:
    double timeElapsed;
    bool isRunning;
};
