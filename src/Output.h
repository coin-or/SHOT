/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include <float.h>
#include "Enums.h"

// Used for OSOutput
#include "cstdio"
#define HAVE_STDIO_H 1
#include "OSOutput.h"

#include <boost/format.hpp>

namespace SHOT
{
class Output
{
  public:
    Output();
    virtual ~Output();

    void outputAlways(std::string message);
    void outputError(std::string message);
    void outputError(std::string message, std::string errormessage);
    void outputSummary(std::string message);
    void outputWarning(std::string message);
    void outputInfo(std::string message);
    void outputDebug(std::string message);
    void outputTrace(std::string message);
    void outputDetailedTrace(std::string message);

    void setLogLevels(int consoleLogLevel, int fileLogLevel);

  private:
    OSOutput *osOutput;
};
}