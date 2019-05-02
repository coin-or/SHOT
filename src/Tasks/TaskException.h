/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Environment.h"

#include <sstream>
#include <string>

namespace SHOT
{
class TaskException : public std::exception
{
public:
    TaskException(EnvironmentPtr envPtr, std::string msg) : message(msg) {}
    TaskException() = default;

    const char* what() const throw() override
    {
        if(message == "")
            return "Unspecified task exception occurred!";
        else
        {
            std::stringstream tmpMessage;
            tmpMessage << "Task exception: ";
            tmpMessage << message;

            return (tmpMessage.str().c_str());
        }
    }

private:
    std::string message;
};

class TaskExceptionFunctionNotDefined : public std::exception
{
public:
    TaskExceptionFunctionNotDefined(EnvironmentPtr envPtr, std::string task) : taskName(task) {}

    const char* what() const throw() override
    {
        std::stringstream message;
        message << "Exception: task function in ";
        message << taskName;
        message << " not defined!";

        return (message.str().c_str());
    }

private:
    std::string taskName;
};

class TaskExceptionNotFound : public std::exception
{
public:
    TaskExceptionNotFound(EnvironmentPtr envPtr, std::string task) : taskID(task) {}

    const char* what() const throw() override
    {
        std::stringstream message;
        message << "Exception: task with ID ";
        message << taskID;
        message << " does not exist!";

        return (message.str().c_str());
    }

private:
    std::string taskID;
};
} // namespace SHOT