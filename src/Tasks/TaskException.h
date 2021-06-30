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
#include <stdexcept>

namespace SHOT
{
class TaskException : public std::runtime_error
{
public:
    TaskException(EnvironmentPtr envPtr [[maybe_unused]], std::string msg) : std::runtime_error(""), message(msg)
    {
        std::stringstream tmpMessage;

        if(message == "")
            tmpMessage << "Unspecified task exception occurred!";
        else
        {
            tmpMessage << "Task exception: ";
            tmpMessage << message;
        }

        static_cast<std::exception&>(*this) = std::runtime_error(tmpMessage.str());
    }

private:
    std::string message;
};

class TaskExceptionFunctionNotDefined : public std::runtime_error
{
public:
    TaskExceptionFunctionNotDefined(EnvironmentPtr envPtr [[maybe_unused]], std::string task)
        : std::runtime_error(""), taskName(task)
    {
        std::stringstream message;
        message << "Exception: task function in ";
        message << taskName;
        message << " not defined!";

        static_cast<std::exception&>(*this) = std::runtime_error(message.str());
    }

private:
    std::string taskName;
};

class TaskExceptionNotFound : public std::runtime_error
{
public:
    TaskExceptionNotFound(EnvironmentPtr envPtr [[maybe_unused]], std::string task)
        : std::runtime_error(""), taskID(task)
    {
        std::stringstream message;
        message << "Exception: task with ID ";
        message << taskID;
        message << " does not exist!";

        static_cast<std::exception&>(*this) = std::runtime_error(message.str());
    }

private:
    std::string taskID;
};
} // namespace SHOT