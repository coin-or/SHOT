#pragma once
#include <sstream>
#include <exception>

class TaskException: public std::exception
{
	public:

		TaskException(std::string msg) :
				message(msg)
		{
		}
		TaskException();

		const char * what() const throw ()
		{
			if (message == "") return "Unspecified task exception occurred!";
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

class TaskExceptionFunctionNotDefined: public std::exception
{
	public:

		TaskExceptionFunctionNotDefined(std::string task) :
				taskName(task)
		{
		}

		const char * what() const throw ()
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

class TaskExceptionNotFound: public std::exception
{
	public:

		TaskExceptionNotFound(std::string task) :
				taskID(task)
		{
		}

		const char * what() const throw ()
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
