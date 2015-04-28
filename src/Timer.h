#pragma once
#include <chrono>
#include "Enums.h"
#include <string>

class Timer
{
	public:
		Timer();
		Timer(std::string timerName);
		Timer(std::string timerName, std::string desc);
		~Timer();

		//E_TimerTypes timerType;

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
