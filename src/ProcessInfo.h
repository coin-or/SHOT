#pragma once
#include "Enums.h"
#include <vector>
#include <map>
#include "Iteration.h"
#include "Timer.h"

#include "SHOTSettings.h"

#include "TaskHandler.h"

#include "CoinMessageHandler.hpp"
#include "OSResult.h"
#include "OSrLWriter.h"
#include "OSErrorClass.h"

#include "MILPSolver/IRelaxationStrategy.h"
class OptProblemOriginal;
class IMILPSolver;
//class TaskHandler;

struct InteriorPoint
{
		vector<double> point;
		int NLPSolver;
};

struct PrimalSolution
{
		vector<double> point;
		E_PrimalSolutionSource sourceType;
		double objValue;
		int iterFound;
		IndexValuePair maxDevatingConstraint;
};

struct DualSolution
{
		vector<double> point;
		E_DualSolutionSource sourceType;
		double objValue;
		int iterFound;
};

class ProcessInfo
{
	public:
		static ProcessInfo* getInstance();

		OSResult *osResult;
		OptProblemOriginal *originalProblem;

		//TaskHandler *tasks;

		CoinMessageHandler logger;
		IMILPSolver *MILPSolver;
		IRelaxationStrategy *relaxationStrategy;
		TaskHandler *tasks;

		void initializeResults(int numObj, int numVar, int numConstr);

		~ProcessInfo()
		{
			instanceFlag = false;
		}

		vector<double> primalSolution; // TODO remove
		//double lastObjectiveValue; // TODO remove
		vector<Iteration> iterations;
		vector<PrimalSolution> primalSolutions;
		vector<DualSolution> dualSolutions;

		vector<PrimalSolution> primalSolutionCandidates;
		vector<DualSolution> dualSolutionCandidates;

		pair<double, double> getCorrectedObjectiveBounds();

		void addPrimalSolution(vector<double> pt, E_PrimalSolutionSource source, double objVal, int iter,
				IndexValuePair maxConstrDev);
		void addPrimalSolution(vector<double> pt, E_PrimalSolutionSource source, double objVal, int iter);
		void addDualSolution(vector<double> pt, E_DualSolutionSource source, double objVal, int iter);
		void addPrimalSolution(SolutionPoint pt, E_PrimalSolutionSource source);
		void addDualSolution(SolutionPoint pt, E_DualSolutionSource source);

		void addPrimalSolutionCandidate(vector<double> pt, E_PrimalSolutionSource source, int iter);
		void addPrimalSolutionCandidates(vector<vector<double>> pts, E_PrimalSolutionSource source, int iter);

		void addPrimalSolutionCandidate(SolutionPoint pt, E_PrimalSolutionSource source);
		void addPrimalSolutionCandidates(std::vector<SolutionPoint> pts, E_PrimalSolutionSource source);

		void addDualSolutionCandidate(SolutionPoint pt, E_DualSolutionSource source);
		void addDualSolutionCandidates(std::vector<SolutionPoint> pts, E_DualSolutionSource source);
		void addDualSolutionCandidate(vector<double> pt, E_DualSolutionSource source, int iter);

		std::pair<double, double> currentObjectiveBounds;
		double getAbsoluteObjectiveGap();
		double getRelativeObjectiveGap();

		int iterationCount;
		int iterLP;
		int iterFeasMILP;
		int iterOptMILP;

		int itersWithStagnationMILP; // TODO move to task
		int iterSignificantObjectiveUpdate; // TODO move to task
		int itersMILPWithoutNLPCall; // TODO move to task
		double solTimeLastNLPCall; // TODO move to task

		int iterLastPrimalBoundUpdate;
		int iterLastDualBoundUpdate;

		int numOriginalInteriorPoints;

		std::vector<int> itersSolvedAsECP;

		//double getLastMaxDeviation();
		void setOriginalProblem(OptProblemOriginal *problem);

		void createTimer(string name, string description);
		void startTimer(string name);
		void stopTimer(string name);
		void restartTimer(string name);
		double getElapsedTime(string name);

		double getPrimalBound();
		double getDualBound();

		Iteration *getCurrentIteration();
		Iteration *getPreviousIteration();

		E_TerminationReason terminationReason;

		std::string getOSrl();
		std::string getTraceResult();

		void createIteration();

		std::vector<InteriorPoint> interiorPts;

		std::vector<std::pair<int, std::vector<double>>>hyperplaneWaitingList;

		std::vector<Timer> timers;

		private:
		static bool instanceFlag;
		static ProcessInfo *single;
		SHOTSettings::Settings *settings;

		ProcessInfo();

	};
