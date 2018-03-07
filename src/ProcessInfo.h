/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Enums.h"
#include "Structs.h"
#include "vector"
#include "map"
#include "Iteration.h"
#include "Timer.h"

#include "UtilityFunctions.h"

// Used for OSOutput
#include "cstdio"
#define HAVE_STDIO_H 1
#include "OSOutput.h"

#include "SHOTSettings.h"

#include "TaskHandler.h"

#include "OSResult.h"
#include "OSrLWriter.h"
#include "OSErrorClass.h"

#include "MIPSolver/IRelaxationStrategy.h"

#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"

class OptProblemOriginal;
class IMIPSolver;
class ILinesearchMethod;

#include "LinesearchMethod/ILinesearchMethod.h"

class ProcessInfo
{
  public:
    OSResult *osResult;
    OptProblemOriginal *originalProblem;

    IMIPSolver *MIPSolver;
    IRelaxationStrategy *relaxationStrategy;

    TaskHandler *tasks;

    void initializeResults(int numObj, int numVar, int numConstr);

    vector<double> primalSolution; // TODO remove
    //double lastObjectiveValue; // TODO remove
    vector<Iteration> iterations;
    vector<PrimalSolution> primalSolutions;
    vector<DualSolution> dualSolutions;

    vector<PrimalSolution> primalSolutionCandidates;
    vector<PrimalFixedNLPCandidate> primalFixedNLPCandidates;
    vector<DualSolution> dualSolutionCandidates;

    pair<double, double> getCorrectedObjectiveBounds();

    void addPrimalSolution(vector<double> pt, E_PrimalSolutionSource source, double objVal, int iter,
                           IndexValuePair maxConstrDev);
    void addPrimalSolution(vector<double> pt, E_PrimalSolutionSource source, double objVal, int iter);
    void addPrimalSolution(SolutionPoint pt, E_PrimalSolutionSource source);

    void addPrimalFixedNLPCandidate(vector<double> pt, E_PrimalNLPSource source, double objVal, int iter,
                                    IndexValuePair maxConstrDev);

    void addDualSolution(vector<double> pt, E_DualSolutionSource source, double objVal, int iter);
    void addDualSolution(SolutionPoint pt, E_DualSolutionSource source);
    void addDualSolution(DualSolution solution);
    void addPrimalSolutionCandidate(vector<double> pt, E_PrimalSolutionSource source, int iter);
    void addPrimalSolutionCandidates(vector<vector<double>> pts, E_PrimalSolutionSource source, int iter);

    void addPrimalSolutionCandidate(SolutionPoint pt, E_PrimalSolutionSource source);
    void addPrimalSolutionCandidates(std::vector<SolutionPoint> pts, E_PrimalSolutionSource source);

    void checkPrimalSolutionCandidates();
    void checkDualSolutionCandidates();

    bool isRelativeObjectiveGapToleranceMet();
    bool isAbsoluteObjectiveGapToleranceMet();

    void addDualSolutionCandidate(SolutionPoint pt, E_DualSolutionSource source);
    void addDualSolutionCandidates(std::vector<SolutionPoint> pts, E_DualSolutionSource source);
    void addDualSolutionCandidate(vector<double> pt, E_DualSolutionSource source, int iter);
    void addDualSolutionCandidate(DualSolution solution);

    std::pair<double, double> currentObjectiveBounds;
    double getAbsoluteObjectiveGap();
    double getRelativeObjectiveGap();
    void setObjectiveUpdatedByLinesearch(bool updated);
    bool getObjectiveUpdatedByLinesearch();

    int iterationCount;
    int iterLP;
    int iterQP;
    int iterFeasMILP;
    int iterOptMILP;
    int iterFeasMIQP;
    int iterOptMIQP;
    int iterFeasMIQCQP;
    int iterOptMIQCQP;

    int numNLPProbsSolved;
    int numPrimalFixedNLPProbsSolved;

    int itersWithStagnationMIP;         // TODO move to task
    int iterSignificantObjectiveUpdate; // TODO move to task
    int MIPIterationsWithoutNLPCall;    // TODO move to task
    double solTimeLastNLPCall;          // TODO move to task

    int iterLastPrimalBoundUpdate;
    int iterLastDualBoundUpdate;

    double timeLastDualBoundUpdate;

    int lastLazyAddedIter;

    int numOriginalInteriorPoints;

    int numFunctionEvals;
    int numGradientEvals;

    int numConstraintsRemovedInPresolve;
    int numVariableBoundsTightenedInPresolve;
    int numIntegerCutsAdded;

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

    E_TerminationReason terminationReason = E_TerminationReason::Error;
    E_SolutionStrategy usedSolutionStrategy = E_SolutionStrategy::None;
    ES_MIPSolver usedMIPSolver = ES_MIPSolver::None;
    ES_PrimalNLPSolver usedPrimalNLPSolver = ES_PrimalNLPSolver::None;

    std::string getOSrl();
    std::string getTraceResult();

    void createIteration();

    std::vector<shared_ptr<InteriorPoint>> interiorPts;

    std::vector<Hyperplane> hyperplaneWaitingList;

    std::vector<Hyperplane> addedHyperplanes;

    std::vector<std::vector<int>> integerCutWaitingList;

    std::vector<Timer> timers;

    void outputAlways(std::string message);
    void outputError(std::string message);
    void outputError(std::string message, std::string errormessage);
    void outputSummary(std::string message);
    void outputWarning(std::string message);
    void outputInfo(std::string message);
    void outputDebug(std::string message);
    void outputTrace(std::string message);
    void outputDetailedTrace(std::string message);

    ILinesearchMethod *linesearchMethod;

    ~ProcessInfo();

    static ProcessInfo &getInstance()
    {
        static ProcessInfo inst;
        return (inst);
    }

  private:
    bool objectiveUpdatedByLinesearch;

    bool checkPrimalSolutionPoint(PrimalSolution primalSol);

    ProcessInfo();
};
