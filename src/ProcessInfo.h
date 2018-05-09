/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "SHOTConfig.h"
#include "UtilityFunctions.h"
#include "SHOTSettings.h"
#include "Enums.h"
#include "Structs.h"
#include "Output.h"
#include "Iteration.h"
#include "Timer.h"
#include "Model.h"

#include "OSResult.h"
#include "OSrLWriter.h"
#include "OSErrorClass.h"

#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"
#include "OptProblems/OptProblemOriginal.h"

#include "MIPSolver/IRelaxationStrategy.h"
#include "LinesearchMethod/ILinesearchMethod.h"

#ifdef HAS_GAMS
#include "gmomcc.h"
#endif

class Iteration;

class ProcessInfo
{
  public:
    std::unique_ptr<OSResult> osResult;

    IRelaxationStrategy *relaxationStrategy;
    ILinesearchMethod *linesearchMethod;

    void initializeResults(int numObj, int numVar, int numConstr);

    std::vector<double> primalSolution; // TODO remove

    std::vector<Iteration> iterations;
    std::vector<PrimalSolution> primalSolutions;
    std::vector<DualSolution> dualSolutions;

    std::vector<PrimalSolution> primalSolutionCandidates;
    std::vector<PrimalFixedNLPCandidate> primalFixedNLPCandidates;
    std::vector<DualSolution> dualSolutionCandidates;

    DoublePair getCorrectedObjectiveBounds();

    void addPrimalSolution(std::vector<double> pt, E_PrimalSolutionSource source, double objVal, int iter,
                           IndexValuePair maxConstrDev);
    void addPrimalSolution(std::vector<double> pt, E_PrimalSolutionSource source, double objVal, int iter);
    void addPrimalSolution(SolutionPoint pt, E_PrimalSolutionSource source);

    void addPrimalFixedNLPCandidate(std::vector<double> pt, E_PrimalNLPSource source, double objVal, int iter,
                                    IndexValuePair maxConstrDev);

    void addDualSolution(std::vector<double> pt, E_DualSolutionSource source, double objVal, int iter);
    void addDualSolution(SolutionPoint pt, E_DualSolutionSource source);
    void addDualSolution(DualSolution solution);
    void addPrimalSolutionCandidate(std::vector<double> pt, E_PrimalSolutionSource source, int iter);
    void addPrimalSolutionCandidates(std::vector<std::vector<double>> pts, E_PrimalSolutionSource source, int iter);

    void addPrimalSolutionCandidate(SolutionPoint pt, E_PrimalSolutionSource source);
    void addPrimalSolutionCandidates(std::vector<SolutionPoint> pts, E_PrimalSolutionSource source);

    void checkPrimalSolutionCandidates();
    void checkDualSolutionCandidates();

    bool isRelativeObjectiveGapToleranceMet();
    bool isAbsoluteObjectiveGapToleranceMet();

    void addDualSolutionCandidate(SolutionPoint pt, E_DualSolutionSource source);
    void addDualSolutionCandidates(std::vector<SolutionPoint> pts, E_DualSolutionSource source);
    void addDualSolutionCandidate(std::vector<double> pt, E_DualSolutionSource source, int iter);
    void addDualSolutionCandidate(DualSolution solution);

    double getAbsoluteObjectiveGap();
    double getRelativeObjectiveGap();
    void setObjectiveUpdatedByLinesearch(bool updated);
    bool getObjectiveUpdatedByLinesearch();

    std::vector<int> itersSolvedAsECP;

    void createTimer(std::string name, std::string description);
    void startTimer(std::string name);
    void stopTimer(std::string name);
    void restartTimer(std::string name);
    double getElapsedTime(std::string name);

    double getPrimalBound();
    void setPrimalBound(double value);
    double getDualBound();
    void setDualBound(double value);

    Iteration *getCurrentIteration();
    Iteration *getPreviousIteration();

    E_TerminationReason terminationReason = E_TerminationReason::None;
    E_SolutionStrategy usedSolutionStrategy = E_SolutionStrategy::None;

    ES_MIPSolver usedMIPSolver = ES_MIPSolver::None;
    ES_PrimalNLPSolver usedPrimalNLPSolver = ES_PrimalNLPSolver::None;

    std::string getOSrl();
    std::string getTraceResult();

    void createIteration();

    std::vector<std::shared_ptr<InteriorPoint>> interiorPts;

    std::vector<Hyperplane> hyperplaneWaitingList;

    std::vector<Hyperplane> addedHyperplanes;

    std::vector<std::vector<int>> integerCutWaitingList;

    std::vector<Timer> timers;

#ifdef HAS_GAMS
    gmoHandle_t GAMSModelingObject;
#endif

    ~ProcessInfo();

    ProcessInfo(EnvironmentPtr envPtr);

  private:
    bool objectiveUpdatedByLinesearch;

    bool checkPrimalSolutionPoint(PrimalSolution primalSol);

    EnvironmentPtr env;
};