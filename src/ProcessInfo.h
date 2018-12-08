/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Shared.h"
#include "SHOTConfig.h"

#include "MIPSolver/IRelaxationStrategy.h"
#include "LinesearchMethod/ILinesearchMethod.h"

#ifdef HAS_GAMS
#include "gmomcc.h"
#endif

namespace SHOT
{
class Iteration;

class IRelaxationStrategy;

class ProcessInfo
{
  public:
    std::unique_ptr<OSResult> osResult;

    std::shared_ptr<IRelaxationStrategy> relaxationStrategy;
    std::shared_ptr<ILinesearchMethod> linesearchMethod;

    void initializeResults(int numObj, int numVar, int numConstr);

    VectorDouble primalSolution; // TODO remove

    std::vector<Iteration> iterations;
    std::vector<PrimalSolution> primalSolutions;
    std::vector<DualSolution> dualSolutions;

    std::vector<PrimalSolution> primalSolutionCandidates;
    std::vector<PrimalFixedNLPCandidate> primalFixedNLPCandidates;
    std::vector<DualSolution> dualSolutionCandidates;

    PairDouble getCorrectedObjectiveBounds();

    void addPrimalFixedNLPCandidate(VectorDouble pt, E_PrimalNLPSource source, double objVal, int iter,
                                    PairIndexValue maxConstrDev);

    void addDualSolution(DualSolution solution);
    void addDualSolutionCandidate(DualSolution solution);
    void checkDualSolutionCandidates();

    void addPrimalSolutionCandidate(VectorDouble pt, E_PrimalSolutionSource source, int iter);
    void addPrimalSolutionCandidates(std::vector<VectorDouble> pts, E_PrimalSolutionSource source, int iter);

    void addPrimalSolutionCandidate(SolutionPoint pt, E_PrimalSolutionSource source);
    void addPrimalSolutionCandidates(std::vector<SolutionPoint> pts, E_PrimalSolutionSource source);

    void checkPrimalSolutionCandidates();

    bool isRelativeObjectiveGapToleranceMet();
    bool isAbsoluteObjectiveGapToleranceMet();

    double getAbsoluteObjectiveGap();
    double getRelativeObjectiveGap();
    void setObjectiveUpdatedByLinesearch(bool updated);
    bool getObjectiveUpdatedByLinesearch();

    VectorInteger itersSolvedAsECP;

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

    std::vector<VectorInteger> integerCutWaitingList;

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

} // namespace SHOT