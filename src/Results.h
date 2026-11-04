/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <map>
#include <memory>
#include <vector>
#include <optional>

#include "Environment.h"
#include "Iteration.h"
#include "SHOTConfig.h"
#include "Structs.h"

#include "tinyxml2.h"

namespace SHOT
{

class Variables;

class DllExport Results
{
public:
    Results(EnvironmentPtr envPtr);
    ~Results();

    VectorDouble primalSolution;
    std::vector<PrimalSolution> primalSolutions;
    std::map<E_PrimalSolutionSource, int> primalSolutionSourceStatistics;
    std::map<E_AuxiliaryVariableType, int> auxiliaryVariablesIntroduced;

    void addPrimalSolution(PrimalSolution solution);

    // The best primal solution that fulfils the constraints to Termination.ConstraintTolerance, see
    // useAccuratePrimalSolution()
    std::optional<PrimalSolution> accuratePrimalSolution;

    // Replaces an incumbent that does not fulfil the constraints to Termination.ConstraintTolerance with
    // accuratePrimalSolution, if its objective value is within the termination gap tolerances of the incumbent, and
    // otherwise puts it in the solution pool if that has room for more than the incumbent
    void useAccuratePrimalSolution();
    double getPrimalBound();
    void setPrimalBound(double value, bool resetStagnationCounters = true);

    std::vector<DualSolution> dualSolutions;
    void addDualSolution(DualSolution solution);
    double getCurrentDualBound();
    double getGlobalDualBound();
    void setDualBound(double value, bool forceGlobal = false);
    /// Forget the current dual bound, and optionally the global one, when their validity is lost
    void resetDualBounds(bool resetGlobal);

    // For minimization problems, the lower bound is the dual while the upper bound is the primal objective value
    // for maximization problems, the lower bound is the primal while the upper bound is the dual objective value
    double currentDualBound = NAN;
    double currentPrimalBound = NAN;

    double globalDualBound = NAN;

    // Whether the best primal solution has a variable at a bound that has replaced a missing bound when the problem was
    // read, in which case the gap is not considered closed
    bool isPrimalSolutionAtArtificialBound();

    bool isRelativeObjectiveGapToleranceMet();
    bool isAbsoluteObjectiveGapToleranceMet();

    double getAbsoluteGlobalObjectiveGap();
    double getRelativeGlobalObjectiveGap();

    double getAbsoluteCurrentObjectiveGap();
    double getRelativeCurrentObjectiveGap();

    void createIteration();
    IterationPtr getCurrentIteration();
    IterationPtr getPreviousIteration();
    std::optional<IterationPtr> getLastFeasibleIteration();
    std::vector<IterationPtr> iterations;
    int getNumberOfIterations();

    E_TerminationReason terminationReason = E_TerminationReason::NotTerminated;
    std::string terminationReasonDescription;

    E_ModelReturnStatus getModelReturnStatus();
    bool hasPrimalSolution();

    // A message that the convexity of the problem, or of its objective function or some of its constraints, has been
    // specified by the user instead of detected, in which case the global optimality of a solution depends on it. Empty
    // otherwise.
    std::string getUserSpecifiedConvexityDescription();

    E_SolutionStrategy usedSolutionStrategy = E_SolutionStrategy::None;

    ES_MIPSolver usedMIPSolver = ES_MIPSolver::NotUsed;
    ES_PrimalNLPSolver usedPrimalNLPSolver = ES_PrimalNLPSolver::NotUsed;
    std::string usedPrimalNLPSolverDescription = "";
    std::string usedPrimalGAMSNLPSolver = "";

    bool solutionIsGlobal = true;

    // Whether a relaxation of the problem, e.g. the convex bounding problem, has been infeasible, which proves that the
    // problem is infeasible also when the dual problem is no longer a relaxation
    bool isInfeasibilityProven = false;

    // Whether a primal bound has passed a dual bound that is regarded as global by more than the objective gap
    // tolerances, which means that the dual bound was not valid
    bool hasPrimalBoundPassedGlobalDualBound = false;

    std::string getResultsOSrL();
    std::string getResultsTrace();
    std::string getResultsSol();

    void savePrimalSolutionToFile(
        const PrimalSolution& solution, const VectorString& variables, const std::string& fileName);
    void savePrimalSolutionToFile(
        const PrimalSolution& solution, const Variables& variables, const std::string& fileName);

    void increaseAuxiliaryVariableCounter(E_AuxiliaryVariableType type);
    int getAuxiliaryVariableCounter(E_AuxiliaryVariableType type);

private:
    void notifyBoundsUpdated(double previousPrimalBound, double previousDualBound, double previousGlobalDualBound);

    // The largest violation of the constraints of the original problem in the point
    double getMaximumConstraintError(const VectorDouble& point);

    EnvironmentPtr env;
};

} // namespace SHOT
