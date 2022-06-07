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
    double getPrimalBound();
    void setPrimalBound(double value);

    std::vector<DualSolution> dualSolutions;
    void addDualSolution(DualSolution solution);
    double getCurrentDualBound();
    double getGlobalDualBound();
    void setDualBound(double value);

    // For minimization problems, the lower bound is the dual while the upper bound is the primal objective value
    // for maximization problems, the lower bound is the primal while the upper bound is the dual objective value
    double currentDualBound = NAN;
    double currentPrimalBound = NAN;

    double globalDualBound;

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

    E_TerminationReason terminationReason = E_TerminationReason::None;
    std::string terminationReasonDescription;

    E_ModelReturnStatus getModelReturnStatus();
    bool hasPrimalSolution();

    E_SolutionStrategy usedSolutionStrategy = E_SolutionStrategy::None;

    ES_MIPSolver usedMIPSolver = ES_MIPSolver::None;
    ES_PrimalNLPSolver usedPrimalNLPSolver = ES_PrimalNLPSolver::None;
    std::string usedPrimalNLPSolverDescription = "";

    bool solutionIsGlobal = true;

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
    EnvironmentPtr env;
};

} // namespace SHOT