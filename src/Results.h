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

#include "OSResult.h"
#include "OSrLWriter.h"

#include "tinyxml2.h"

namespace SHOT
{

class Results
{
public:
    Results(EnvironmentPtr envPtr);
    ~Results();

    std::unique_ptr<OSResult> osResult;

    void initializeResults(int numObj, int numVar, int numConstr);

    VectorDouble primalSolution;
    std::vector<PrimalSolution> primalSolutions;

    void addPrimalSolution(PrimalSolution solution);
    double getPrimalBound();
    void setPrimalBound(double value);

    std::vector<DualSolution> dualSolutions;
    void addDualSolution(DualSolution solution);
    double getDualBound();
    void setDualBound(double value);

    // Contains the objective bounds. The first is the lower and the second is the upper bound.
    // For minimization problems, the lower bound is the dual while the upper bound is the primal objective value
    // for maximization problems, the lower bound is the primal while the upper bound is the dual objective value
    double currentDualBound;
    double currentPrimalBound;

    bool isRelativeObjectiveGapToleranceMet();
    bool isAbsoluteObjectiveGapToleranceMet();

    double getAbsoluteObjectiveGap();
    double getRelativeObjectiveGap();

    void createIteration();
    IterationPtr getCurrentIteration();
    IterationPtr getPreviousIteration();
    std::vector<IterationPtr> iterations;

    E_TerminationReason terminationReason = E_TerminationReason::None;
    std::string terminationReasonDescription;

    E_SolutionStrategy usedSolutionStrategy = E_SolutionStrategy::None;

    ES_MIPSolver usedMIPSolver = ES_MIPSolver::None;
    ES_PrimalNLPSolver usedPrimalNLPSolver = ES_PrimalNLPSolver::None;

    std::string getResultsOSrL();
    std::string getResultsTrace();

    void savePrimalSolutionToFile(
        const PrimalSolution& solution, const VectorString& variables, const std::string& fileName);
    void savePrimalSolutionToFile(
        const PrimalSolution& solution, const Variables& variables, const std::string& fileName);

private:
    EnvironmentPtr env;
};

} // namespace SHOT