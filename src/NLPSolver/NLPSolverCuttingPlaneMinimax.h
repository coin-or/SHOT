/**
    The Supporting Hyperplane Optimization Toolkit (SHOT).

    @author Andreas Lundell, Åbo Akademi University

    @section LICENSE
    This software is licensed under the Eclipse Public License 2.0.
    Please see the README and LICENSE files for more information.
 */

#pragma once

#include "NLPSolverBase.h"
#include "../Tasks/TaskAddHyperplanes.h"

#ifdef HAS_CPLEX
#include "MIPSolver/MIPSolverCplex.h"
#endif

#ifdef HAS_GUROBI
#include "MIPSolver/MIPSolverGurobi.h"
#endif

#ifdef HAS_CBC
#include "MIPSolver/MIPSolverCbc.h"
#endif

#include "../Model/Problem.h"

namespace SHOT
{
class NLPSolverCuttingPlaneMinimax : public NLPSolverBase
{
public:
    NLPSolverCuttingPlaneMinimax(EnvironmentPtr envPtr, ProblemPtr problem);
    ~NLPSolverCuttingPlaneMinimax() override;

    void setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues) override;
    void clearStartingPoint() override;

    virtual bool isObjectiveFunctionNonlinear();
    virtual int getObjectiveFunctionVariableIndex();

    VectorDouble getVariableLowerBounds() override;
    VectorDouble getVariableUpperBounds() override;

    void updateVariableLowerBound(int variableIndex, double bound) override;
    void updateVariableUpperBound(int variableIndex, double bound) override;

    void saveProblemToFile(std::string fileName) override;

    std::string getSolverDescription() override { return ("Built in minmax solver"); };

private:
    std::unique_ptr<IMIPSolver> LPSolver;
    ProblemPtr sourceProblem;
    VectorString variableNames;

    double getSolution(int i) override;
    VectorDouble getSolution() override;
    double getObjectiveValue() override;

    void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues) override;

    void unfixVariables() override;

    E_NLPSolutionStatus solveProblemInstance() override;

    void saveOptionsToFile(std::string fileName) override;

    VectorDouble solution;
    double objectiveValue = NAN;

    bool createProblem(IMIPSolver* destinationProblem, ProblemPtr sourceProblem);
};
} // namespace SHOT