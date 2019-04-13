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
#include "MIPSolver/MIPSolverOsiCbc.h"
#endif

#include "../Model/Problem.h"

namespace SHOT
{
class NLPSolverCuttingPlaneMinimax : public NLPSolverBase
{
public:
    NLPSolverCuttingPlaneMinimax(EnvironmentPtr envPtr, ProblemPtr problem);
    virtual ~NLPSolverCuttingPlaneMinimax();

    virtual void setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues);
    virtual void clearStartingPoint();

    virtual bool isObjectiveFunctionNonlinear();
    virtual int getObjectiveFunctionVariableIndex();

    virtual VectorDouble getVariableLowerBounds();
    virtual VectorDouble getVariableUpperBounds();

    virtual void updateVariableLowerBound(int variableIndex, double bound);
    virtual void updateVariableUpperBound(int variableIndex, double bound);

    virtual void saveProblemToFile(std::string fileName);

private:
    std::unique_ptr<IMIPSolver> LPSolver;
    ProblemPtr sourceProblem;
    VectorString variableNames;

    virtual double getSolution(int i);
    virtual VectorDouble getSolution();
    virtual double getObjectiveValue();

    virtual void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues);

    virtual void unfixVariables();

    virtual E_NLPSolutionStatus solveProblemInstance();

    virtual void saveOptionsToFile(std::string fileName);

    bool isProblemCreated;

    VectorDouble solution;
    double objectiveValue = NAN;

    bool createProblem(IMIPSolver* destinationProblem, ProblemPtr sourceProblem);
};
} // namespace SHOT