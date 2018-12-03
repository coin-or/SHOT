/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "vector"
#include "../Enums.h"
#include "OSInstance.h"
#include "../ProcessInfo.h"
#include "../OptProblems/OptProblemOriginal.h"
#include "../OptProblems/OptProblemOriginalLinearObjective.h"
#include "../OptProblems/OptProblemOriginalQuadraticObjective.h"
#include "../OptProblems/OptProblemOriginalNonlinearObjective.h"
#include "../Structs.h"
#include "../Model/Problem.h"

namespace SHOT
{
class IMIPSolver
{
  public:
    virtual void checkParameters() = 0;

    virtual bool createLinearProblem(OptProblem *origProblem) = 0;
    virtual bool createLinearProblem(ProblemPtr sourceProblem) = 0;

    virtual void addVariable(int index, std::string name, E_VariableType type, double lowerBound, double upperBound) = 0;

    virtual void initializeObjective() = 0;
    virtual void addLinearTermToObjective(double coefficient, int variableIndex) = 0;
    virtual void addQuadraticTermToObjective(double coefficient, int firstVariableIndex, int secondVariableIndex) = 0;
    virtual void finalizeObjective(bool isMinimize, double constant = 0.0) = 0;

    virtual void initializeConstraint() = 0;
    virtual void addLinearTermToConstraint(double coefficient, int variableIndex) = 0;
    virtual void addQuadraticTermToConstraint(double coefficient, int firstVariableIndex, int secondVariableIndex) = 0;
    virtual void finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant = 0.0) = 0;

    virtual void initializeSolverSettings() = 0;

    virtual VectorDouble getVariableSolution(int solIdx) = 0;
    virtual int getNumberOfSolutions() = 0;

    virtual void activateDiscreteVariables(bool activate) = 0;
    virtual bool getDiscreteVariableStatus() = 0;
    virtual E_ProblemSolutionStatus solveProblem() = 0;
    virtual E_ProblemSolutionStatus getSolutionStatus() = 0;
    virtual double getObjectiveValue() = 0;

    virtual double getDualObjectiveValue() = 0;

    virtual double getObjectiveValue(int solIdx) = 0;

    virtual int increaseSolutionLimit(int increment) = 0;
    virtual void setSolutionLimit(long limit) = 0;
    virtual int getSolutionLimit() = 0;

    virtual void writeProblemToFile(std::string filename) = 0;
    virtual void writePresolvedToFile(std::string filename) = 0;

    virtual std::vector<SolutionPoint> getAllVariableSolutions() = 0;
    virtual int addLinearConstraint(std::vector<PairIndexValue> elements, double constant) = 0;
    virtual int addLinearConstraint(std::vector<PairIndexValue> elements, double constant, bool isGreaterThan) = 0;

    virtual void setTimeLimit(double seconds) = 0;

    virtual void setCutOff(double cutOff) = 0;

    virtual void addMIPStart(VectorDouble point) = 0;
    virtual void deleteMIPStarts() = 0;

    virtual void fixVariable(int varIndex, double value) = 0;
    virtual void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues) = 0;
    virtual void unfixVariables() = 0;

    virtual void updateVariableBound(int varIndex, double lowerBound, double upperBound) = 0;

    virtual PairDouble getCurrentVariableBounds(int varIndex) = 0;

    virtual void presolveAndUpdateBounds() = 0;
    virtual std::pair<VectorDouble, VectorDouble> presolveAndGetNewBounds() = 0;

    virtual void createHyperplane(Hyperplane hyperplane) = 0;
    virtual void createIntegerCut(VectorInteger binaryIndexes) = 0;
    virtual void createInteriorHyperplane(Hyperplane hyperplane) = 0;

    virtual boost::optional<std::pair<std::vector<PairIndexValue>, double>> createHyperplaneTerms(Hyperplane hyperplane) = 0;

    virtual bool supportsQuadraticObjective() = 0;
    virtual bool supportsQuadraticConstraints() = 0;

    virtual std::vector<GeneratedHyperplane> *getGeneratedHyperplanes() = 0;

    virtual void updateNonlinearObjectiveFromPrimalDualBounds() = 0;

    virtual int getNumberOfExploredNodes() = 0;
    virtual int getNumberOfOpenNodes() = 0;

    virtual ~IMIPSolver(){};

  protected:
};
} // namespace SHOT