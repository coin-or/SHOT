/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Environment.h"
#include "../Enums.h"
#include "../Structs.h"

#include <map>
#include <optional>
#include <string>
#include <vector>
#include <utility>

namespace SHOT
{

class IMIPSolver
{
public:
    virtual ~IMIPSolver() = default;

    virtual bool initializeProblem() = 0;
    virtual void checkParameters() = 0;

    virtual bool addVariable(
        std::string name, E_VariableType type, double lowerBound, double upperBound, double semiBound)
        = 0;

    virtual bool initializeObjective() = 0;
    virtual bool addLinearTermToObjective(double coefficient, int variableIndex) = 0;
    virtual bool addQuadraticTermToObjective(double coefficient, int firstVariableIndex, int secondVariableIndex) = 0;
    virtual bool finalizeObjective(bool isMinimize, double constant = 0.0) = 0;

    virtual bool initializeConstraint() = 0;
    virtual bool addLinearTermToConstraint(double coefficient, int variableIndex) = 0;
    virtual bool addQuadraticTermToConstraint(double coefficient, int firstVariableIndex, int secondVariableIndex) = 0;
    virtual bool finalizeConstraint(std::string name, double valueLHS, double valueRHS, double constant = 0.0) = 0;

    virtual bool finalizeProblem() = 0;

    virtual void initializeSolverSettings() = 0;

    virtual VectorDouble getVariableSolution(int solIdx) = 0;
    virtual int getNumberOfSolutions() = 0;

    virtual E_DualProblemClass getProblemClass() = 0;

    virtual void activateDiscreteVariables(bool activate) = 0;
    virtual bool getDiscreteVariableStatus() = 0;

    virtual void executeRelaxationStrategy() = 0;

    virtual E_ProblemSolutionStatus solveProblem() = 0;
    virtual bool repairInfeasibility() = 0;

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
    virtual int addLinearConstraint(std::map<int, double>& elements, double constant, std::string name) = 0;
    virtual int addLinearConstraint(
        const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan)
        = 0;
    virtual int addLinearConstraint(
        const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan, bool allowRepair)
        = 0;

    virtual bool addSpecialOrderedSet(E_SOSType type, VectorInteger variableIndexes, VectorDouble variableWeights = {})
        = 0;

    virtual void setTimeLimit(double seconds) = 0;

    virtual void setCutOff(double cutOff) = 0;
    virtual void setCutOffAsConstraint(double cutOff) = 0;

    virtual void addMIPStart(VectorDouble point) = 0;
    virtual void deleteMIPStarts() = 0;

    virtual void fixVariable(int varIndex, double value) = 0;
    virtual void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues) = 0;
    virtual void unfixVariables() = 0;

    virtual void updateVariableBound(int varIndex, double lowerBound, double upperBound) = 0;
    virtual void updateVariableLowerBound(int varIndex, double lowerBound) = 0;
    virtual void updateVariableUpperBound(int varIndex, double upperBound) = 0;

    virtual PairDouble getCurrentVariableBounds(int varIndex) = 0;

    virtual void presolveAndUpdateBounds() = 0;
    virtual std::pair<VectorDouble, VectorDouble> presolveAndGetNewBounds() = 0;

    virtual bool createHyperplane(HyperplanePtr hyperplane) = 0;
    virtual bool createInteriorHyperplane(HyperplanePtr hyperplane) = 0;
    virtual bool createIntegerCut(IntegerCut& integerCut) = 0;

    virtual std::optional<std::pair<std::map<int, double>, double>> createHyperplaneTerms(HyperplanePtr hyperplane) = 0;

    virtual bool supportsQuadraticObjective() = 0;
    virtual bool supportsQuadraticConstraints() = 0;

    virtual double getUnboundedVariableBoundValue() = 0;

    virtual int getNumberOfExploredNodes() = 0;
    virtual int getNumberOfOpenNodes() = 0;

    virtual int getNumberOfVariables() = 0;

    virtual bool hasDualAuxiliaryObjectiveVariable() = 0;
    virtual int getDualAuxiliaryObjectiveVariableIndex() = 0;
    virtual void setDualAuxiliaryObjectiveVariableIndex(int index) = 0;

    virtual std::string getConstraintIdentifier(E_HyperplaneSource source) = 0;

    virtual std::string getSolverVersion() = 0;

protected:
};
} // namespace SHOT