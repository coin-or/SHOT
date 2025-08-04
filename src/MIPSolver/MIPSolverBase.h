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
#include "IMIPSolver.h"

#include "IRelaxationStrategy.h"
#include "RelaxationStrategyStandard.h"
#include "RelaxationStrategyNone.h"

#include <map>
#include <optional>
#include <utility>
#include <cassert>
#include <iostream>

namespace SHOT
{
class MIPSolverBase
{
private:
    VectorInteger fixedVariableIndexes;
    std::vector<PairDouble> fixedVariableOriginalBounds;

    bool dualAuxiliaryObjectiveVariableDefined = false;
    int dualAuxiliaryObjectiveVariableIndex = -1;
    int constraintCounter = 0;

    bool warningMessageShownLargeRHS = false;

protected:
    int numberOfVariables = 0;
    int numberOfConstraints = 0;
    bool isMinimizationProblem;
    bool isProblemDiscrete = false;
    std::vector<E_VariableType> variableTypes;
    VectorDouble variableLowerBounds;
    VectorDouble variableUpperBounds;
    VectorString variableNames;

    bool cutOffConstraintDefined = false;
    int cutOffConstraintIndex;

    bool hasQuadraticObjective = false;
    bool hasQudraticConstraint = false;

public:
    ~MIPSolverBase();

    virtual bool createHyperplane(HyperplanePtr hyperplane);

    virtual bool createInteriorHyperplane(HyperplanePtr hyperplane);

    std::optional<std::pair<std::map<int, double>, double>> createHyperplaneTerms(HyperplanePtr hyperplane);

    virtual void setCutOffAsConstraint(double cutOff) = 0;

    virtual E_DualProblemClass getProblemClass();
    virtual bool getDiscreteVariableStatus();

    virtual void executeRelaxationStrategy();

    virtual std::vector<SolutionPoint> getAllVariableSolutions();
    virtual int getNumberOfSolutions() = 0;
    virtual VectorDouble getVariableSolution(int i) = 0;
    virtual double getObjectiveValue(int i) = 0;
    virtual double getObjectiveValue();

    virtual void presolveAndUpdateBounds();
    virtual std::pair<VectorDouble, VectorDouble> presolveAndGetNewBounds() = 0;

    virtual PairDouble getCurrentVariableBounds(int varIndex) = 0;

    virtual void fixVariable(int varIndex, double value) = 0;
    virtual void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues);
    virtual void unfixVariables();
    virtual void updateVariableBound(int varIndex, double lowerBound, double upperBound) = 0;

    virtual int addLinearConstraint(std::map<int, double>& elements, double constant, std::string name) = 0;
    virtual int addLinearConstraint(
        const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan)
        = 0;
    virtual int addLinearConstraint(
        const std::map<int, double>& elements, double constant, std::string name, bool isGreaterThan, bool allowRepair)
        = 0;

    virtual bool addSpecialOrderedSet(E_SOSType type, VectorInteger variableIndexes, VectorDouble variableWeights = {})
        = 0;

    virtual void activateDiscreteVariables(bool activate) = 0;

    virtual int getNumberOfExploredNodes() = 0;
    virtual int getNumberOfOpenNodes();

    virtual int getNumberOfVariables() { return numberOfVariables; }

    virtual bool hasDualAuxiliaryObjectiveVariable() { return dualAuxiliaryObjectiveVariableDefined; };
    virtual int getDualAuxiliaryObjectiveVariableIndex()
    {
        assert(dualAuxiliaryObjectiveVariableDefined);
        assert(dualAuxiliaryObjectiveVariableIndex >= 0);
        return dualAuxiliaryObjectiveVariableIndex;
    };

    virtual void setDualAuxiliaryObjectiveVariableIndex(int index)
    {
        dualAuxiliaryObjectiveVariableIndex = index;
        dualAuxiliaryObjectiveVariableDefined = true;
    };

    virtual inline std::string getConstraintIdentifier(E_HyperplaneSource source)
    {
        std::string identifier = "";

        switch(source)
        {
        case E_HyperplaneSource::MIPOptimalRootsearch:
            identifier = "H_RS_OPT";
            break;
        case E_HyperplaneSource::MIPSolutionPoolRootsearch:
            identifier = "H_RS_POOL";
            break;
        case E_HyperplaneSource::LPRelaxedRootsearch:
            identifier = "H_RS_REL";
            break;
        case E_HyperplaneSource::MIPOptimalSolutionPoint:
            identifier = "H_SP_OPT";
            break;
        case E_HyperplaneSource::MIPSolutionPoolSolutionPoint:
            identifier = "H_SP_POOL";
            break;
        case E_HyperplaneSource::LPRelaxedSolutionPoint:
            identifier = "H_SP_REL";
            break;
        case E_HyperplaneSource::LPFixedIntegers:
            identifier = "H_FIX";
            break;
        case E_HyperplaneSource::PrimalSolutionSearch:
            identifier = "H_PRIM";
            break;
        case E_HyperplaneSource::PrimalSolutionSearchInteriorObjective:
            identifier = "H_PRIM_IO";
            break;
        case E_HyperplaneSource::InteriorPointSearch:
            identifier = "H_IP";
            break;
        case E_HyperplaneSource::MIPCallbackRelaxed:
            identifier = "H_CB_R";
            break;
        case E_HyperplaneSource::ObjectiveRootsearch:
            identifier = "H_RS_OBJ";
        case E_HyperplaneSource::ObjectiveCuttingPlane:
            identifier = "H_CP_OBJ";
            break;
        case E_HyperplaneSource::External:
            identifier = "H_EXT";
            break;
        default:
            break;
        }

        return (identifier);
    }

    std::vector<int> integerCuts; // Contains the constraint indexes that are integerCuts
    std::vector<bool> allowRepairOfConstraint; // Whether to allow the added cuts to be relaxed

    int prevSolutionLimit = 1;

    bool discreteVariablesActivated;
    bool cachedSolutionHasChanged;
    bool modelUpdated = true;

    bool isVariablesFixed = false;
    bool alreadyInitialized = false;

    std::vector<SolutionPoint> lastSolutions;

    std::unique_ptr<IRelaxationStrategy> relaxationStrategy;

    EnvironmentPtr env;
};
} // namespace SHOT