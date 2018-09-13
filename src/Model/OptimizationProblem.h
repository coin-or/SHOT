/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "ModelStructs.h"
#include "Terms.h"
#include "ObjectiveFunction.h"
#include "NonlinearExpressions.h"
#include "Constraints.h"

#include <vector>
#include <string>
#include <memory>
#include <boost/optional.hpp>

namespace SHOT
{

struct OptimizationProblemProperties
{
    bool isValid = false; // Whether the values here are valid anymore

    bool isConvex = false;
    bool isNonconvex = false;

    bool isMINLPProblem = false;
    bool isNLPProblem = false;
    bool isMIQPProblem = false;
    bool isQPProblem = false;
    bool isMIQCQPProblem = false;
    bool isQCQPProblem = false;
    bool isMILPProblem = false;
    bool isLPProblem = false;

    int numberOfVariables = 0;
    int numberOfRealVariables = 0;
    int numberOfDiscreteVariables = 0; //Binary and integer variables
    int numberOfBinaryVariables = 0;
    int numberOfIntegerVariables = 0; //Not including binary variables
    int numberOfSemicontinuousVariables = 0;

    int numberOfNumericConstraints = 0;
    int numberOfLinearConstraints = 0;
    int numberOfQuadraticConstraints = 0;
    int numberOfNonlinearConstraints = 0;

    std::string name = "";
    std::string description = "";
};

class OptimizationProblem
{
  private:
    Variables allVariables;
    Variables realVariables;
    Variables binaryVariables;
    Variables integerVariables;
    Variables semicontinuousVariables;

    VectorDouble variableLowerBounds;
    VectorDouble variableUpperBounds;

    bool variablesUpdated = false;
    bool constraintsUpdated = false;
    bool objectiveUpdated = false;

    ObjectiveFunctionPtr objectiveFunction;

    NumericConstraints numericConstraints;
    LinearConstraints linearConstraints;
    QuadraticConstraints quadraticConstraints;
    NonlinearConstraints nonlinearConstraints;

    void updateVariables()
    {
        auto numVariables = allVariables.size();

        // Update bound vectors
        if (variableLowerBounds.size() != numVariables)
            variableLowerBounds.resize(numVariables);

        if (variableUpperBounds.size() != numVariables)
            variableUpperBounds.resize(numVariables);

        for (int i = 0; i < numVariables; ++i)
        {
            variableLowerBounds.at(i) = allVariables.at(i)->lowerBound;
            variableUpperBounds.at(i) = allVariables.at(i)->upperBound;
        }

        variablesUpdated = true;
    };

    void updateProperties()
    {
        properties.isConvex = true;
        properties.isNonconvex = false;
       
        properties.numberOfVariables = allVariables.size();
        properties.numberOfRealVariables = realVariables.size();
        properties.numberOfBinaryVariables = binaryVariables.size();
        properties.numberOfIntegerVariables = integerVariables.size();
        properties.numberOfDiscreteVariables = properties.numberOfBinaryVariables + properties.numberOfIntegerVariables;
        properties.numberOfSemicontinuousVariables = semicontinuousVariables.size();
 
        properties.numberOfNumericConstraints = numericConstraints.size(); 
        properties.numberOfLinearConstraints = linearConstraints.size(); 
        properties.numberOfQuadraticConstraints = quadraticConstraints.size(); 
        properties.numberOfNonlinearConstraints = nonlinearConstraints.size(); 

        bool isObjNonlinear = (objectiveFunction->properties.classification > E_ObjectiveFunctionClassification::Quadratic);
        bool isObjQuadratic = (objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic);
        bool areConstrsNonlinear = (properties.numberOfNonlinearConstraints > 0);
        bool areConstrsQuadratic = (properties.numberOfQuadraticConstraints > 0);       
        bool areVarsDiscrete = (properties.numberOfDiscreteVariables >0);
        
        if (areVarsDiscrete)
        {
            if (areConstrsNonlinear || isObjNonlinear)
                properties.isMINLPProblem = true;
            else if (areConstrsQuadratic)
                properties.isMIQCQPProblem = true;
            else if (isObjQuadratic)
                properties.isMIQPProblem = true;
            else
                properties.isMILPProblem = true;
        }
        else
        {
            if (areConstrsNonlinear || isObjNonlinear)
                properties.isNLPProblem = true;
            else if (areConstrsQuadratic)
                properties.isQCQPProblem = true;
            else if (isObjQuadratic)
                properties.isQPProblem = true;
            else
                properties.isLPProblem = true;
        }

	properties.isValid = true;
    };

  public:
    OptimizationProblem()
    {
    };

    virtual ~OptimizationProblem()
    {
    };
    /*
    // Copy constructor
    OptimizationProblem(const OptimizationProblem &sourceProblem)
    {
        //TODO
    }*/

    OptimizationProblemProperties properties;

    ObjectiveFunctionPtr getObjectiveFunction()
    {
        return objectiveFunction;
    };

    Variables getAllVariables()
    {
        return allVariables;
    };

    Variables getRealVariables()
    {
        return realVariables;
    };

    Variables getBinaryVariables()
    {
        return binaryVariables;
    };

    Variables getIntegerVariables()
    {
        return integerVariables;
    };

    Variables getSemicontinuousVariables()
    {
        return semicontinuousVariables;
    };

    void add(VariablePtr variable)
    {
        allVariables.push_back(variable);

        switch (variable->type)
        {
        case (E_VariableType::Real):
            realVariables.push_back(variable);
            break;
        case (E_VariableType::Binary):
            binaryVariables.push_back(variable);
            break;
        case (E_VariableType::Integer):
            integerVariables.push_back(variable);
            break;
        case (E_VariableType::Semicontinuous):
            semicontinuousVariables.push_back(variable);
            break;
        default:
            break;
        }
    };

    void add(LinearConstraintPtr constraint)
    {
        numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
        linearConstraints.push_back(constraint);
    };

    void add(QuadraticConstraintPtr constraint)
    {
        numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
        quadraticConstraints.push_back(constraint);
    };

    void add(NonlinearConstraintPtr constraint)
    {
        numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
        nonlinearConstraints.push_back(constraint);
    };

    void add(ObjectiveFunctionPtr objective)
    {
        objectiveFunction = objective;
        objectiveFunction->updateProperties();
    };

    template <class T>
    void add(std::vector<T> elements)
    {
        for (auto E : elements)
        {
            add(E);
        }
    };

    VariablePtr getVariable(int variableIndex)
    {
        if (variableIndex > allVariables.size())
        {
            throw new VariableNotFoundException(" with variableIndex " + std::to_string(variableIndex));
        }

        return allVariables.at(variableIndex);
    };

    double getVariableLowerBound(int variableIndex)
    {
        return allVariables.at(variableIndex)->lowerBound;
    };

    double getVariableUpperBound(int variableIndex)
    {
        return allVariables.at(variableIndex)->upperBound;
    };

    VectorDouble getVariableLowerBounds()
    {
        if (!variablesUpdated)
        {
            updateVariables();
        }

        return variableLowerBounds;
    };

    VectorDouble getVariableUpperBounds()
    {
        if (!variablesUpdated)
        {
            updateVariables();
        }

        return variableUpperBounds;
    };

    void setVariableLowerBound(int variableIndex, double bound)
    {
        allVariables.at(variableIndex)->lowerBound = bound;
        variablesUpdated = true;
    };

    void setVariableUpperBound(int variableIndex, double bound)
    {
        allVariables.at(variableIndex)->upperBound = bound;
        variablesUpdated = true;
    };

    void setVariableBounds(int variableIndex, double lowerBound, double upperBound)
    {
        allVariables.at(variableIndex)->lowerBound = lowerBound;
        allVariables.at(variableIndex)->upperBound = upperBound;
        variablesUpdated = true;
    };

    //Methods for constraints
    NumericConstraints getAllNumericConstraints()
    {
        return numericConstraints;
    };

    LinearConstraints getLinearConstraints()
    {
        return linearConstraints;
    };

    QuadraticConstraints getQuadraticConstraints()
    {
        return quadraticConstraints;
    };

    NonlinearConstraints getNonlinearConstraints()
    {
        return nonlinearConstraints;
    };

    boost::optional<NumericConstraintValue> getMostDeviatingNumericConstraint(const VectorDouble &point)
    {
        return (this->getMostDeviatingNumericConstraint(point, numericConstraints));
    };

    virtual boost::optional<NumericConstraintValue> getMostDeviatingNumericConstraint(const VectorDouble &point, NumericConstraints constraintSelection)
    {
        boost::optional<NumericConstraintValue> optional;
        double error = -1;

        for (auto C : constraintSelection)
        {
            auto constraintValue = C->calculateNumericValue(point);

            if (constraintValue.isFulfilled)
                continue;

            if (!optional) // No constraint with error found yet
            {
                optional = constraintValue;
                error = constraintValue.error;

            }
            else if (constraintValue.error > error)
            {
                optional = constraintValue;
                error = constraintValue.error;

            }
        }

        return optional;
    };

    template <class T>
    NumericConstraintValues getAllDeviatingConstraints(const VectorDouble &point, double tolerance, std::vector<T> constraintSelection)
    {
        NumericConstraintValues constraintValues;
        for (auto C : constraintSelection)
        {
            NumericConstraintValue constraintValue = C->calculateNumericValue(point);
            if (constraintValue.error > tolerance)
                constraintValues.push_back(constraintValue);
        }

        return constraintValues;
    };

    virtual NumericConstraintValues getAllDeviatingNumericConstraints(const VectorDouble &point, double tolerance)
    {
        return getAllDeviatingConstraints(point, tolerance, numericConstraints);
    };

    virtual NumericConstraintValues getAllDeviatingLinearConstraints(const VectorDouble &point, double tolerance)
    {
        return getAllDeviatingConstraints(point, tolerance, linearConstraints);
    };

    virtual NumericConstraintValues getAllDeviatingQuadraticConstraints(const VectorDouble &point, double tolerance)
    {
        return getAllDeviatingConstraints(point, tolerance, quadraticConstraints);
    };

    virtual NumericConstraintValues getAllDeviatingNonlinearConstraints(const VectorDouble &point, double tolerance)
    {
        return getAllDeviatingConstraints(point, tolerance, nonlinearConstraints);
    };

    virtual bool areLinearConstraintsFulfilled(VectorDouble point, double tolerance)
    {
        auto deviatingConstraints = getAllDeviatingLinearConstraints(point, tolerance);
        return (deviatingConstraints.size() == 0);
    };

    virtual bool areQuadraticConstraintsFulfilled(VectorDouble point, double tolerance)
    {
        auto deviatingConstraints = getAllDeviatingQuadraticConstraints(point, tolerance);
        return (deviatingConstraints.size() == 0);
    };

    virtual bool areNonlinearConstraintsFulfilled(VectorDouble point, double tolerance)
    {
        auto deviatingConstraints = getAllDeviatingNonlinearConstraints(point, tolerance);
        return (deviatingConstraints.size() == 0);
    };

    virtual bool areNumericConstraintsFulfilled(VectorDouble point, double tolerance)
    {
        auto deviatingConstraints = getAllDeviatingNumericConstraints(point, tolerance);
        return (deviatingConstraints.size() == 0);
    };

    virtual bool areIntegralityConstraintsFulfilled(VectorDouble point, double tolerance)
    {
        for (auto V : integerVariables)
        {
            if (abs(point.at(V->index) - round(point.at(V->index))) > tolerance)
                return false;
        }

        return true;
    };

    bool areVariableBoundsFulfilled(VectorDouble point, double tolerance)
    {
        for (int i = 0; i < properties.numberOfVariables; ++i)
        {
            if (point.at(i) - tolerance > allVariables.at(i)->upperBound)
            {
                return false;
            }
            if (point.at(i) + tolerance < allVariables.at(i)->lowerBound)
            {
                return false;
            }
        }

        return true;
    };

    friend std::ostream &operator<<(std::ostream &stream, const OptimizationProblem &problem)
    {
        stream << problem.objectiveFunction << '\n';

        if (problem.numericConstraints.size() > 0)
            stream << "subject to:\n";

        for (auto C : problem.numericConstraints)
        {
            stream << C << '\n';
        }

        stream << "variables:\n";

        for (auto V : problem.allVariables)
        {
            stream << V << '\n';
        }

        return stream;
    };
};

typedef std::shared_ptr<OptimizationProblem> OptimizationProblemPtr;

std::ostream &operator<<(std::ostream &stream, OptimizationProblemPtr problem)
{
    stream << *problem;
    return stream;
};

} // namespace SHOT
