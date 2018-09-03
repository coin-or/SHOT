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
    bool isQPProblem = false;
    bool isMIQPProblem = false;
    bool isMIQCQPProblem = false;

    int numberOfVariables = 0;
    int numberOfRealVariables = 0;
    int numberOfDiscreteVariables = 0; //Binary and integer variables
    int numberOfBinaryVariables = 0;
    int numberOfIntegerVariables = 0; //Not including binary variables

    int numberOfLinearConstraints = 0;
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
    Variables semicontinousVariables;

    bool constraintsInitialized = false;
    NumericConstraints numericConstraints;
    LinearConstraints linearConstraints;
    QuadraticConstraints quadraticConstraints;
    NonlinearConstraints nonlinearConstraints;

  public:
    OptimizationProblem();
    virtual ~OptimizationProblem();
    /*
    // Copy constructor
    OptimizationProblem(const OptimizationProblem &sourceProblem)
    {
        //TODO
    }*/

    OptimizationProblemProperties properties;
    void updateProperties();

    // Methods for variables
    Variables getAllVariables();
    Variables getRealVariables();
    Variables getBinaryVariables();
    Variables getIntegerVariables();
    Variables getSemicontinuousVariables();

    void addVariable(VariablePtr variable);
    void addVariables(Variables variables);
    VariablePtr getVariable(int variableIndex);

    //AuxilliaryVariables getAuxilliaryVariables();
    //VectorDouble getVariableLowerBounds();
    //VectorDouble getVariableUpperBounds();

    //double getVariableLowerBound(int variableIndex);
    //double getVariableUpperBound(int variableIndex);
    //PairDouble getVariableBounds(int variableIndex);

    //void setVariableUpperBound(int variableIndex, double upperBound);
    //void setVariableLowerBound(int variableIndex, double lowerBound);
    //void setVariableBounds(int variableIndex, double lowerBound, double upperBound);

    //Methods for constraints
    NumericConstraints getAllNumericConstraints();
    LinearConstraints getLinearConstraints();
    QuadraticConstraints getQuadraticConstraints();
    NonlinearConstraints getNonlinearConstraints();

    virtual void addConstraint(LinearConstraintPtr constraint);
    virtual void addConstraint(QuadraticConstraintPtr constraint);
    virtual void addConstraint(NonlinearConstraintPtr constraint);

    virtual boost::optional<NumericConstraintValue> getMostDeviatingNumericConstraint(const VectorDouble &point);
    virtual boost::optional<NumericConstraintValue> getMostDeviatingNumericConstraint(const VectorDouble &point, NumericConstraints);

    virtual NumericConstraintValues getAllDeviatingNumericConstraints(const VectorDouble &point, double tolerance);

    /*
    virtual bool areLinearConstraintsFulfilled(VectorDouble point);
    virtual bool areLinearConstraintsFulfilled(VectorDouble point, double tolerance);

    virtual bool areNonlinearConstraintsFulfilled(VectorDouble point);
    virtual bool areNonlinearConstraintsFulfilled(VectorDouble point, double tolerance);

    virtual bool areIntegralityConstraintsFulfilled(VectorDouble point, double tolerance);
    virtual bool areVariableBoundsFulfilled(VectorDouble point, double tolerance);*/
};
} // namespace SHOT
