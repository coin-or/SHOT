
/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Structs.h"

#include <utility>
#include <vector>
#include <memory>
#include <list>
#include <string>
#include <sstream>
#include <exception>
#include <iostream>
#include <limits>

#include "../../ThirdParty/mcpp/ffunc.hpp"

namespace SHOT
{

// Forward declaration
class OptimizationProblem;
typedef std::shared_ptr<OptimizationProblem> OptimizationProblemPtr;

typedef std::vector<PairIndexValue> SparseVector;
typedef std::vector<PairCoordinateValue> SparseMatrix;

typedef mc::FFGraph FactorableFunctionGraph;
typedef std::shared_ptr<FactorableFunctionGraph> FactorableFunctionGraphPtr;
typedef mc::FFVar FactorableFunction;
typedef std::shared_ptr<FactorableFunction> FactorableFunctionPtr;
typedef std::vector<FactorableFunctionPtr> FactorableFunctions;

std::ostream &operator<<(std::ostream &stream, FactorableFunctionGraphPtr graph)
{
    stream << *graph;
    return stream;
};

std::ostream &operator<<(std::ostream &stream, FactorableFunctionPtr function)
{
    stream << *function;
    return stream;
};

// Variable definitions

enum class E_VariableType
{
    Real,
    Binary,
    Integer,
    Semicontinuous
};

class Variable
{
  public:
    std::string name = "";
    int index;

    E_VariableType type;
    OptimizationProblemPtr ownerProblem;

    double upperBound;
    double lowerBound;

    bool isNonlinear = false;
    FactorableFunction factorableFunctionVariable;

    Variable() : lowerBound(-std::numeric_limits<double>::infinity()),
                 upperBound(std::numeric_limits<double>::infinity()){};

    Variable(std::string variableName, int variableIndex, E_VariableType variableType, double LB, double UB) : name(variableName),
                                                                                                               index(variableIndex),
                                                                                                               type(variableType),
                                                                                                               lowerBound(LB),
                                                                                                               upperBound(UB){};

    Variable(std::string variableName, int variableIndex, E_VariableType variableType) : name(variableName),
                                                                                         index(variableIndex),
                                                                                         type(variableType),
                                                                                         lowerBound(-std::numeric_limits<double>::infinity()),
                                                                                         upperBound(std::numeric_limits<double>::infinity()){};

    double calculate(const VectorDouble &point)
    {
        return (point.at(index));
    }

    void takeOwnership(OptimizationProblemPtr owner)
    {
        ownerProblem = owner;
    }
};

typedef std::shared_ptr<Variable> VariablePtr;

std::ostream &operator<<(std::ostream &stream, VariablePtr var)
{
    stream << "[" << var->index << "]:\t";

    switch (var->type)
    {
    case E_VariableType::Real:
        stream << var->lowerBound << " <= " << var->name << " <= " << var->upperBound;
        break;

    case E_VariableType::Binary:
        stream << var->name << " in {0,1}";
        break;

    case E_VariableType::Integer:
        if (var->lowerBound == 0.0 && var->upperBound == 1.0)
            stream << var->name << " in {0,1}";
        else
            stream << var->name << " in {" << var->lowerBound << ",...," << var->upperBound << "}";
        break;

    case E_VariableType::Semicontinuous:
        stream << var->name << " in {0} or " << var->lowerBound << " <= " << var->name << " <= " << var->upperBound;
        break;

    default:
        stream << var->lowerBound << " <= " << var->name << " <= " << var->upperBound;
        break;
    }

    return stream;
};

typedef std::vector<VariablePtr> Variables;

/*
class NonlinearVariable
{
  public:
    FactorableFunctionPtr factorableFunctionVariable;
    Variable originalVariable;

    NonlinearVariable()
    {
        Variable::lowerBound = -std::numeric_limits<double>::infinity();
        Variable::upperBound = std::numeric_limits<double>::infinity();
    };

    NonlinearVariable(std::string variableName, int variableIndex, E_VariableType variableType, double LB, double UB)
    {
        Variable::name = variableName;
        Variable::index = variableIndex;
        Variable::type = variableType;
        Variable::lowerBound = LB;
        Variable::upperBound = UB;
    };

    NonlinearVariable(std::string variableName, int variableIndex, E_VariableType variableType)
    {
        Variable::name = variableName;
        Variable::index = variableIndex;
        Variable::type = variableType;
        Variable::lowerBound = -std::numeric_limits<double>::infinity();
        Variable::upperBound = std::numeric_limits<double>::infinity();
    };

    void updateFactorableFunctionVariable(FactorableFunctionGraphPtr graph)
    {
        factorableFunctionVariable = std::make_shared<FactorableFunction>(graph.get());
    };
};

typedef std::shared_ptr<NonlinearVariable> NonlinearVariablePtr;
typedef std::vector<NonlinearVariablePtr> NonlinearVariables;
*/
/*
enum class E_AuxilliaryVariableType
{
    ConvexityReformulation,
    ConvexityPreservingReformulation,
    EpigraphReformulation
};

class AuxilliaryVariable : Variable
{
  public:
    E_AuxilliaryVariableType type;
};

typedef std::shared_ptr<AuxilliaryVariable> AuxilliaryVariablePtr;
typedef std::vector<AuxilliaryVariablePtr> AuxilliaryVariables;
*/

// End variable definitions

// Begin function definitions

enum class E_Curvature
{
    None,
    Convex,
    Concave,
    Nonconvex,
    Indeterminate
};

// End function definitions

// Begin exception definitions

class VariableNotFoundException : public std::exception
{
  private:
    std::string errorMessage;

  public:
    VariableNotFoundException(std::string message) : errorMessage(message)
    {
    }

    const char *what() const throw()
    {
        std::stringstream message;
        message << "Could not find variable ";
        message << errorMessage;

        return (message.str().c_str());
    }
};

// End exception definitions

} // namespace SHOT
