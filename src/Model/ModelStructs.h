
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

namespace SHOT
{
typedef std::vector<PairIndexValue> SparseVector;
typedef std::vector<PairCoordinateValue> SparseMatrix;

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
    std::string name;
    int index;

    E_VariableType type;

    double upperBound;
    double lowerBound;

    double calculate(const VectorDouble &point)
    {
        return (point.at(index));
    }
};

typedef std::shared_ptr<Variable> VariablePtr;
typedef std::vector<VariablePtr> Variables;

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

// End variable definitions

// Begin function definitions

enum class E_Curvature
{
    Convex,
    Concave,
    Nonconvex,
    Indeterminate,
    None
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
