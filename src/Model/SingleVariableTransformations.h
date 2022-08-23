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
#include "../Model/Variables.h"
#include "../Model/AuxiliaryVariables.h"

#include <iterator>
#include <set>

namespace SHOT
{

struct BreakpointValue
{
    BreakpointValue(double point, double value, double iterationAdded, E_BreakpointSource source)
    {
        this->point = point;
        this->value = value;
        this->iterationAdded = iterationAdded;
        this->source = source;
    }

    double point;
    double value;
    int iterationAdded;
    E_BreakpointSource source;

    bool operator<(const BreakpointValue& bp) const { return (point < bp.point); }
};

class SingleVariableTransformation
{
public:
    virtual ~SingleVariableTransformation() = default;

    virtual double calculateInverse(double point) = 0;
    virtual double calculate(double point) = 0;

    inline bool containsBreakpoint(double point, double tolerance = 0.0)
    {
        for(auto const& BP : breakpoints)
        {
            if(abs(BP.point - point) <= tolerance)
                return true;
        }

        return (false);
    }

    inline bool addBreakpoint(double point, int iteration, E_BreakpointSource source)
    {
        if(containsBreakpoint(point, 1e-8))
            return false;

        breakpoints.emplace(point, calculateInverse(point), iteration, source);

        return (true);
    };

    inline bool addMidIntervalBreakpoint(double point, int iteration, E_BreakpointSource source)
    {
        auto iter = breakpoints.begin();

        while(iter != breakpoints.end())
        {
            auto next = std::next(iter);

            if(point > (*iter).point && point <= (*next).point)
            {
                double newPoint = ((*iter).point + (*next).point) / 2.0;

                if(containsBreakpoint(newPoint, 1e-6))
                    return (false);

                breakpoints.emplace(newPoint, calculateInverse(newPoint), iteration, source);
                return (true);
            }

            iter++;
        }

        return (false);
    };

    virtual std::ostream& print(std::ostream&) const = 0;

    inline void takeOwnership(ProblemPtr owner) { ownerProblem = owner; }

    std::set<BreakpointValue> breakpoints;
    VariablePtr originalVariable;
    AuxiliaryVariablePtr transformationVariable;
    E_SingleVariableTransformationType type;

    std::weak_ptr<Problem> ownerProblem;
};

class SingleVariableExponentialTransformation : public SingleVariableTransformation
{

public:
    SingleVariableExponentialTransformation() { type = E_SingleVariableTransformationType::Exponential; };

    SingleVariableExponentialTransformation(VectorDouble initialBreakpoints)
    {
        for(auto& PT : initialBreakpoints)
        {
            addBreakpoint(PT, 0, E_BreakpointSource::Initial);
        }

        type = E_SingleVariableTransformationType::Exponential;
    };

    ~SingleVariableExponentialTransformation() = default;

    std::ostream& print(std::ostream& stream) const override;

    double calculateInverse(double point) override { return (log(point)); };
    double calculate(double point) override { return (exp(point)); };
};

class SingleVariablePowerTransformation : public SingleVariableTransformation
{

public:
    SingleVariablePowerTransformation() { type = E_SingleVariableTransformationType::Power; };

    SingleVariablePowerTransformation(double transformationPower, VectorDouble initialBreakpoints)
    {
        power = transformationPower;

        for(auto& PT : initialBreakpoints)
        {
            addBreakpoint(PT, 0, E_BreakpointSource::Initial);
        }

        type = E_SingleVariableTransformationType::Power;
    };

    ~SingleVariablePowerTransformation() = default;

    std::ostream& print(std::ostream& stream) const override;

    double calculateInverse(double point) override { return (pow(point, 1 / power)); };
    double calculate(double point) override { return (pow(point, power)); };

    double power = NAN;
};

using SingleVariableTransformationPtr = std::shared_ptr<SingleVariableTransformation>;
using SingleVariableExponentialTransformationPtr = std::shared_ptr<SingleVariableExponentialTransformation>;
using SingleVariablePowerTransformationPtr = std::shared_ptr<SingleVariablePowerTransformation>;

std::ostream& operator<<(std::ostream& stream, SingleVariableTransformationPtr transformation);
std::ostream& operator<<(std::ostream& stream, const SingleVariableTransformation& transformation);
std::ostream& operator<<(std::ostream& stream, SingleVariableExponentialTransformationPtr transformation);
std::ostream& operator<<(std::ostream& stream, SingleVariablePowerTransformationPtr transformation);

class SingleVariableTransformations : private std::vector<SingleVariableTransformationPtr>
{
protected:
    std::weak_ptr<Problem> ownerProblem;

public:
    using std::vector<SingleVariableTransformationPtr>::operator[];

    using std::vector<SingleVariableTransformationPtr>::at;
    using std::vector<SingleVariableTransformationPtr>::begin;
    using std::vector<SingleVariableTransformationPtr>::clear;
    using std::vector<SingleVariableTransformationPtr>::end;
    using std::vector<SingleVariableTransformationPtr>::erase;
    using std::vector<SingleVariableTransformationPtr>::push_back;
    using std::vector<SingleVariableTransformationPtr>::reserve;
    using std::vector<SingleVariableTransformationPtr>::resize;
    using std::vector<SingleVariableTransformationPtr>::size;

    SingleVariableTransformations() = default;
    SingleVariableTransformations(std::initializer_list<SingleVariableTransformationPtr> transformations)
    {
        for(auto& T : transformations)
            (*this).push_back(T);
    };

    inline void takeOwnership(ProblemPtr owner)
    {
        ownerProblem = owner;

        for(auto& T : *this)
        {
            T->takeOwnership(owner);
        }
    }
};
};