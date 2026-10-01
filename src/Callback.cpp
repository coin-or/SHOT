/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Callback.h"

#include "DualSolver.h"
#include "Iteration.h"
#include "Output.h"
#include "Results.h"
#include "TaskHandler.h"
#include "Timing.h"

#include "MIPSolver/IMIPSolver.h"
#include "Model/ObjectiveFunction.h"
#include "Model/Problem.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace SHOT
{

namespace
{
    // Set while the callbacks are called on this thread, so that a callback that causes callbacks to be called, or
    // registers or removes one, is detected instead of deadlocking on the mutex of the handler
    thread_local bool isInvokingCallbacks = false;

    struct InvocationGuard
    {
        InvocationGuard() { isInvokingCallbacks = true; }
        ~InvocationGuard() { isInvokingCallbacks = false; }
    };

    void throwIfInvoking(const std::string& action)
    {
        if(isInvokingCallbacks)
            throw std::logic_error("A callback cannot " + action);
    }
} // namespace

std::string getCallbackLocationName(E_CallbackLocation location)
{
    switch(location)
    {
    case E_CallbackLocation::None:
        return ("None");
    case E_CallbackLocation::InteriorPointSearch:
        return ("InteriorPointSearch");
    case E_CallbackLocation::DualBoundUpdate:
        return ("DualBoundUpdate");
    case E_CallbackLocation::PrimalCandidateSearch:
        return ("PrimalCandidateSearch");
    case E_CallbackLocation::PrimalCandidateCheck:
        return ("PrimalCandidateCheck");
    case E_CallbackLocation::NewPrimalSolution:
        return ("NewPrimalSolution");
    case E_CallbackLocation::TerminationCheck:
        return ("TerminationCheck");
    case E_CallbackLocation::HyperplaneSelection:
        return ("HyperplaneSelection");
    }

    return ("Unknown");
}

CallbackContext::CallbackContext(EnvironmentPtr envPtr, E_CallbackLocation location)
    : location(location)
    , env(envPtr)
    , originalProblem(envPtr->problem)
    , reformulatedProblem(envPtr->reformulatedProblem)
{
    const double infinity = std::numeric_limits<double>::infinity();

    if(originalProblem)
        minimization = originalProblem->objectiveFunction->properties.isMinimize;

    if(env->results->getNumberOfIterations() > 0)
        iterationNumber = env->results->getCurrentIteration()->iterationNumber;

    elapsedTime = env->timing->getElapsedTime("Total");

    // SHOT uses NaN for a dual bound that has not been set yet, and the largest double for a missing primal bound
    auto isUnavailable = [](double value) { return (std::isnan(value) || std::abs(value) >= SHOT_DBL_MAX); };

    double currentDualBound = env->results->getCurrentDualBound();
    dualBound = isUnavailable(currentDualBound) ? (minimization ? -infinity : infinity) : currentDualBound;

    double currentGlobalDualBound = env->results->getGlobalDualBound();
    globalDualBound
        = isUnavailable(currentGlobalDualBound) ? (minimization ? -infinity : infinity) : currentGlobalDualBound;

    primalSolutionAvailable = env->results->hasPrimalSolution();

    double currentPrimalBound = primalSolutionAvailable ? env->results->getPrimalBound() : SHOT_DBL_MAX;
    primalBound = isUnavailable(currentPrimalBound) ? (minimization ? infinity : -infinity) : currentPrimalBound;

    if(std::isinf(dualBound) || std::isinf(primalBound))
    {
        relativeGap = infinity;
        absoluteGap = infinity;
    }
    else
    {
        relativeGap = env->results->getRelativeCurrentObjectiveGap();
        absoluteGap = env->results->getAbsoluteCurrentObjectiveGap();
    }

    terminationPending = env->tasks->isTerminated();
    finalizing = env->tasks->isFinalizing();

    solutionStatistics = env->solutionStatistics;
}

void CallbackContext::checkValid() const
{
    if(!valid)
    {
        throw CallbackContextExpired("The context of a callback at location " + getCallbackLocationName(location)
            + " was used after the callback returned.");
    }
}

bool CallbackContext::isMinimization() const
{
    checkValid();
    return (minimization);
}

int CallbackContext::getIterationNumber() const
{
    checkValid();
    return (iterationNumber);
}

double CallbackContext::getElapsedTime() const
{
    checkValid();
    return (elapsedTime);
}

double CallbackContext::getDualBound() const
{
    checkValid();
    return (dualBound);
}

double CallbackContext::getGlobalDualBound() const
{
    checkValid();
    return (globalDualBound);
}

double CallbackContext::getPrimalBound() const
{
    checkValid();
    return (primalBound);
}

double CallbackContext::getRelativeGap() const
{
    checkValid();
    return (relativeGap);
}

double CallbackContext::getAbsoluteGap() const
{
    checkValid();
    return (absoluteGap);
}

SolutionStatistics CallbackContext::getSolutionStatistics() const
{
    checkValid();
    return (solutionStatistics);
}

ProblemPtr CallbackContext::getOriginalProblem() const
{
    checkValid();
    return (originalProblem);
}

ProblemPtr CallbackContext::getReformulatedProblem() const
{
    checkValid();
    return (reformulatedProblem);
}

bool CallbackContext::hasPrimalSolution() const
{
    checkValid();
    return (primalSolutionAvailable);
}

VectorDouble CallbackContext::getPrimalSolution() const
{
    checkValid();

    if(!primalSolutionAvailable)
        throw std::logic_error("There is no primal solution.");

    return (env->results->primalSolution);
}

void CallbackContext::terminate()
{
    checkValid();
    terminationRequested = true;
}

bool CallbackContext::isTerminationRequested() const
{
    checkValid();
    return (terminationRequested);
}

bool CallbackContext::isTerminationPending() const
{
    checkValid();
    return (terminationPending);
}

bool CallbackContext::isFinalizing() const
{
    checkValid();
    return (finalizing);
}

void CallbackContext::invalidate()
{
    valid = false;

    env.reset();
    originalProblem.reset();
    reformulatedProblem.reset();

    releaseData();
}

void CallbackContext::discardActions() { terminationRequested = false; }

void CallbackContext::validatePoint(
    const VectorDouble& point, bool allowOriginal, bool allowReformulated, const std::string& description) const
{
    int originalSize = originalProblem ? originalProblem->properties.numberOfVariables : -1;
    int reformulatedSize = reformulatedProblem ? reformulatedProblem->properties.numberOfVariables : -1;
    int size = point.size();

    if(!((allowOriginal && size == originalSize) || (allowReformulated && size == reformulatedSize)))
    {
        std::string expected;

        if(allowOriginal && allowReformulated && originalSize != reformulatedSize)
            expected = std::to_string(originalSize) + " (original problem) or " + std::to_string(reformulatedSize)
                + " (reformulated problem)";
        else
            expected = std::to_string(allowOriginal ? originalSize : reformulatedSize);

        throw std::invalid_argument(
            description + " has " + std::to_string(size) + " values, but " + expected + " are required.");
    }

    for(size_t i = 0; i < point.size(); i++)
    {
        if(!std::isfinite(point[i]))
            throw std::invalid_argument(description + " has a value that is not finite at index " + std::to_string(i)
                + ": " + std::to_string(point[i]) + ".");
    }
}

PrimalCandidateCheckContext::PrimalCandidateCheckContext(
    EnvironmentPtr envPtr, VectorDouble candidatePoint, double candidateObjective, E_PrimalSolutionSource source)
    : CallbackContext(envPtr, Location)
    , point(std::move(candidatePoint))
    , objectiveValue(candidateObjective)
    , sourceType(source)
{
}

const VectorDouble& PrimalCandidateCheckContext::getPoint() const
{
    checkValid();
    return (point);
}

double PrimalCandidateCheckContext::getObjectiveValue() const
{
    checkValid();
    return (objectiveValue);
}

E_PrimalSolutionSource PrimalCandidateCheckContext::getSource() const
{
    checkValid();
    return (sourceType);
}

void PrimalCandidateCheckContext::rejectCandidate()
{
    checkValid();
    rejected = true;
}

bool PrimalCandidateCheckContext::isCandidateRejected() const
{
    checkValid();
    return (rejected);
}

void PrimalCandidateCheckContext::discardActions()
{
    CallbackContext::discardActions();
    rejected = false;
}

void PrimalCandidateCheckContext::releaseData() { VectorDouble().swap(point); }

NewPrimalSolutionContext::NewPrimalSolutionContext(EnvironmentPtr envPtr, VectorDouble solutionPoint,
    double solutionObjective, E_PrimalSolutionSource source, bool isIncumbentSolution)
    : CallbackContext(envPtr, Location)
    , point(std::move(solutionPoint))
    , objectiveValue(solutionObjective)
    , sourceType(source)
    , incumbent(isIncumbentSolution)
{
}

const VectorDouble& NewPrimalSolutionContext::getPoint() const
{
    checkValid();
    return (point);
}

double NewPrimalSolutionContext::getObjectiveValue() const
{
    checkValid();
    return (objectiveValue);
}

E_PrimalSolutionSource NewPrimalSolutionContext::getSource() const
{
    checkValid();
    return (sourceType);
}

bool NewPrimalSolutionContext::isIncumbent() const
{
    checkValid();
    return (incumbent);
}

void NewPrimalSolutionContext::releaseData() { VectorDouble().swap(point); }

DualBoundUpdateContext::DualBoundUpdateContext(EnvironmentPtr envPtr) : CallbackContext(envPtr, Location) { }

void DualBoundUpdateContext::setDualBound(double value)
{
    checkValid();

    if(!std::isfinite(value))
        throw std::invalid_argument("The dual bound must be finite, but it is " + std::to_string(value) + ".");

    if(!proposedDualBound.has_value())
        proposedDualBound = value;
    else if(isMinimization())
        proposedDualBound = std::max(*proposedDualBound, value);
    else
        proposedDualBound = std::min(*proposedDualBound, value);
}

std::optional<double> DualBoundUpdateContext::getProposedDualBound() const
{
    checkValid();
    return (proposedDualBound);
}

void DualBoundUpdateContext::discardActions()
{
    CallbackContext::discardActions();
    proposedDualBound.reset();
}

PrimalCandidateSearchContext::PrimalCandidateSearchContext(EnvironmentPtr envPtr)
    : CallbackContext(envPtr, Location) { }

void PrimalCandidateSearchContext::addPrimalSolution(const VectorDouble& point)
{
    checkValid();
    validatePoint(point, true, true, "The primal solution");
    addedPrimalSolutions.push_back(point);
}

const std::vector<VectorDouble>& PrimalCandidateSearchContext::getAddedPrimalSolutions() const
{
    checkValid();
    return (addedPrimalSolutions);
}

void PrimalCandidateSearchContext::discardActions()
{
    CallbackContext::discardActions();
    addedPrimalSolutions.clear();
}

void PrimalCandidateSearchContext::releaseData() { std::vector<VectorDouble>().swap(addedPrimalSolutions); }

HyperplaneSelectionContext::HyperplaneSelectionContext(EnvironmentPtr envPtr, std::vector<SolutionPoint> points)
    : CallbackContext(envPtr, Location), solutionPoints(std::move(points)), dualSolver(envPtr->dualSolver)
{
}

const std::vector<SolutionPoint>& HyperplaneSelectionContext::getSolutionPoints() const
{
    checkValid();
    return (solutionPoints);
}

void HyperplaneSelectionContext::addHyperplane(const ExternalHyperplane& hyperplane)
{
    checkValid();

    if(hyperplane.variableIndexes.size() != hyperplane.variableCoefficients.size())
    {
        throw std::invalid_argument("The hyperplane has " + std::to_string(hyperplane.variableIndexes.size())
            + " variable indexes but " + std::to_string(hyperplane.variableCoefficients.size()) + " coefficients.");
    }

    if(hyperplane.variableIndexes.empty())
        throw std::invalid_argument("The hyperplane has no variables.");

    // The indexes are those of the variables in the dual problem: the variables of the reformulated problem, and the
    // auxiliary objective variable if the dual problem has one
    int numberOfVariables = getReformulatedProblem()->properties.numberOfVariables;
    int auxiliaryObjectiveVariableIndex = -1;

    if(dualSolver && dualSolver->MIPSolver && dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
        auxiliaryObjectiveVariableIndex = dualSolver->MIPSolver->getDualAuxiliaryObjectiveVariableIndex();

    for(size_t i = 0; i < hyperplane.variableIndexes.size(); i++)
    {
        int index = hyperplane.variableIndexes[i];

        if(index < 0 || (index >= numberOfVariables && index != auxiliaryObjectiveVariableIndex))
        {
            throw std::invalid_argument("The hyperplane has the variable index " + std::to_string(index)
                + ", but the reformulated problem has " + std::to_string(numberOfVariables) + " variables.");
        }

        if(!std::isfinite(hyperplane.variableCoefficients[i]))
        {
            throw std::invalid_argument("The hyperplane has a coefficient that is not finite for the variable index "
                + std::to_string(index) + ".");
        }
    }

    if(!std::isfinite(hyperplane.rhsValue))
        throw std::invalid_argument("The right-hand side of the hyperplane is not finite.");

    addedHyperplanes.push_back(hyperplane);
}

const std::vector<ExternalHyperplane>& HyperplaneSelectionContext::getAddedHyperplanes() const
{
    checkValid();
    return (addedHyperplanes);
}

void HyperplaneSelectionContext::discardActions()
{
    CallbackContext::discardActions();
    addedHyperplanes.clear();
}

void HyperplaneSelectionContext::releaseData()
{
    std::vector<SolutionPoint>().swap(solutionPoints);
    std::vector<ExternalHyperplane>().swap(addedHyperplanes);
    dualSolver.reset();
}

InteriorPointSearchContext::InteriorPointSearchContext(EnvironmentPtr envPtr, std::vector<VectorDouble> points)
    : CallbackContext(envPtr, Location), interiorPoints(std::move(points))
{
}

const std::vector<VectorDouble>& InteriorPointSearchContext::getInteriorPoints() const
{
    checkValid();
    return (interiorPoints);
}

void InteriorPointSearchContext::setInteriorPoints(const std::vector<VectorDouble>& points)
{
    checkValid();

    if(points.empty())
    {
        throw std::invalid_argument(
            "At least one interior point must be given. To keep the current points, do not set any.");
    }

    for(size_t i = 0; i < points.size(); i++)
        validatePoint(points[i], true, true, "The interior point " + std::to_string(i));

    replacementInteriorPoints = points;
}

const std::optional<std::vector<VectorDouble>>& InteriorPointSearchContext::getReplacementInteriorPoints() const
{
    checkValid();
    return (replacementInteriorPoints);
}

void InteriorPointSearchContext::discardActions()
{
    CallbackContext::discardActions();
    replacementInteriorPoints.reset();
}

void InteriorPointSearchContext::releaseData()
{
    std::vector<VectorDouble>().swap(interiorPoints);
    replacementInteriorPoints.reset();
}

TerminationCheckContext::TerminationCheckContext(EnvironmentPtr envPtr) : CallbackContext(envPtr, Location) { }

CallbackHandler::CallbackHandler(EnvironmentPtr envPtr) : env(envPtr) { }

int CallbackHandler::add(E_CallbackLocation locations, CallbackFunction callback)
{
    throwIfInvoking("register a callback");

    if(!callback)
        throw std::invalid_argument("The callback is empty.");

    auto mask = static_cast<std::uint64_t>(locations);

    if(mask == 0)
        throw std::invalid_argument("A callback must be registered for at least one location.");

    if((mask & ~static_cast<std::uint64_t>(AllCallbackLocations)) != 0)
        throw std::invalid_argument("The mask " + std::to_string(mask) + " contains an unknown callback location.");

    std::lock_guard<std::mutex> lock(invokeMutex);

    int handle = nextHandle++;
    callbacks.push_back({ handle, locations, std::move(callback) });
    updateActiveLocations();

    return (handle);
}

bool CallbackHandler::remove(int handle)
{
    throwIfInvoking("remove a callback");

    std::lock_guard<std::mutex> lock(invokeMutex);

    auto callback = std::find_if(
        callbacks.begin(), callbacks.end(), [handle](const RegisteredCallback& C) { return (C.handle == handle); });

    if(callback == callbacks.end())
        return (false);

    callbacks.erase(callback);
    updateActiveLocations();

    return (true);
}

void CallbackHandler::updateActiveLocations()
{
    std::uint64_t mask = 0;

    if(!failed)
    {
        for(auto& C : callbacks)
            mask |= static_cast<std::uint64_t>(C.locations);
    }

    activeLocations.store(mask);
}

void CallbackHandler::invoke(CallbackContext& context)
{
    throwIfInvoking("cause other callbacks to be called");

    std::lock_guard<std::mutex> lock(invokeMutex);

    // A callback has failed at another location, possibly in another thread, since the caller checked isActive()
    if(failed)
    {
        context.discardActions();
        return;
    }

    {
        InvocationGuard guard;

        for(auto& C : callbacks)
        {
            if(!hasCallbackLocation(C.locations, context.getLocation()))
                continue;

            try
            {
                C.function(context);
            }
            catch(...)
            {
                failure = std::current_exception();
                failed = true;
                activeLocations.store(0);
                context.discardActions();

                try
                {
                    std::rethrow_exception(failure);
                }
                catch(const std::exception& e)
                {
                    failureMessage = e.what();
                }
                catch(...)
                {
                    failureMessage = "unknown exception";
                }

                env->output->outputError(" A callback at location " + getCallbackLocationName(context.getLocation())
                        + " failed, so SHOT will terminate:",
                    failureMessage);

                env->tasks->terminate();
                return;
            }
        }
    }

    if(context.isTerminationRequested())
    {
        env->output->outputInfo("        Termination requested by a callback at location "
            + getCallbackLocationName(context.getLocation()) + ".");
        env->tasks->terminate();
    }
}

void CallbackHandler::rethrowFailure()
{
    std::exception_ptr exception;

    {
        std::lock_guard<std::mutex> lock(invokeMutex);
        std::swap(exception, failure);
    }

    if(exception)
        std::rethrow_exception(exception);
}

} // namespace SHOT
