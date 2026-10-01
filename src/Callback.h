/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "Environment.h"
#include "Enums.h"
#include "Structs.h"

#include <atomic>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace SHOT
{

/**
 * @brief The locations in the solution process where SHOT calls the registered callbacks
 *
 * The values are bit flags, so one callback can be registered for several locations by combining them with
 * operator|. The callback is given a context of the class for the location, e.g., a PrimalCandidateCheckContext for
 * PrimalCandidateCheck, and can use CallbackContext::as<T>() to get it. See docs/Callbacks.md for where each
 * location is reached in the solution strategies.
 */
enum class E_CallbackLocation : std::uint64_t
{
    None = 0,
    InteriorPointSearch = 1ull << 0, ///< After the interior point search for the ESH dual strategy
    DualBoundUpdate = 1ull << 1, ///< When SHOT can accept a dual bound from the user
    PrimalCandidateSearch = 1ull << 2, ///< When SHOT collects primal solution candidates
    PrimalCandidateCheck = 1ull << 3, ///< Before a primal solution candidate is checked
    NewPrimalSolution = 1ull << 4, ///< After a primal solution has been stored
    TerminationCheck = 1ull << 5, ///< When SHOT checks whether to terminate
    HyperplaneSelection = 1ull << 6 ///< When SHOT selects the hyperplanes to add to the dual problem
};

/// All the locations, for registering a callback that is called everywhere
constexpr E_CallbackLocation AllCallbackLocations = static_cast<E_CallbackLocation>((1ull << 7) - 1);

constexpr E_CallbackLocation operator|(E_CallbackLocation first, E_CallbackLocation second)
{
    return static_cast<E_CallbackLocation>(static_cast<std::uint64_t>(first) | static_cast<std::uint64_t>(second));
}

constexpr E_CallbackLocation operator&(E_CallbackLocation first, E_CallbackLocation second)
{
    return static_cast<E_CallbackLocation>(static_cast<std::uint64_t>(first) & static_cast<std::uint64_t>(second));
}

/// Whether the location is one of the locations in the mask
constexpr bool hasCallbackLocation(E_CallbackLocation mask, E_CallbackLocation location)
{
    return ((mask & location) != E_CallbackLocation::None);
}

/// The name of a single location, as used in messages and in the Python interface
DllExport std::string getCallbackLocationName(E_CallbackLocation location);

/// Thrown when a callback context is used after the callback it was given to has returned
class DllExport CallbackContextExpired : public std::logic_error
{
public:
    using std::logic_error::logic_error;
};

/**
 * @brief The information and actions available to a callback
 *
 * The context is only valid while the callback it is given to runs. It holds a snapshot of the state of the solver,
 * taken when SHOT reaches the location, so all callbacks called at the same location see the same values, and none
 * of them sees the actions of another. The actions are applied by SHOT after all the callbacks have returned.
 *
 * Values that are not available are reported as infinite: the dual bound before the first dual problem has been
 * solved, the primal bound before a primal solution has been found, and the gaps while either bound is missing.
 *
 * The problems returned by getOriginalProblem() and getReformulatedProblem() can be used to evaluate functions, but
 * must not be modified.
 *
 * SHOT creates the contexts with std::make_shared, so that a context kept by the Python interface after the callback
 * has returned is still an object, whose methods throw CallbackContextExpired.
 */
class DllExport CallbackContext : public std::enable_shared_from_this<CallbackContext>
{
public:
    virtual ~CallbackContext() = default;

    CallbackContext(const CallbackContext&) = delete;
    CallbackContext& operator=(const CallbackContext&) = delete;

    /// The location the callback is called at; can also be used after the context has expired
    E_CallbackLocation getLocation() const { return (location); }

    /// Whether the context can still be used, i.e., the callback it was given to has not returned
    bool isValid() const { return (valid); }

    bool isMinimization() const;

    /// The number of the current iteration, or 0 before the first iteration
    int getIterationNumber() const;

    /// The time since SHOT was started, in seconds
    double getElapsedTime() const;

    /// The current dual bound, which is the one the termination criteria use
    double getDualBound() const;

    /// The dual bound that is valid for the whole problem
    double getGlobalDualBound() const;

    double getPrimalBound() const;

    /// The relative gap between the current dual bound and the primal bound
    double getRelativeGap() const;

    /// The absolute gap between the current dual bound and the primal bound
    double getAbsoluteGap() const;

    /// A copy of the statistics of the solution process
    SolutionStatistics getSolutionStatistics() const;

    ProblemPtr getOriginalProblem() const;
    ProblemPtr getReformulatedProblem() const;

    bool hasPrimalSolution() const;

    /// The best primal solution, in the variables of the original problem; throws if there is none
    VectorDouble getPrimalSolution() const;

    /// Requests SHOT to terminate at its next termination check; available at every location
    void terminate();

    /// Whether terminate() has been called in this context, by this or an earlier callback at the same location
    bool isTerminationRequested() const;

    /// Whether termination was requested before SHOT reached the location, e.g., by a callback at another location,
    /// so that SHOT is stopping. Stays true while the solution is finalized
    bool isTerminationPending() const;

    /// Whether SHOT is finalizing the solution, for any termination reason
    bool isFinalizing() const;

    /// The context as the class of a location, or nullptr if the callback is called at another location
    template <typename T> T* as() { return (location == T::Location) ? static_cast<T*>(this) : nullptr; }
    template <typename T> const T* as() const
    {
        return (location == T::Location) ? static_cast<const T*>(this) : nullptr;
    }

    /// Called by SHOT when the callbacks have returned; the context cannot be used after this
    void invalidate();

    /// Called by SHOT when a callback has failed, so that no action of the callbacks is applied
    virtual void discardActions();

protected:
    CallbackContext(EnvironmentPtr envPtr, E_CallbackLocation location);

    /// Throws CallbackContextExpired if the callback has returned
    void checkValid() const;

    /// Releases the data held by a derived class when the context is invalidated
    virtual void releaseData() { }

    /// Throws std::invalid_argument if the point is neither in the variables of the original problem nor in those of
    /// the reformulated problem (as allowed), or if a value is not finite
    void validatePoint(
        const VectorDouble& point, bool allowOriginal, bool allowReformulated, const std::string& description) const;

private:
    E_CallbackLocation location;
    bool valid = true;
    bool terminationRequested = false;

    EnvironmentPtr env;
    ProblemPtr originalProblem;
    ProblemPtr reformulatedProblem;

    bool minimization = true;
    int iterationNumber = 0;
    double elapsedTime = 0.0;
    double dualBound;
    double globalDualBound;
    double primalBound;
    double relativeGap;
    double absoluteGap;
    bool primalSolutionAvailable = false;
    bool terminationPending = false;
    bool finalizing = false;
    SolutionStatistics solutionStatistics;
};

/// Called for each primal solution candidate before SHOT checks it; the candidate can be rejected
class DllExport PrimalCandidateCheckContext : public CallbackContext
{
public:
    static constexpr E_CallbackLocation Location = E_CallbackLocation::PrimalCandidateCheck;

    PrimalCandidateCheckContext(
        EnvironmentPtr envPtr, VectorDouble candidatePoint, double candidateObjective, E_PrimalSolutionSource source);

    /// The candidate, in the variables of the original problem
    const VectorDouble& getPoint() const;
    double getObjectiveValue() const;
    E_PrimalSolutionSource getSource() const;

    /// SHOT will not check the candidate, so it cannot become a primal solution
    void rejectCandidate();
    bool isCandidateRejected() const;

    void discardActions() override;

protected:
    void releaseData() override;

private:
    VectorDouble point;
    double objectiveValue;
    E_PrimalSolutionSource sourceType;
    bool rejected = false;
};

/// Called when a primal solution has been stored; no actions except terminate()
class DllExport NewPrimalSolutionContext : public CallbackContext
{
public:
    static constexpr E_CallbackLocation Location = E_CallbackLocation::NewPrimalSolution;

    NewPrimalSolutionContext(EnvironmentPtr envPtr, VectorDouble solutionPoint, double solutionObjective,
        E_PrimalSolutionSource source, bool isIncumbentSolution);

    /// The solution, in the variables of the original problem
    const VectorDouble& getPoint() const;
    double getObjectiveValue() const;
    E_PrimalSolutionSource getSource() const;

    /// Whether the solution is better than the best one SHOT had before it
    bool isIncumbent() const;

protected:
    void releaseData() override;

private:
    VectorDouble point;
    double objectiveValue;
    E_PrimalSolutionSource sourceType;
    bool incumbent;
};

/// Called when SHOT can accept a dual bound from the user
class DllExport DualBoundUpdateContext : public CallbackContext
{
public:
    static constexpr E_CallbackLocation Location = E_CallbackLocation::DualBoundUpdate;

    DualBoundUpdateContext(EnvironmentPtr envPtr);

    /// Proposes a dual bound; SHOT uses it if it is better than the current dual bound. If several are proposed, the
    /// tightest one is used
    void setDualBound(double value);

    std::optional<double> getProposedDualBound() const;

    void discardActions() override;

private:
    std::optional<double> proposedDualBound;
};

/// Called when SHOT collects primal solution candidates; candidates can be added
class DllExport PrimalCandidateSearchContext : public CallbackContext
{
public:
    static constexpr E_CallbackLocation Location = E_CallbackLocation::PrimalCandidateSearch;

    PrimalCandidateSearchContext(EnvironmentPtr envPtr);

    /// Adds a candidate, in the variables of either the original or the reformulated problem; can be called several
    /// times
    void addPrimalSolution(const VectorDouble& point);

    const std::vector<VectorDouble>& getAddedPrimalSolutions() const;

    void discardActions() override;

protected:
    void releaseData() override;

private:
    std::vector<VectorDouble> addedPrimalSolutions;
};

/// Called when SHOT selects the hyperplanes to add to the dual problem; hyperplanes can be added
class DllExport HyperplaneSelectionContext : public CallbackContext
{
public:
    static constexpr E_CallbackLocation Location = E_CallbackLocation::HyperplaneSelection;

    HyperplaneSelectionContext(EnvironmentPtr envPtr, std::vector<SolutionPoint> points);

    /// The solution points of the dual problem, in the variables of the reformulated problem
    const std::vector<SolutionPoint>& getSolutionPoints() const;

    /// Adds a hyperplane in the variables of the reformulated problem; can be called several times
    void addHyperplane(const ExternalHyperplane& hyperplane);

    const std::vector<ExternalHyperplane>& getAddedHyperplanes() const;

    void discardActions() override;

protected:
    void releaseData() override;

private:
    std::vector<SolutionPoint> solutionPoints;
    std::vector<ExternalHyperplane> addedHyperplanes;
    DualSolverPtr dualSolver;
};

/// Called after the interior point search for the ESH dual strategy; the interior points can be replaced
class DllExport InteriorPointSearchContext : public CallbackContext
{
public:
    static constexpr E_CallbackLocation Location = E_CallbackLocation::InteriorPointSearch;

    InteriorPointSearchContext(EnvironmentPtr envPtr, std::vector<VectorDouble> points);

    /// The interior points SHOT has found, in the variables of the reformulated problem; can be empty
    const std::vector<VectorDouble>& getInteriorPoints() const;

    /// Replaces the interior points. The points can be in the variables of either the original or the reformulated
    /// problem, and there must be at least one. If called several times, the last call is used
    void setInteriorPoints(const std::vector<VectorDouble>& points);

    /// The replacement points, or nullopt if the current points are kept
    const std::optional<std::vector<VectorDouble>>& getReplacementInteriorPoints() const;

    void discardActions() override;

protected:
    void releaseData() override;

private:
    std::vector<VectorDouble> interiorPoints;
    std::optional<std::vector<VectorDouble>> replacementInteriorPoints;
};

/// Called when SHOT checks whether to terminate; no actions except terminate()
class DllExport TerminationCheckContext : public CallbackContext
{
public:
    static constexpr E_CallbackLocation Location = E_CallbackLocation::TerminationCheck;

    TerminationCheckContext(EnvironmentPtr envPtr);
};

using CallbackFunction = std::function<void(CallbackContext&)>;

/**
 * @brief Calls the callbacks registered for a location
 *
 * The callbacks are called one at a time, also when SHOT reaches a location from several threads, and in the order
 * they were registered. If a callback throws, the remaining callbacks are not called, the actions of all callbacks
 * at that location are discarded, no callback is called again, and SHOT terminates. The exception is rethrown from
 * Solver::solveProblem().
 */
class DllExport CallbackHandler
{
public:
    CallbackHandler(EnvironmentPtr envPtr);

    /// Registers a callback for the locations in the mask; returns a handle for remove()
    int add(E_CallbackLocation locations, CallbackFunction callback);

    /// Removes a callback; returns false if there is no callback with the handle
    bool remove(int handle);

    /// Whether a callback is registered for the location, and callbacks have not been disabled by a failure
    bool isActive(E_CallbackLocation location) const
    {
        return ((activeLocations.load(std::memory_order_relaxed) & static_cast<std::uint64_t>(location)) != 0);
    }

    /// Calls the callbacks registered for the location of the context. A requested termination is passed on to the
    /// task handler, the other actions are applied by the caller
    void invoke(CallbackContext& context);

    bool hasFailed() const { return (failed.load()); }

    /// The message of the exception of a failed callback
    std::string getFailureMessage() const { return (failureMessage); }

    /// Rethrows the exception of a failed callback, if there is one, and forgets it
    void rethrowFailure();

private:
    struct RegisteredCallback
    {
        int handle;
        E_CallbackLocation locations;
        CallbackFunction function;
    };

    void updateActiveLocations();

    std::vector<RegisteredCallback> callbacks;
    std::atomic<std::uint64_t> activeLocations { 0 };
    int nextHandle = 1;

    std::mutex invokeMutex;
    std::atomic<bool> failed { false };
    std::exception_ptr failure;
    std::string failureMessage;

    EnvironmentPtr env;
};

} // namespace SHOT
