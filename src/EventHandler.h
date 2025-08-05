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
#include "Output.h"

#include <any>
#include <functional>
#include <map>
#include <vector>
#include <utility>
#include <optional>
#include <type_traits>
#include <stdexcept>

namespace SHOT
{

/**
 * @brief EventHandler that supports both notification callbacks and data providers
 *
 * This class provides a single registration interface that automatically detects whether
 * a callback is a notification callback (returns void) or a data provider (returns a value).
 *
 * Usage examples:
 *
 * // Data provider for user termination check - returns bool
 * eventHandler.registerCallback(E_EventType::UserTerminationCheck, []() {
 *     return shouldTerminate(); // Returns bool -> data provider for termination check
 * });
 *
 * // Data provider for dual bound - returns double
 * eventHandler.registerCallback(E_EventType::ExternalDualBound, []() {
 *     return computeDualBound(); // Returns double -> data provider
 * });
 *
 * // Data provider for primal solution - returns std::vector<VectorDouble>
 * eventHandler.registerCallback(E_EventType::ExternalPrimalSolution, []() {
 *     return getSolution(); // Returns std::vector<VectorDouble> -> data provider
 * });
 *
 * // Notification with parameters - returns void
 * eventHandler.registerCallback(E_EventType::NewPrimalSolution, [](std::any solution) {
 *     processSolution(solution); // Returns void -> notification callback
 * });
 */
class EventHandler
{
public:
    inline EventHandler(EnvironmentPtr envPtr) : env(envPtr) {};

    /**
     * @brief Unified callback registration method
     *
     * This method automatically detects the callback type based on its signature:
     * - If callback returns void: Registered as notification callback
     * - If callback returns a value: Registered as data provider
     * - If callback takes no arguments: Compatible with both types
     * - If callback takes std::any argument: Notification callback with data
     *
     * @tparam Callback The callback function type (lambda, function pointer, etc.)
     * @param event The event type to register for
     * @param callback The callback function to register
     */
    template <typename Callback> void registerCallback(const E_EventType& event, Callback&& callback)
    {

        using CallbackType = std::decay_t<Callback>;

        // Check if callback can be called with no arguments
        if constexpr(std::is_invocable_v<CallbackType>)
        {
            using ReturnType = std::invoke_result_t<CallbackType>;

            if constexpr(std::is_void_v<ReturnType>)
            {
                // Notification callback - returns void
                notificationCallbacks[event].push_back([callback](std::any) { callback(); });

                env->output->outputCritical("Registering callback for event: " + std::to_string(static_cast<int>(event))
                    + " (no args, no return)");
            }
            else
            {
                // Data provider - returns a value
                dataProviders[event] = [callback]() -> std::any { return std::any(callback()); };

                env->output->outputCritical("Registering callback for event: " + std::to_string(static_cast<int>(event))
                    + " (no args, returns value)");
            }
        }
        // Check if callback can be called with std::any argument
        else if constexpr(std::is_invocable_v<CallbackType, std::any>)
        {
            using ReturnType = std::invoke_result_t<CallbackType, std::any>;

            if constexpr(std::is_void_v<ReturnType>)
            {
                // Notification callback with arguments
                notificationCallbacks[event].push_back([callback](std::any args) { callback(args); });

                env->output->outputCritical("Registering callback for event: " + std::to_string(static_cast<int>(event))
                    + " (args, no return)");
            }
            else
            {
                // Data provider with arguments
                parameterizedDataProviders[event]
                    = [callback](std::any args) -> std::any { return std::any(callback(args)); };

                env->output->outputCritical("Registering callback for event: " + std::to_string(static_cast<int>(event))
                    + " (args, returns value)");
            }
        }
        else
        {
            static_assert(std::is_invocable_v<CallbackType> || std::is_invocable_v<CallbackType, std::any>,
                "Callback must be invocable with either no arguments or std::any argument");
        }
    }

    /**
     * @brief Notify all callbacks registered for a specific event type
     *
     * @param event The event type to notify
     * @param args Arguments to pass to the callbacks (wrapped in std::any)
     */
    void notify(const E_EventType& event, std::any args) const
    {
        env->output->outputTrace(
            "Notifying callbacks for event: " + std::to_string(static_cast<int>(event)) + " (args)");
        auto it = notificationCallbacks.find(event);
        if(it != notificationCallbacks.end())
        {
            for(const auto& callback : it->second)
            {
                callback(args);
            }
        }
    }

    /**
     * @brief Notify callbacks with no arguments
     *
     * @param event The event type to notify
     */
    void notify(const E_EventType& event) const
    {
        env->output->outputTrace(
            "Notifying callbacks for event: " + std::to_string(static_cast<int>(event)) + " (no args)");
        notify(event, std::any());
    }

    /**
     * @brief Request data from a registered data provider (no arguments)
     *
     * @tparam ReturnType The expected return type
     * @param event The event type to request data for
     * @return std::optional<ReturnType> The data if available, std::nullopt otherwise
     */
    template <typename ReturnType> std::optional<ReturnType> requestData(const E_EventType& event) const
    {
        auto it = dataProviders.find(event);
        if(it == dataProviders.end())
            return std::nullopt;

        try
        {
            std::any result = it->second();
            return std::any_cast<ReturnType>(result);
        }
        catch(const std::bad_any_cast&)
        {
            return std::nullopt;
        }
    }

    /**
     * @brief Request data from a registered data provider (with arguments)
     *
     * @tparam ReturnType The expected return type
     * @tparam ArgType The argument type to pass to the provider
     * @param event The event type to request data for
     * @param arg The argument to pass to the data provider
     * @return std::optional<ReturnType> The data if available, std::nullopt otherwise
     */
    template <typename ReturnType, typename ArgType>
    std::optional<ReturnType> requestData(const E_EventType& event, const ArgType& arg) const
    {
        auto it = parameterizedDataProviders.find(event);
        if(it == parameterizedDataProviders.end())
            return std::nullopt;

        try
        {
            std::any result = it->second(std::any(arg));
            return std::any_cast<ReturnType>(result);
        }
        catch(const std::bad_any_cast&)
        {
            return std::nullopt;
        }
    }

    /**
     * @brief Check if a data provider is registered for an event
     *
     * @param event The event type to check
     * @return true if a data provider is registered, false otherwise
     */
    bool hasDataProvider(const E_EventType& event) const
    {
        return dataProviders.find(event) != dataProviders.end()
            || parameterizedDataProviders.find(event) != parameterizedDataProviders.end();
    }

private:
    /// Map of event types to their registered notification callbacks
    std::map<E_EventType, std::vector<std::function<void(std::any)>>> notificationCallbacks;

    /// Map of event types to data provider callbacks (no parameters)
    std::map<E_EventType, std::function<std::any()>> dataProviders;

    /// Map of event types to parameterized data provider callbacks
    std::map<E_EventType, std::function<std::any(std::any)>> parameterizedDataProviders;

    /// Pointer to the environment
    EnvironmentPtr env;
};

} // namespace SHOT