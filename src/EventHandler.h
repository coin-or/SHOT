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

#include <any>
#include <functional>
#include <map>
#include <vector>
#include <utility>

namespace SHOT
{

class EventHandler
{
public:
    inline EventHandler(EnvironmentPtr envPtr) : env(envPtr) {};

    // Register a callback for a specific event type
    template <typename Callback> void registerCallback(const E_EventType& event, Callback&& callback)
    {
        registeredCallbacks[event].push_back([callback](std::any args) { callback(args); });
    }

    // Notify all callbacks registered for a specific event type
    void notify(const E_EventType& event, std::any args) const
    {
        if(registeredCallbacks.empty())
            return;

        auto it = registeredCallbacks.find(event);
        if(it == registeredCallbacks.end())
            return;

        for(const auto& callback : it->second)
        {
            callback(args);
        }
    }

private:
    // Map of event types to their registered callbacks
    std::map<E_EventType, std::vector<std::function<void(std::any)>>> registeredCallbacks;

    EnvironmentPtr env;
};
} // namespace SHOT