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

    template <typename Callback> void registerCallback(const E_EventType& event, Callback&& callback)
    {
        registeredCallbacks[event].push_back(std::forward<Callback>(callback));
    }

    template <typename Callback> void registerCallback(E_EventType&& event, Callback&& callback)
    {
        registeredCallbacks[std::move(event)].push_back(std::forward<Callback>(callback));
    }

    inline void notify(const E_EventType& event) const
    {
        if(registeredCallbacks.size() == 0)
            return;

        if(registeredCallbacks.find(event) == registeredCallbacks.end())
            return;

        for(const auto& C : registeredCallbacks.at(event))
            C();
    }

private:
    std::map<E_EventType, std::vector<std::function<void()>>> registeredCallbacks;

    EnvironmentPtr env;
};
} // namespace SHOT