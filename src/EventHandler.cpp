/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "EventHandler.h"

namespace SHOT
{

EventHandler::EventHandler(EnvironmentPtr envPtr) : env(envPtr) { }

void EventHandler::notify(const E_EventType& event, std::any args) const
{
    env->output->outputTrace("Notifying callbacks for event: " + std::to_string(static_cast<int>(event)) + " (args)");
    auto it = notificationCallbacks.find(event);
    if(it != notificationCallbacks.end())
    {
        for(const auto& callback : it->second)
        {
            callback(args);
        }
    }
}

void EventHandler::notify(const E_EventType& event) const
{
    env->output->outputTrace(
        "Notifying callbacks for event: " + std::to_string(static_cast<int>(event)) + " (no args)");
    notify(event, std::any());
}

bool EventHandler::hasDataProvider(const E_EventType& event) const
{
    return dataProviders.find(event) != dataProviders.end()
        || parameterizedDataProviders.find(event) != parameterizedDataProviders.end();
}

bool EventHandler::hasNotificationCallbacks(const E_EventType& event) const
{
    auto it = notificationCallbacks.find(event);
    return it != notificationCallbacks.end() && !it->second.empty();
}

bool EventHandler::hasAnyCallbacks(const E_EventType& event) const
{
    return hasDataProvider(event) || hasNotificationCallbacks(event);
}

} // namespace SHOT
