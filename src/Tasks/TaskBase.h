/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Enums.h"
#include <iostream>
#include <typeinfo>
#include "TaskException.h"

class TaskBase
{
  public:
    virtual bool isActive();
    virtual void activate();
    virtual void deactivate();

    virtual void initialize();

    virtual std::string getType();

    virtual void run();

  protected:
  private:
    bool m_isActive;
};
