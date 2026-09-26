/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "InstanceTestCommon.h"

int InstanceTest(int argc, char* argv[])
{
    return runInstanceTestMain(argc, argv, InstanceTestScope::Core, "Instance");
}
