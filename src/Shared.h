
/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <ctime>
#include <exception>
#include <float.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <list>
#include <math.h>
#include <memory>
#include <mutex>
#include <ostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <thread>
#include <typeinfo>
#include <utility>
#include <vector>

#include <algorithm>
#include <iostream>
#include <map>

#include <boost/algorithm/string.hpp>
#include "boost/filesystem.hpp"
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include "boost/math/tools/minima.hpp"
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <boost/math/special_functions/fpclassify.hpp> // isnan

#include "ffunc.hpp"

#include "OSErrorClass.h"
#include "OSiLReader.h"
#include "OSiLWriter.h"
#include "OSInstance.h"
#include "OSIpoptSolver.h"
#include "OSnl2OS.h"
#include "OSOption.h"
#include "OSoLReader.h"
#include "OSResult.h"
#include "OSrLWriter.h"

#include "CoinHelperFunctions.hpp" // for CoinCopyOfArrayOrZero, maybe should eliminate this
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"
#include "CoinFinite.hpp"

#include "Enums.h"
#include "Structs.h"
#include "Timer.h"

#include "Environment.h"
#include "Output.h"
#include "SHOTSettings.h"

#include "ModelShared.h"
#include "UtilityFunctions.h"

#include "ProcessInfo.h"
#include "Iteration.h"
#include "Report.h"

#include "Model/Variables.h"
#include "Model/Terms.h"
#include "Model/NonlinearExpressions.h"
#include "Model/ObjectiveFunction.h"
#include "Model/Constraints.h"
#include "Model/Problem.h"
#include "Model.h"