
/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "OSiLReader.h"
#include "OSiLWriter.h"
#include "OSInstance.h"
#include "OSIpoptSolver.h"
#include "OSnl2OS.h"
#include "OSOption.h"
#include "OSoLReader.h"
#include "OSoLWriter.h"
#include "OSResult.h"
#include "OSrLWriter.h"

#include "CoinHelperFunctions.hpp" // for CoinCopyOfArrayOrZero, maybe should eliminate this
#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"
#include "CoinFinite.hpp"