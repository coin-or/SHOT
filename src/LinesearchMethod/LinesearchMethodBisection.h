/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "ILinesearchMethod.h"
#include "SHOTSettings.h"
#include "../OptProblems/OptProblemOriginal.h"

class LinesearchMethodBisection : public ILinesearchMethod
{
  public:
    LinesearchMethodBisection(EnvironmentPtr envPtr);

    virtual ~LinesearchMethodBisection();

    virtual std::pair<std::vector<double>, std::vector<double>> findZero(std::vector<double> ptA,
                                                                         std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol);

    virtual std::pair<std::vector<double>, std::vector<double>> findZero(std::vector<double> ptA,
                                                                         std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol, std::vector<int> constrIdxs);
};
