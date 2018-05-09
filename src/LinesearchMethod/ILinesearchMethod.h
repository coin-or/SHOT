/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Environment.h"

class ILinesearchMethod
{
  public:
    ILinesearchMethod(){};
    ILinesearchMethod(EnvironmentPtr envPtr){};
    virtual ~ILinesearchMethod(){};

    virtual std::pair<std::vector<double>, std::vector<double>> findZero(std::vector<double> ptA, std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol) = 0;

    virtual std::pair<std::vector<double>, std::vector<double>> findZero(std::vector<double> ptA, std::vector<double> ptB, int Nmax, double lambdaTol, double constrTol, std::vector<int> constrIdxs) = 0;

  protected:
    EnvironmentPtr env;
};
