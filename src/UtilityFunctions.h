/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include <ostream>
#include "boost/format.hpp"
#include <math.h>
#include <stdio.h>
#include <cmath>
#include <boost/math/special_functions/fpclassify.hpp> // isnan
#include <chrono>
#include <ctime>
#include <iostream>
#include <fstream>
#include <cerrno>
#include "OSInstance.h"

#include "Structs.h"
#include "Model/Variables.h"

// Fix for missing NAN i Visual Studio
#ifdef WIN32
#ifndef NAN
static const unsigned long __nan[2] =
    {0xffffffff, 0x7fffffff};
#define NAN (*(const float *)__nan)
#endif
#endif

// Fix for Visual Studio c++ compiler
namespace SHOT::UtilityFunctions
{
int round(double d);

bool isnan(double val);

void saveVariablePointVectorToFile(const VectorDouble &point, const VectorString &variables, const std::string &fileName);
void saveVariablePointVectorToFile(const VectorDouble &point, const Variables &variables, const std::string &fileName);

void savePrimalSolutionToFile(const PrimalSolution &solution, const VectorString &variables, const std::string &fileName);

bool isObjectiveGenerallyNonlinear(OSInstance *instance);
bool isObjectiveQuadratic(OSInstance *instance);
bool areAllConstraintsLinear(OSInstance *instance);
bool areAllConstraintsQuadratic(OSInstance *instance);
bool areAllVariablesReal(OSInstance *instance);

void displayVector(const VectorDouble &point);
void displayVector(const VectorDouble &point1, const VectorDouble &point2);
void displayVector(const VectorDouble &point1, const VectorDouble &point2, const VectorDouble &point3);
void displayVector(const VectorInteger &point);
void displayVector(const VectorString &point);
void displayVector(const VectorInteger &point1, const VectorInteger &point2);
void displayVector(const VectorInteger &point1, const VectorDouble &point2);

void displayVector(const std::vector<VectorDouble> &points);
void displayVector(const std::vector<VectorInteger> &points);
void displayVector(const std::vector<VectorString> &points);

void displayDifferencesInVector(const VectorDouble &point1, const VectorDouble &point2, double tol);

double L2Norm(const VectorDouble &ptA, const VectorDouble &ptB);
VectorDouble L2Norms(const std::vector<VectorDouble> &ptsA, const VectorDouble &ptB);
VectorDouble calculateCenterPoint(const std::vector<VectorDouble> &pts);

int numDifferentRoundedSelectedElements(const VectorDouble &firstPt, const VectorDouble &secondPt,
                                        const VectorInteger &indexes);
bool isDifferentRoundedSelectedElements(const VectorDouble &firstPt, const VectorDouble &secondPt,
                                        const VectorInteger &indexes);

bool isDifferentSelectedElements(const VectorDouble &firstPt, const VectorDouble &secondPt,
                                 const VectorInteger &indexes);

std::string toStringFormat(const double value, const std::string &format, const bool useInfinitySymbol);
std::string toStringFormat(const double value, const std::string &format);
std::string toString(const double value);

double getJulianFractionalDate();

bool writeStringToFile(const std::string &fileName, const std::string &str);

std::string getFileAsString(const std::string &fileName);

/*
 * Generic implementation to erase elements by value
 * It iterates over all the elements and for every element it matches
 * the value with the passed value, if it matches then it will delete
 * that entry and move to next.
 * From: https://thispointer.com/
 */
template <typename K, typename V>
inline void erase_if(std::map<K, V> &mapOfElemen, V value)
{
    auto it = mapOfElemen.begin();
    // Iterate through the map
    while (it != mapOfElemen.end())
    {
        // Check if value of this entry matches with given value
        if (it->second == value)
        {
            // Erase the current element, erase() will return the
            // next iterator. So, don't need to increment
            it = mapOfElemen.erase(it);
        }
        else
        {
            // Go to next entry in map
            it++;
        }
    }
}
} // namespace SHOT::UtilityFunctions