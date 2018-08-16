/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include <ostream>
#include "OSInstance.h"
#include "boost/format.hpp"
#include <math.h>
#include <stdio.h>
#include <cmath>
#include <boost/math/special_functions/fpclassify.hpp> // isnan
#include "Structs.h"
#include <chrono>
#include <ctime>
#include <iostream>
#include <fstream>
#include <cerrno>

// Fix for missing NAN i Visual Studio
#ifdef WIN32
#ifndef NAN
static const unsigned long __nan[2] =
    {0xffffffff, 0x7fffffff};
#define NAN (*(const float *)__nan)
#endif
#endif

// Fix for Visual Studio c++ compiler
namespace UtilityFunctions
{
int round(double d);

bool isnan(double val);

void saveVariablePointVectorToFile(DoubleVector point, std::vector<std::string> variables,
                                   std::string fileName);

void savePrimalSolutionToFile(PrimalSolution solution, std::vector<std::string> variables, std::string fileName);

bool isObjectiveGenerallyNonlinear(OSInstance *instance);
bool isObjectiveQuadratic(OSInstance *instance);
bool areAllConstraintsLinear(OSInstance *instance);
bool areAllConstraintsQuadratic(OSInstance *instance);
bool areAllVariablesReal(OSInstance *instance);

void displayVector(DoubleVector point);
void displayVector(DoubleVector point1, DoubleVector point2);
void displayVector(std::vector<int> point);
void displayVector(std::vector<std::string> point);
void displayVector(std::vector<int> point1, std::vector<int> point2);
void displayVector(std::vector<int> point1, DoubleVector point2);

void displayVector(std::vector<DoubleVector> points);
void displayVector(std::vector<std::vector<int>> points);
void displayVector(std::vector<std::vector<std::string>> points);

void displayDifferencesInVector(DoubleVector point1, DoubleVector point2, double tol);

double L2Norm(DoubleVector ptA, DoubleVector ptB);
DoubleVector L2Norms(std::vector<DoubleVector> ptsA, DoubleVector ptB);
DoubleVector calculateCenterPoint(std::vector<DoubleVector> pts);

int numDifferentRoundedSelectedElements(DoubleVector firstPt, DoubleVector secondPt,
                                        std::vector<int> indexes);
bool isDifferentRoundedSelectedElements(DoubleVector firstPt, DoubleVector secondPt,
                                        std::vector<int> indexes);

bool isDifferentSelectedElements(DoubleVector firstPt, DoubleVector secondPt,
                                 std::vector<int> indexes);

std::string toStringFormat(double value, std::string format, bool useInfinitySymbol);
std::string toStringFormat(double value, std::string format);
std::string toString(double value);

double getJulianFractionalDate();

bool writeStringToFile(std::string fileName, std::string str);

std::string getFileAsString(std::string fileName);
} // namespace UtilityFunctions
