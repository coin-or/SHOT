/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "Structs.h"

namespace SHOT
{
class Variable;
using VariablePtr = std::shared_ptr<Variable>;
using SparseVariableVector = std::map<VariablePtr, double>;
using SparseVariableMatrix = std::map<std::pair<VariablePtr, VariablePtr>, double>;
}

namespace SHOT::Utilities
{

using PairString = std::pair<std::string, std::string>;
using PairDouble = std::pair<double, double>;
using VectorString = std::vector<std::string>;
using VectorDouble = std::vector<double>;
using VectorInteger = std::vector<int>;

void saveVariablePointVectorToFile(
    const VectorDouble& point, const VectorString& variables, const std::string& fileName);

void displayVector(const VectorDouble& point);
void displayVector(const VectorDouble& point1, const VectorDouble& point2);
void displayVector(const VectorDouble& point1, const VectorDouble& point2, const VectorDouble& point3);
void displayVector(const VectorInteger& point);
void displayVector(const VectorString& point);
void displayVector(const VectorInteger& point1, const VectorInteger& point2);
void displayVector(const VectorInteger& point1, const VectorDouble& point2);

void displayVector(const std::vector<VectorDouble>& points);
void displayVector(const std::vector<VectorInteger>& points);
void displayVector(const std::vector<VectorString>& points);

void displayDifferencesInVector(const VectorDouble& point1, const VectorDouble& point2, double tol);

double L2Norm(const VectorDouble& ptA, const VectorDouble& ptB);
VectorDouble L2Norms(const std::vector<VectorDouble>& ptsA, const VectorDouble& ptB);
VectorDouble calculateCenterPoint(const std::vector<VectorDouble>& pts);

int numDifferentRoundedSelectedElements(
    const VectorDouble& firstPt, const VectorDouble& secondPt, const VectorInteger& indexes);
bool isDifferentRoundedSelectedElements(
    const VectorDouble& firstPt, const VectorDouble& secondPt, const VectorInteger& indexes);

bool isDifferentSelectedElements(
    const VectorDouble& firstPt, const VectorDouble& secondPt, const VectorInteger& indexes);

bool isDifferent(const VectorDouble& firstPt, const VectorDouble& secondPt);

std::string toStringFormat(
    const double value, const std::string& format, const bool useInfinitySymbol, const std::string infLabel = "inf.");
std::string toStringFormat(const double value, const std::string& format);
std::string toString(const double value);

double getJulianFractionalDate();

bool DllExport writeStringToFile(const std::string& fileName, const std::string& str);

std::string getFileAsString(const std::string& fileName);

VectorString getLinesInFile(const std::string& fileName);

/*
 * Generic implementation to erase elements by value
 * It iterates over all the elements and for every element it matches
 * the value with the passed value, if it matches then it will delete
 * that entry and move to next.
 * From: https://thispointer.com/
 */
template <typename K, typename V> inline void erase_if(std::map<K, V>& mapOfElement, V value)
{
    auto it = mapOfElement.begin();
    // Iterate through the map
    while(it != mapOfElement.end())
    {
        // Check if value of this entry matches with given value
        if(it->second == value)
        {
            // Erase the current element, erase() will return the
            // next iterator. So, don't need to increment
            it = mapOfElement.erase(it);
        }
        else
        {
            // Go to next entry in map
            it++;
        }
    }
}

template <typename T> double calculateHash(std::vector<T> const& point);

bool isAlmostEqual(double x, double y, const double epsilon);

bool isAlmostZero(double x, const double epsilon = std::numeric_limits<double>::epsilon());

bool isInteger(double value);
std::string trim(std::string& str);

SparseVariableVector combineSparseVariableVectors(
    const SparseVariableVector& first, const SparseVariableVector& second);

SparseVariableVector combineSparseVariableVectors(
    const SparseVariableVector& first, const SparseVariableVector& second, const SparseVariableVector& third);

SparseVariableMatrix combineSparseVariableMatrices(
    const SparseVariableMatrix& first, const SparseVariableMatrix& second);

SparseVariableMatrix combineSparseVariableMatrices(
    const SparseVariableMatrix& first, const SparseVariableMatrix& second, const SparseVariableMatrix& third);

E_Convexity combineConvexity(const E_Convexity first, const E_Convexity second);

E_Monotonicity combineMonotonicity(const E_Monotonicity first, const E_Monotonicity second);

std::vector<std::string> splitStringByCharacter(const std::string& source, char character);

// Creates a unique directory in the specified folder (or system temporary folder if folder is an empty string).
// Returns an empty string if the directory could not be created.
std::string createTemporaryDirectory(std::string filePrefix, std::string folder = "");
} // namespace SHOT::Utilities