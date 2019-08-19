/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

#include "Utilities.h"

#include <boost/functional/hash/hash.hpp>

#include "spdlog/fmt/fmt.h"

namespace SHOT::Utilities
{

void saveVariablePointVectorToFile(
    const VectorDouble& point, const VectorString& variables, const std::string& fileName)
{
    if(point.size() > variables.size())
    {
        std::cout << "Error when saving variable point to file. Sizes mismatch!" << point.size()
                  << "!=" << variables.size() << '\n';
    }

    std::stringstream str;

    int number = std::min(point.size(), variables.size());

    for(int i = 0; i < number; i++)
    {
        str << variables.at(i);
        str << "\t";
        str << std::setprecision(std::numeric_limits<double>::digits10);
        str << point.at(i);
        str << '\n';
    }

    writeStringToFile(fileName, str.str());
}

void displayVector(const VectorDouble& point)
{
    std::stringstream str;

    if(point.size() == 0)
        str << "vector is empty";

    str << std::setprecision(std::numeric_limits<double>::digits10);

    for(size_t i = 0; i < point.size(); i++)
    {
        str << i;
        str << "\t";

        str << point.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
}

void displayVector(const VectorDouble& point1, const VectorDouble& point2)
{
    std::stringstream str;

    str << std::setprecision(std::numeric_limits<double>::digits10);

    if(point1.size() != point2.size())
        return;

    for(size_t i = 0; i < point1.size(); i++)
    {
        str << i;
        str << "\t";
        str << point1.at(i);

        str << "\t";
        str << point2.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
}

void displayVector(const VectorDouble& point1, const VectorDouble& point2, const VectorDouble& point3)
{
    std::stringstream str;
    str << std::setprecision(std::numeric_limits<double>::digits10);

    if(point1.size() != point2.size())
        return;

    for(size_t i = 0; i < point1.size(); i++)
    {
        str << i;
        str << "\t";
        str << point1.at(i);

        str << "\t";
        str << point2.at(i);

        str << "\t";
        str << point3.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
}

void displayDifferencesInVector(const VectorDouble& point1, const VectorDouble& point2, double tol)
{
    std::stringstream str;
    str << std::setprecision(std::numeric_limits<double>::digits10);

    if(point1.size() != point2.size())
        return;

    for(size_t i = 0; i < point1.size(); i++)
    {
        if(std::abs(point1.at(i) - point2.at(i)) > tol)
        {
            str << i;
            str << "\t";
            str << point1.at(i);

            str << "\t";
            str << point2.at(i);
            str << '\n';
        }
    }

    std::cout << str.str() << '\n';
}

void displayVector(const VectorInteger& point)
{
    std::stringstream str;

    if(point.size() == 0)
        str << "vector is empty";

    for(size_t i = 0; i < point.size(); i++)
    {
        str << i;
        str << "\t";
        str << point.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
}

void displayVector(const VectorString& point)
{
    std::stringstream str;

    if(point.size() == 0)
        str << "vector is empty";

    for(size_t i = 0; i < point.size(); i++)
    {
        str << i;
        str << "\t";
        str << point.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
}

void displayVector(const std::vector<VectorDouble>& points)
{
    std::stringstream str;
    str << std::setprecision(std::numeric_limits<double>::digits10);

    for(size_t i = 0; i < points.at(0).size(); i++)
    {
        str << i;

        for(const auto& P : points)
        {
            str << "\t";
            str << P.at(i);
        }

        str << '\n';
    }

    std::cout << str.str() << '\n';
}

void displayVector(const std::vector<VectorInteger>& points)
{
    std::stringstream str;

    for(size_t i = 0; i < points.at(0).size(); i++)
    {
        str << i;

        for(const auto& P : points)
        {
            str << "\t";
            str << P.at(i);
        }

        str << '\n';
    }

    std::cout << str.str() << '\n';
}

void displayVector(const std::vector<VectorString>& points)
{
    std::stringstream str;

    for(size_t i = 0; i < points.at(0).size(); i++)
    {
        str << i;

        for(const auto& P : points)
        {
            str << "\t";
            str << P.at(i);
        }

        str << '\n';
    }

    std::cout << str.str() << '\n';
}

double L2Norm(const VectorDouble& ptA, const VectorDouble& ptB)
{
    double norm = 0.0;

    if(ptA.size() != ptB.size())
    {
        return (-1.0);
    }

    for(size_t i = 0; i < ptA.size(); i++)
    {
        norm = norm + pow(ptA.at(i) - ptB.at(i), 2.0);
    }

    norm = sqrt(norm);

    return (norm);
}

VectorDouble L2Norms(const std::vector<VectorDouble>& ptsA, const VectorDouble& ptB)
{
    VectorDouble norms(ptsA.size());

    for(size_t i = 0; i < ptsA.size(); i++)
    {
        norms.at(i) = L2Norm(ptsA.at(i), ptB);
    }

    return (norms);
}

VectorDouble calculateCenterPoint(const std::vector<VectorDouble>& pts)
{
    int ptSize = pts.at(0).size();
    int numPts = pts.size();

    VectorDouble newPt(ptSize, 0.0);

    for(int i = 0; i < ptSize; i++)
    {

        for(int j = 0; j < numPts; j++)
        {
            newPt.at(i) += pts.at(j).at(i);
        }

        newPt.at(i) = newPt.at(i) / numPts;
    }

    return (newPt);
}

int numDifferentRoundedSelectedElements(
    const VectorDouble& firstPt, const VectorDouble& secondPt, const VectorInteger& indexes)
{
    int numDiff = 0;

    for(int I : indexes)
    {
        if(round(firstPt.at(I)) != round(secondPt.at(I)))
        {
            numDiff++;
        }
    }
    return (numDiff);
}

bool isDifferentRoundedSelectedElements(
    const VectorDouble& firstPt, const VectorDouble& secondPt, const VectorInteger& indexes)
{
    for(int I : indexes)
    {
        if(round(firstPt.at(I)) != round(secondPt.at(I)))
            return (true);
    }

    return (false);
}

bool isDifferentSelectedElements(
    const VectorDouble& firstPt, const VectorDouble& secondPt, const VectorInteger& indexes)
{
    for(int I : indexes)
    {
        if(firstPt.at(I) != secondPt.at(I))
            return (true);
    }

    return (false);
}

bool isDifferent(const VectorDouble& firstPt, const VectorDouble& secondPt)
{
    assert(firstPt.size() == secondPt.size());

    for(size_t i = 0; i < firstPt.size(); i++)
    {
        if(firstPt.at(i) != secondPt.at(i))
            return (true);
    }

    return (false);
}

std::string toStringFormat(const double value, const std::string& format)
{
    return (toStringFormat(value, format, true));
}

std::string toStringFormat(double value, const std::string& format, const bool useInfinitySymbol)
{
    std::string str;

    if(useInfinitySymbol && value < -1.e20)
    {
        str = "-inf.";
    }
    else if(useInfinitySymbol && value > 1.e20)
    {
        str = "inf.";
    }
    else
    {
        str = fmt::format(format, value);
    }

    return (str);
}

std::string toString(double value) { return (toStringFormat(value, "{:.3f}", true)); }

void displayVector(const VectorInteger& point1, const VectorInteger& point2)
{
    std::stringstream str;

    if(point1.size() != point2.size())
        return;

    for(size_t i = 0; i < point1.size(); i++)
    {
        str << i;
        str << "\t";
        str << point1.at(i);

        str << "\t";
        str << point2.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
}

void displayVector(const VectorInteger& point1, const VectorDouble& point2)
{
    std::stringstream str;

    if(point1.size() != point2.size())
        return;

    for(size_t i = 0; i < point1.size(); i++)
    {
        str << i;
        str << "\t";
        str << point1.at(i);

        str << "\t";
        str << point2.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
}

double getJulianFractionalDate()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    struct tm* parts = std::localtime(&now_c);

    auto Y = 1900 + parts->tm_year;
    auto M = 1 + parts->tm_mon;
    auto D = parts->tm_mday;
    auto hours = parts->tm_hour;
    auto mins = parts->tm_min;
    auto secs = parts->tm_sec;

    auto secstoday = (3600 * hours + 60 * mins + secs);

    // To conform with GAMS Julian format
    auto julianDate = -2415020 + (1461 * (Y + 4800 + (M - 14) / 12)) / 4 + (367 * (M - 2 - 12 * ((M - 14) / 12))) / 12
        - (3 * ((Y + 4900 + (M - 14) / 12) / 100)) / 4 + D - 32075 + secstoday / 86400.0;

    return julianDate;
}

bool writeStringToFile(const std::string& fileName, const std::string& str)
{
    std::ofstream f(fileName);

    if(f)
    {
        f << str;
    }
    else
    {
        return false;
    }

    f.close();
    return true;
}

std::string getFileAsString(const std::string& fileName)
{
    std::ifstream in(fileName, std::ios::in | std::ios::binary);
    if(in)
    {
        std::string contents;
        in.seekg(0, std::ios::end);
        contents.resize(in.tellg());
        in.seekg(0, std::ios::beg);
        in.read(&contents[0], contents.size());
        in.close();
        return (contents);
    }

    throw(errno);
}

std::size_t calculateHash(VectorDouble const& point)
{
    std::size_t seed = 0;

    for(auto& X : point)
        boost::hash_combine(seed, X);

    return (seed);
}

bool isAlmostEqual(double x, double y, const double epsilon) { return std::abs(x - y) <= epsilon * std::abs(x); }

std::string trim(const std::string& str)
{
    size_t first = str.find_first_not_of(' ');
    if(std::string::npos == first)
    {
        return str;
    }

    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}

bool isInteger(double value)
{
    double intpart;

    return (std::modf(value, &intpart) == 0.0);
}

SparseVariableVector combineSparseVariableVectors(const SparseVariableVector& first, const SparseVariableVector& second)
{
    SparseVariableVector result;

    for(auto& G : first)
    {
        auto element = result.emplace(G.first, G.second);

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += G.second;
        }
    }

    for(auto& G : second)
    {
        auto element = result.emplace(G.first, G.second);

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += G.second;
        }
    }

    return result;
}

SparseVariableVector combineSparseVariableVectors(
    const SparseVariableVector& first, const SparseVariableVector& second, const SparseVariableVector& third)
{
    SparseVariableVector result;

    for(auto& G : first)
    {
        auto element = result.emplace(G.first, G.second);

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += G.second;
        }
    }

    for(auto& G : second)
    {
        auto element = result.emplace(G.first, G.second);

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += G.second;
        }
    }

    for(auto& G : third)
    {
        auto element = result.emplace(G.first, G.second);

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += G.second;
        }
    }

    return result;
}

SparseVariableMatrix combineSparseVariableMatrices(
    const SparseVariableMatrix& first, const SparseVariableMatrix& second)
{
    SparseVariableMatrix result;

    for(auto& G : first)
    {
        auto element = result.emplace(G.first, G.second);

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += G.second;
        }
    }

    for(auto& G : second)
    {
        auto element = result.emplace(G.first, G.second);

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += G.second;
        }
    }

    return result;
}

SparseVariableMatrix combineSparseVariableMatrices(
    const SparseVariableMatrix& first, const SparseVariableMatrix& second, const SparseVariableMatrix& third)
{
    SparseVariableMatrix result;

    for(auto& G : first)
    {
        auto element = result.emplace(G.first, G.second);

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += G.second;
        }
    }

    for(auto& G : second)
    {
        auto element = result.emplace(G.first, G.second);

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += G.second;
        }
    }

    for(auto& G : third)
    {
        auto element = result.emplace(G.first, G.second);

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += G.second;
        }
    }

    return result;
}

E_Convexity combineConvexity(const E_Convexity first, const E_Convexity second)
{
    if(first == E_Convexity::Unknown || second == E_Convexity::Unknown)
        return E_Convexity::Unknown;

    if(first == E_Convexity::Nonconvex || second == E_Convexity::Nonconvex)
        return E_Convexity::Nonconvex;

    if(first == E_Convexity::Convex && second == E_Convexity::Concave)
        return E_Convexity::Unknown;

    if(first == E_Convexity::Concave && second == E_Convexity::Convex)
        return E_Convexity::Unknown;

    if(first == E_Convexity::Convex || second == E_Convexity::Convex)
        return E_Convexity::Convex;

    if(first == E_Convexity::Concave || second == E_Convexity::Concave)
        return E_Convexity::Concave;

    return E_Convexity::Linear;
}

} // namespace SHOT::Utilities