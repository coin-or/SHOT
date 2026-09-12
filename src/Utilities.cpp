/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <algorithm>

#include "Utilities.h"

#include <boost/functional/hash/hash.hpp>

#include "spdlog/fmt/fmt.h"

#ifdef HAS_STD_FILESYSTEM
#include <filesystem>
namespace fs = std;
#endif

#ifdef HAS_STD_EXPERIMENTAL_FILESYSTEM
#include <experimental/filesystem>
namespace fs = std::experimental;
#endif

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

std::string toStringFormat(double value, const std::string& format, const bool useInfinitySymbol, std::string infLabel)
{
    std::string str;

    if(useInfinitySymbol && value < -1.e20)
    {
        str = "-" + infLabel;
    }
    else if(useInfinitySymbol && value > 1.e20)
    {
        str = infLabel;
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
    std::ofstream f(fileName, std::ios::binary);

    if(f)
    {
        f << str;
    }
    else
    {
        f.close();
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

VectorString getLinesInFile(const std::string& fileName)
{
    VectorString lines;

    std::ifstream myfile(fileName);

    if(myfile.is_open())
    {
        std::string line;

        while(getline(myfile, line))
            lines.push_back(line);

        myfile.close();
    }

    return (lines);
}

double fixedPseudoRandomNumber(size_t index, size_t stream, double low, double high)
{
    // The bits of the index are mixed with SplitMix64, which gives well spread out values without keeping any
    // state, so the number for an index is the same however many numbers have been generated before it
    uint64_t value = static_cast<uint64_t>(index) * 2 + stream + 0x9E3779B97F4A7C15ULL;
    value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ULL;
    value = (value ^ (value >> 27)) * 0x94D049BB133111EBULL;
    value = value ^ (value >> 31);

    // The 53 significant bits are mapped onto [0, 1) here rather than by a distribution from the standard
    // library, since those are allowed to differ between implementations
    double unitValue = static_cast<double>(value >> 11) / static_cast<double>(1ULL << 53);

    return (low + (high - low) * unitValue);
}

template double calculateHash(VectorDouble const& point);
template double calculateHash(VectorInteger const& point);

template <typename T> double calculateHash(std::vector<T> const& point)
{
    // The coefficients are calculated where they are used instead of being kept in a vector that grows as longer
    // points are hashed. That vector was shared by every solver in the process, and the hashes are calculated in
    // the callbacks of the MIP solver, which run in parallel, so extending it could reallocate it while another
    // thread was reading it.
    double scalarProduct = 0.0;

    for(size_t i = 0; i < point.size(); i++)
        scalarProduct += fixedPseudoRandomNumber(i, 0, 1.0, 101.0) * point[i];

    return (scalarProduct);
}

bool isAlmostEqual(double x, double y, const double epsilon) { return std::abs(x - y) <= epsilon * std::abs(x); }

bool isAlmostZero(double x, const double epsilon) { return std::abs(x) < epsilon; }

std::string trim(std::string& str)
{
    str.erase(0, str.find_first_not_of(' '));
    str.erase(str.find_last_not_of(' ') + 1);

    return (str);
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
    if(first == E_Convexity::NotSet && second == E_Convexity::NotSet)
        return E_Convexity::NotSet;

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

E_Monotonicity combineMonotonicity(const E_Monotonicity first, const E_Monotonicity second)
{
    if(first == E_Monotonicity::NotSet && second == E_Monotonicity::NotSet)
        return E_Monotonicity::NotSet;

    if(first == E_Monotonicity::Unknown || second == E_Monotonicity::Unknown)
        return E_Monotonicity::Unknown;

    if(first == E_Monotonicity::Constant && second == E_Monotonicity::Constant)
        return E_Monotonicity::Constant;

    if((first == E_Monotonicity::Nondecreasing || first == E_Monotonicity::Constant)
        && (second == E_Monotonicity::Nondecreasing || second == E_Monotonicity::Constant))
        return E_Monotonicity::Nondecreasing;

    if((first == E_Monotonicity::Nonincreasing || first == E_Monotonicity::Constant)
        && (second == E_Monotonicity::Nonincreasing || second == E_Monotonicity::Constant))
        return E_Monotonicity::Nonincreasing;

    return E_Monotonicity::Unknown;
}

std::vector<std::string> splitStringByCharacter(const std::string& source, char character)
{
    auto result = std::vector<std::string> {};
    auto ss = std::stringstream { source };

    for(std::string line; std::getline(ss, line, character);)
        result.push_back(line);

    return result;
}

std::string createTemporaryDirectory(std::string filePrefix, std::string folder)
{
    auto tmpDirPath = fs::filesystem::temp_directory_path();
    int i = 0;

    int maxTries = 1000;
    std::random_device dev;
    std::mt19937 prng(dev());
    std::uniform_int_distribution<uint64_t> rand(0);
    fs::filesystem::path path;

    while(true)
    {
        std::stringstream ss;
        ss << filePrefix << std::hex << rand(prng);

        if(folder == "")
            path = tmpDirPath / ss.str();
        else
            path = fs::filesystem::path(folder) / ss.str();

        if(fs::filesystem::create_directory(path))
        // True if the directory was created.
        {
            break;
        }
        if(i == maxTries)
        {
            return ("");
        }

        i++;
    }

    return (path.string());
}

} // namespace SHOT::Utilities