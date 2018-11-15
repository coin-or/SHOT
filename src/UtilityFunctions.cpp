/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "UtilityFunctions.h"
//#include "Model/Variables.h"

namespace SHOT::UtilityFunctions
{

int round(double d)
{
    return static_cast<int>(d + 0.5);
};

bool isnan(double val)
{
    /*#if defined(_WIN64)
	 // x64 version
	 return _isnanf(val) != 0;
	 #else
	 return _isnan(val) != 0;
	 #endif*/
    return boost::math::isnan(val);
};

void saveVariablePointVectorToFile(const VectorDouble &point, const VectorString &variables, const std::string &fileName)
{
    if (point.size() != variables.size())
    {
        std::cout << "Error when saving variable point to file. Sizes mismatch!" << point.size() << "!="
                  << variables.size() << '\n';
        return;
    }

    std::stringstream str;

    for (int i = 0; i < point.size(); i++)
    {
        str << variables.at(i);
        str << "\t";
        str << point.at(i);
        str << '\n';
    }

    writeStringToFile(fileName, str.str());
};

void saveVariablePointVectorToFile(const VectorDouble &point, const Variables &variables, const std::string &fileName)
{
    if (point.size() != variables.size())
    {
        std::cout << "Error when saving variable point to file. Sizes mismatch!" << point.size() << "!="
                  << variables.size() << '\n';
        return;
    }

    std::stringstream str;

    for (int i = 0; i < point.size(); i++)
    {
        str << variables.at(i);
        str << "\t";
        str << point.at(i);
        str << '\n';
    }

    writeStringToFile(fileName, str.str());
};

void savePrimalSolutionToFile(const PrimalSolution &solution, const VectorString &variables,
                              const std::string &fileName)
{
    std::stringstream str;

    str << "Source: " << solution.sourceDescription;
    str << '\n';

    str << "Iteration found: " << solution.iterFound;
    str << '\n';

    str << "Objective value: " << toStringFormat(solution.objValue, "%.8f", false);
    str << '\n';

    str << "Largest nonlinear error (in constraint " << solution.maxDevatingConstraintNonlinear.index << "): "
        << toStringFormat(solution.maxDevatingConstraintNonlinear.value, "%.8f", false);
    str << '\n';

    str << "Largest linear error (in constraint " << solution.maxDevatingConstraintLinear.index << "): "
        << toStringFormat(solution.maxDevatingConstraintLinear.value, "%.8f", false);
    str << '\n';

    str << "Projection to variable bounds performed: " << (solution.boundProjectionPerformed ? "true" : "false");
    str << '\n';

    str << "Integer rounding performed: " << (solution.integerRoundingPerformed ? "true" : "false");
    str << '\n';

    str << "Max integer rounding error: " << toStringFormat(solution.maxIntegerToleranceError, "%.8f", false);

    str << '\n';
    str << '\n';

    str << "Solution point: ";
    str << '\n';

    for (int i = 0; i < solution.point.size(); i++)
    {
        str << i;
        str << "\t";
        str << toStringFormat(solution.point.at(i), "%.8f", false);
        ;
        str << '\n';
    }

    writeStringToFile(fileName, str.str());
};

void displayVector(const VectorDouble &point)
{
    std::stringstream str;

    if (point.size() == 0)
        str << "vector is empty";

    for (int i = 0; i < point.size(); i++)
    {
        str << i;
        str << "\t";
        str << point.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
};

void displayVector(const VectorDouble &point1, const VectorDouble &point2)
{
    std::stringstream str;

    if (point1.size() != point2.size())
        return;

    for (int i = 0; i < point1.size(); i++)
    {
        str << i;
        str << "\t";
        str << point1.at(i);

        str << "\t";
        str << point2.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
};

void displayVector(const VectorDouble &point1, const VectorDouble &point2, const VectorDouble &point3)
{
    std::stringstream str;

    if (point1.size() != point2.size())
        return;

    for (int i = 0; i < point1.size(); i++)
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
};

void displayDifferencesInVector(const VectorDouble &point1, const VectorDouble &point2, double tol)
{
    std::stringstream str;

    if (point1.size() != point2.size())
        return;

    for (int i = 0; i < point1.size(); i++)
    {
        if (std::abs(point1.at(i) - point2.at(i)) > tol)
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
};

void displayVector(const VectorInteger &point)
{
    std::stringstream str;

    if (point.size() == 0)
        str << "vector is empty";

    for (int i = 0; i < point.size(); i++)
    {
        str << i;
        str << "\t";
        str << point.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
};

void displayVector(const VectorString &point)
{
    std::stringstream str;

    if (point.size() == 0)
        str << "vector is empty";

    for (int i = 0; i < point.size(); i++)
    {
        str << i;
        str << "\t";
        str << point.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
};

void displayVector(const std::vector<VectorDouble> &points)
{
    std::stringstream str;

    for (int i = 0; i < points.at(0).size(); i++)
    {
        str << i;

        for (int j = 0; j < points.size(); j++)
        {
            str << "\t";
            str << points.at(j).at(i);
        }

        str << '\n';
    }

    std::cout << str.str() << '\n';
};

void displayVector(const std::vector<VectorInteger> &points)
{
    std::stringstream str;

    for (int i = 0; i < points.at(0).size(); i++)
    {
        str << i;

        for (int j = 0; j < points.size(); j++)
        {
            str << "\t";
            str << points.at(j).at(i);
        }

        str << '\n';
    }

    std::cout << str.str() << '\n';
};

void displayVector(const std::vector<VectorString> &points)
{
    std::stringstream str;

    for (int i = 0; i < points.at(0).size(); i++)
    {
        str << i;

        for (int j = 0; j < points.size(); j++)
        {
            str << "\t";
            str << points.at(j).at(i);
        }

        str << '\n';
    }

    std::cout << str.str() << '\n';
};

bool isObjectiveGenerallyNonlinear(OSInstance *instance)
{
    for (int i = 0; i < instance->getNumberOfNonlinearExpressions(); i++)
    {
        int tmpIndex = instance->instanceData->nonlinearExpressions->nl[i]->idx;
        if (tmpIndex == -1)
            return (true);
    }
    return (false);
};

bool isObjectiveQuadratic(OSInstance *instance)
{
    for (int i = 0; i < instance->getNumberOfQuadraticTerms(); i++)
    {
        int tmpIndex = instance->instanceData->quadraticCoefficients->qTerm[i]->idx;

        if (tmpIndex == -1)
            return (true);
    }

    return (false);
};

bool areAllConstraintsLinear(OSInstance *instance)
{
    int numNonlinearTreeIndexes = instance->getNumberOfNonlinearExpressionTreeIndexes();
    int numQuadraticRowIndexes = instance->getNumberOfQuadraticRowIndexes();

    for (int i = 0; i < numQuadraticRowIndexes; i++)
    {
        if (instance->getQuadraticRowIndexes()[i] != -1)
            return (false);
    }

    if (numNonlinearTreeIndexes == 0)
        return (true);

    for (int i = 0; i < numNonlinearTreeIndexes; i++)
    {
        if (instance->getNonlinearExpressionTreeIndexes()[i] != -1)
            return (false);
    }

    return (true);
}

bool areAllConstraintsQuadratic(OSInstance *instance)
{
    int numNonlinearTreeIndexes = instance->getNumberOfNonlinearExpressionTreeIndexes();

    if (numNonlinearTreeIndexes != 0)
        return (false);

    int numQuadraticRowIndexes = instance->getNumberOfQuadraticRowIndexes();

    // return false if there are no quadratic rows
    if (numQuadraticRowIndexes == 0)
        return false;

    return (true);
};

bool areAllVariablesReal(OSInstance *instance)
{
    int numDiscreteVars = instance->getNumberOfBinaryVariables() + instance->getNumberOfIntegerVariables();

    if (numDiscreteVars > 0)
        return (false);

    return (true);
};

double L2Norm(const VectorDouble &ptA, const VectorDouble &ptB)
{
    double norm = 0.0;

    if (ptA.size() != ptB.size())
    {
        return (-1.0);
    }

    for (int i = 0; i < ptA.size(); i++)
    {
        norm = norm + pow(ptA.at(i) - ptB.at(i), 2.0);
    }

    norm = sqrt(norm);

    return (norm);
};

VectorDouble L2Norms(const std::vector<VectorDouble> &ptsA, const VectorDouble &ptB)
{
    VectorDouble norms(ptsA.size());

    for (int i = 0; i < ptsA.size(); i++)
    {
        norms.at(i) = L2Norm(ptsA.at(i), ptB);
    }

    return (norms);
};

VectorDouble calculateCenterPoint(const std::vector<VectorDouble> &pts)
{
    int ptSize = pts.at(0).size();
    int numPts = pts.size();

    VectorDouble newPt(ptSize, 0.0);

    for (int i = 0; i < ptSize; i++)
    {

        for (int j = 0; j < numPts; j++)
        {
            newPt.at(i) += pts.at(j).at(i);
        }

        newPt.at(i) = newPt.at(i) / numPts;
    }

    return (newPt);
};

int numDifferentRoundedSelectedElements(const VectorDouble &firstPt, const VectorDouble &secondPt,
                                        const VectorInteger &indexes)
{
    int numDiff = 0;

    for (int i = 0; i < indexes.size(); i++)
    {
        if (round(firstPt.at(indexes.at(i))) != round(secondPt.at(indexes.at(i))))
        {
            numDiff++;
        }
    }
    return (numDiff);
};

bool isDifferentRoundedSelectedElements(const VectorDouble &firstPt, const VectorDouble &secondPt,
                                        const VectorInteger &indexes)
{
    for (int i = 0; i < indexes.size(); i++)
    {
        if (round(firstPt.at(indexes.at(i))) != round(secondPt.at(indexes.at(i))))
            return (true);
    }

    return (false);
};

bool isDifferentSelectedElements(const VectorDouble &firstPt, const VectorDouble &secondPt,
                                 const VectorInteger &indexes)
{
    for (int i = 0; i < indexes.size(); i++)
    {
        if (firstPt.at(indexes.at(i)) != secondPt.at(indexes.at(i)))
            return (true);
    }

    return (false);
};

std::string toStringFormat(const double value, const std::string &format)
{
    return (toStringFormat(value, format, true));
}

std::string toStringFormat(double value, const std::string &format, const bool useInfinitySymbol)
{
    std::string str;

    if (useInfinitySymbol && value < -1.e20)
    {
        str = "-inf.";
    }
    else if (useInfinitySymbol && value > 1.e20)
    {
        str = "inf.";
    }
    else
    {
        str = ((boost::format(format) % value).str());
    }

    return (str);
};

std::string toString(double value)
{
    return (toStringFormat(value, "%.3f", true));
}

void displayVector(const VectorInteger &point1, const VectorInteger &point2)
{
    std::stringstream str;

    if (point1.size() != point2.size())
        return;

    for (int i = 0; i < point1.size(); i++)
    {
        str << i;
        str << "\t";
        str << point1.at(i);

        str << "\t";
        str << point2.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
};

void displayVector(const VectorInteger &point1, const VectorDouble &point2)
{
    std::stringstream str;

    if (point1.size() != point2.size())
        return;

    for (int i = 0; i < point1.size(); i++)
    {
        str << i;
        str << "\t";
        str << point1.at(i);

        str << "\t";
        str << point2.at(i);
        str << '\n';
    }

    std::cout << str.str() << '\n';
};

double getJulianFractionalDate()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    struct tm *parts = std::localtime(&now_c);

    auto Y = 1900 + parts->tm_year;
    auto M = 1 + parts->tm_mon;
    auto D = parts->tm_mday;
    auto hours = parts->tm_hour;
    auto mins = parts->tm_min;
    auto secs = parts->tm_sec;

    auto secstoday = (3600 * hours + 60 * mins + secs);

    // To conform with GAMS Julian format
    auto julianDate = -2415020 + (1461 * (Y + 4800 + (M - 14) / 12)) / 4 + (367 * (M - 2 - 12 * ((M - 14) / 12))) / 12 - (3 * ((Y + 4900 + (M - 14) / 12) / 100)) / 4 + D - 32075 + secstoday / 86400.0;

    return julianDate;
};

bool writeStringToFile(const std::string &fileName, const std::string &str)
{
    std::ofstream f(fileName);

    if (f)
    {
        f << str;
    }
    else
    {
        return false;
    }

    f.close();
    return true;
};

std::string getFileAsString(const std::string &fileName)
{
    std::ifstream in(fileName, std::ios::in | std::ios::binary);
    if (in)
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
};
}; // namespace SHOT