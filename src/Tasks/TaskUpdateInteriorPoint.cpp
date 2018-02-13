#include "TaskUpdateInteriorPoint.h"

TaskUpdateInteriorPoint::TaskUpdateInteriorPoint()
{
}

TaskUpdateInteriorPoint::~TaskUpdateInteriorPoint()
{
    // TODO Auto-generated destructor stub
}

void TaskUpdateInteriorPoint::run()
{
    // If we do not yet have a valid primal solution we can't do anything
    if (ProcessInfo::getInstance().primalSolutions.size() == 0)
        return;

    auto maxDevPrimal = ProcessInfo::getInstance().primalSolutions.at(0).maxDevatingConstraintNonlinear;
    auto tmpPrimalPoint = ProcessInfo::getInstance().primalSolutions.at(0).point;

    // If we do not have an interior point, but uses the ESH dual strategy, update with primal solution
    if (ProcessInfo::getInstance().interiorPts.size() == 0 && maxDevPrimal.value < 0)
    {
        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());
        tmpIP->point = ProcessInfo::getInstance().primalSolutions.at(0).point;
        tmpIP->maxDevatingConstraint = ProcessInfo::getInstance().primalSolutions.at(0).maxDevatingConstraintNonlinear;

        ProcessInfo::getInstance().outputAlways("     Interior point replaced with primal solution point since no interior point was previously available.");

        ProcessInfo::getInstance().interiorPts.push_back(tmpIP);

        return;
    }
    else if(ProcessInfo::getInstance().interiorPts.size() == 0)
    {
        return;
    }

    // Add the new point if it is deeper within the feasible region
    if (maxDevPrimal.value < ProcessInfo::getInstance().interiorPts.at(0)->maxDevatingConstraint.value)
    {
        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());
        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = maxDevPrimal;

        ProcessInfo::getInstance().outputAlways("     Interior point replaced with primal solution point due to constraint deviation.");

        ProcessInfo::getInstance().interiorPts.back() = tmpIP;
    }
    else if (Settings::getInstance().getIntSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual") == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepBoth) && maxDevPrimal.value < 0)
    {
        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = maxDevPrimal;

        ProcessInfo::getInstance().outputAlways("     Primal solution point used as additional interior point.");

        if (ProcessInfo::getInstance().interiorPts.size() == ProcessInfo::getInstance().numOriginalInteriorPoints)
        {
            ProcessInfo::getInstance().interiorPts.push_back(tmpIP);
        }
        else
        {
            ProcessInfo::getInstance().interiorPts.back() = tmpIP;
        }
    }
    else if (Settings::getInstance().getIntSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual") == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepNew) && maxDevPrimal.value < 0)
    {
        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

        // Add the new point only
        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = maxDevPrimal;

        ProcessInfo::getInstance().outputAlways("     Interior point replaced with primal solution point.");

        ProcessInfo::getInstance().interiorPts.back() = tmpIP;
    }
    else if (Settings::getInstance().getIntSetting("ESH.InteriorPoint.UsePrimalSolution", "Dual") == static_cast<int>(ES_AddPrimalPointAsInteriorPoint::OnlyAverage) && maxDevPrimal.value < 0)
    {
        std::shared_ptr<InteriorPoint> tmpIP(new InteriorPoint());

        // Find a new point in the midpoint between the original and new
        for (int i = 0; i < tmpPrimalPoint.size(); i++)
        {
            tmpPrimalPoint.at(i) = (0.5 * tmpPrimalPoint.at(i) + 0.5 * ProcessInfo::getInstance().interiorPts.at(0)->point.at(i));
        }

        tmpIP->point = tmpPrimalPoint;
        tmpIP->maxDevatingConstraint = ProcessInfo::getInstance().originalProblem->getMostDeviatingConstraint(tmpPrimalPoint);

        ProcessInfo::getInstance().outputAlways("     Interior point replaced with primal solution point.");

        ProcessInfo::getInstance().interiorPts.back() = tmpIP;
    }
}

std::string TaskUpdateInteriorPoint::getType()
{
    std::string type = typeid(this).name();
    return (type);
}