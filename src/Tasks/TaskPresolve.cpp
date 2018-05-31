/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskPresolve.h"

TaskPresolve::TaskPresolve(IMIPSolver *MIPSolver)
{
    ProcessInfo::getInstance().startTimer("DualStrategy");

    isPresolved = false;
    this->MIPSolver = MIPSolver;

    ProcessInfo::getInstance().stopTimer("DualStrategy");
}

TaskPresolve::~TaskPresolve()
{
}

void TaskPresolve::run()
{
    ProcessInfo::getInstance().startTimer("DualStrategy");
    auto currIter = ProcessInfo::getInstance().getCurrentIteration();

    auto strategy = static_cast<ES_MIPPresolveStrategy>(Settings::getInstance().getIntSetting("MIP.Presolve.Frequency", "Dual"));

    if (!currIter->isMIP())
    {
        ProcessInfo::getInstance().stopTimer("DualStrategy");
        return;
    }

    if (strategy == ES_MIPPresolveStrategy::Never)
    {
        ProcessInfo::getInstance().stopTimer("DualStrategy");
        return;
    }
    else if (strategy == ES_MIPPresolveStrategy::Once && isPresolved == true)
    {
        ProcessInfo::getInstance().stopTimer("DualStrategy");
        return;
    }

    // Sets the iteration time limit
    auto timeLim = Settings::getInstance().getDoubleSetting("TimeLimit", "Termination") - ProcessInfo::getInstance().getElapsedTime("Total");
    MIPSolver->setTimeLimit(timeLim);

    if (ProcessInfo::getInstance().primalSolutions.size() > 0)
    {
        MIPSolver->setCutOff(ProcessInfo::getInstance().getPrimalBound());
    }

    if (MIPSolver->getDiscreteVariableStatus() && ProcessInfo::getInstance().primalSolutions.size() > 0)
    {
        MIPSolver->addMIPStart(ProcessInfo::getInstance().primalSolution);
    }

    if (Settings::getInstance().getBoolSetting("FixedInteger.UsePresolveBounds", "Primal") || Settings::getInstance().getBoolSetting("MIP.Presolve.UpdateObtainedBounds", "Dual"))
    {
        MIPSolver->presolveAndUpdateBounds();
        isPresolved = true;
    }

    ProcessInfo::getInstance().stopTimer("DualStrategy");
}

std::string TaskPresolve::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
