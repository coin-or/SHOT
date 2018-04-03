/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskInitializeOriginalProblem.h"

TaskInitializeOriginalProblem::TaskInitializeOriginalProblem(OSInstance *originalInstance)
{
    ProcessInfo::getInstance().startTimer("Reformulation");

    // This is needed to fix various problems later on.
    // TODO: figure out why...
    originalInstance->getJacobianSparsityPattern();

    instance = originalInstance;

    bool useQuadraticObjective = (static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QuadraticProblemStrategy::QuadraticObjective;

    bool useQuadraticConstraint = (static_cast<ES_QuadraticProblemStrategy>(Settings::getInstance().getIntSetting("QuadraticStrategy", "Dual"))) == ES_QuadraticProblemStrategy::QuadraticallyConstrained;

    bool isObjNonlinear = UtilityFunctions::isObjectiveGenerallyNonlinear(instance);
    bool isObjQuadratic = UtilityFunctions::isObjectiveQuadratic(instance);
    bool isQuadraticUsed = (useQuadraticObjective || (useQuadraticConstraint));

    if (isObjNonlinear || (isObjQuadratic && !isQuadraticUsed))
    {
        Output::getInstance().outputInfo("Nonlinear objective function detected.");
        ProcessInfo::getInstance().originalProblem = new OptProblemOriginalNonlinearObjective();
    }
    else if (isObjQuadratic && isQuadraticUsed)
    {
        Output::getInstance().outputAlways("Quadratic objective function detected.");
        ProcessInfo::getInstance().originalProblem = new OptProblemOriginalQuadraticObjective();
    }
    else //Linear objective function
    {
        Output::getInstance().outputInfo("Linear objective function detected.");
        ProcessInfo::getInstance().originalProblem = new OptProblemOriginalLinearObjective();
    }

    ProcessInfo::getInstance().originalProblem->setProblem(instance);
    auto debugPath = Settings::getInstance().getStringSetting("Debug.Path", "Output");

    if (Settings::getInstance().getBoolSetting("Debug.Enable", "Output"))
    {
        ProcessInfo::getInstance().originalProblem->saveProblemModelToFile(
            Settings::getInstance().getStringSetting("Debug.Path", "Output") + "/originalproblem.txt");
    }

    int numConstr = ProcessInfo::getInstance().originalProblem->getNumberOfConstraints();

    int numVar = ProcessInfo::getInstance().originalProblem->getNumberOfVariables();

    if (ProcessInfo::getInstance().originalProblem->isObjectiveFunctionNonlinear())
    {
        numVar = numVar - 1;       // Removes the extra objective variable
        numConstr = numConstr - 1; // Removes the extra objective constraint
    }

    ProcessInfo::getInstance().initializeResults(1, numVar, numConstr);

    ProcessInfo::getInstance().stopTimer("Reformulation");
}

TaskInitializeOriginalProblem::~TaskInitializeOriginalProblem()
{
    //delete problem;
    // delete instance;
}

void TaskInitializeOriginalProblem::run()
{
}

std::string TaskInitializeOriginalProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
