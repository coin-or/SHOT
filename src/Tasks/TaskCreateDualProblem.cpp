/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCreateDualProblem.h"

#include "../DualSolver.h"
#include "../Output.h"
#include "../Settings.h"
#include "../Timing.h"

#include "../MIPSolver/IMIPSolver.h"

namespace SHOT
{

TaskCreateDualProblem::TaskCreateDualProblem(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("DualStrategy");

    env->output->outputDebug(" Creating dual problem");

    createProblem(env->dualSolver->MIPSolver, env->reformulatedProblem);

    env->dualSolver->MIPSolver->finalizeProblem();

    env->dualSolver->MIPSolver->initializeSolverSettings();

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        env->dualSolver->MIPSolver->writeProblemToFile(
            env->settings->getSetting<std::string>("Debug.Path", "Output") + "/dualiter0_problem.lp");
    }

    env->output->outputDebug(" Dual problem created");
    env->timing->stopTimer("DualStrategy");
}

TaskCreateDualProblem::~TaskCreateDualProblem() = default;

void TaskCreateDualProblem::run()
{
    // Only run this task after intialization if we want to rebuild the tree in the multi-tree strategy
    if(env->settings->getSetting<bool>("TreeStrategy.Multi.Reinitialize", "Dual"))
    {
        env->timing->startTimer("DualStrategy");

        env->output->outputDebug("        Recreating dual problem");

        createProblem(env->dualSolver->MIPSolver, env->reformulatedProblem);

        env->dualSolver->MIPSolver->finalizeProblem();

        env->dualSolver->MIPSolver->initializeSolverSettings();

        if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            env->dualSolver->MIPSolver->writeProblemToFile(
                env->settings->getSetting<std::string>("Debug.Path", "Output") + "/lp0.lp");
        }

        env->output->outputDebug("        Dual problem recreated");
        env->timing->stopTimer("DualStrategy");
    }
}

bool TaskCreateDualProblem::createProblem(MIPSolverPtr destination, ProblemPtr sourceProblem)
{
    // Now creating the variables

    bool variablesInitialized = true;

    for(auto& V : sourceProblem->allVariables)
    {
        variablesInitialized = variablesInitialized
            && destination->addVariable(
                V->name.c_str(), V->properties.type, V->lowerBound, V->upperBound, V->semiBound);
    }

    if(!variablesInitialized)
        return false;

    if(sourceProblem->auxiliaryObjectiveVariable) // The source problem already has a nonlinear objective variable
    {
        destination->setDualAuxiliaryObjectiveVariableIndex(sourceProblem->auxiliaryObjectiveVariable->index);
    }
    else if(sourceProblem->objectiveFunction->properties.classification > E_ObjectiveFunctionClassification::Quadratic)
    {
        double objVarBound = env->settings->getSetting<double>("Variables.NonlinearObjectiveVariable.Bound", "Model");

        Interval objectiveBound;

        try
        {
            objectiveBound = sourceProblem->objectiveFunction->getBounds();
        }
        catch(mc::Interval::Exceptions&)
        {
            objectiveBound = Interval(-objVarBound, objVarBound);
        }

        destination->setDualAuxiliaryObjectiveVariableIndex(sourceProblem->properties.numberOfVariables);

        destination->addVariable("shot_dual_objvar", E_VariableType::Real, objectiveBound.l(), objectiveBound.u(), 0.0);

        env->output->outputDebug(fmt::format(
            "         SHOT internal dual objective variable created with index {} and bounds [{},{}] created.",
            sourceProblem->properties.numberOfVariables, objectiveBound.l(), objectiveBound.u()));
    }

    // Now creating the objective function

    bool objectiveInitialized = true;

    objectiveInitialized = objectiveInitialized && destination->initializeObjective();

    if(destination->hasDualAuxiliaryObjectiveVariable())
    {
        objectiveInitialized = objectiveInitialized
            && destination->addLinearTermToObjective(1.0, destination->getDualAuxiliaryObjectiveVariableIndex());

        objectiveInitialized = objectiveInitialized
            && destination->finalizeObjective(sourceProblem->objectiveFunction->properties.isMinimize);
    }
    else
    {
        // Linear terms
        for(auto& T : std::dynamic_pointer_cast<LinearObjectiveFunction>(sourceProblem->objectiveFunction)->linearTerms)
        {
            objectiveInitialized
                = objectiveInitialized && destination->addLinearTermToObjective(T->coefficient, T->variable->index);
        }

        // Quadratic terms
        if(sourceProblem->objectiveFunction->properties.hasQuadraticTerms)
        {
            for(auto& T :
                std::dynamic_pointer_cast<QuadraticObjectiveFunction>(sourceProblem->objectiveFunction)->quadraticTerms)
            {
                objectiveInitialized = objectiveInitialized
                    && destination->addQuadraticTermToObjective(
                        T->coefficient, T->firstVariable->index, T->secondVariable->index);
            }
        }

        objectiveInitialized = objectiveInitialized
            && destination->finalizeObjective(
                sourceProblem->objectiveFunction->properties.isMinimize, sourceProblem->objectiveFunction->constant);
    }

    if(!objectiveInitialized)
        return false;

    // Now creating the constraints

    bool constraintsInitialized = true;

    for(auto& C : sourceProblem->linearConstraints)
    {
        constraintsInitialized = constraintsInitialized && destination->initializeConstraint();

        if(C->properties.hasLinearTerms)
        {
            for(auto& T : C->linearTerms)
            {
                constraintsInitialized = constraintsInitialized
                    && destination->addLinearTermToConstraint(T->coefficient, T->variable->index);
            }
        }

        constraintsInitialized
            = constraintsInitialized && destination->finalizeConstraint(C->name, C->valueLHS, C->valueRHS, C->constant);
    }

    for(auto& C : sourceProblem->quadraticConstraints)
    {
        constraintsInitialized = constraintsInitialized && destination->initializeConstraint();

        if(C->properties.hasLinearTerms)
        {
            for(auto& T : C->linearTerms)
            {
                constraintsInitialized = constraintsInitialized
                    && destination->addLinearTermToConstraint(T->coefficient, T->variable->index);
            }
        }

        if(C->properties.hasQuadraticTerms)
        {
            for(auto& T : C->quadraticTerms)
            {
                constraintsInitialized = constraintsInitialized
                    && destination->addQuadraticTermToConstraint(
                        T->coefficient, T->firstVariable->index, T->secondVariable->index);
            }
        }

        constraintsInitialized
            = constraintsInitialized && destination->finalizeConstraint(C->name, C->valueLHS, C->valueRHS, C->constant);
    }

    if(!constraintsInitialized)
        return false;

    bool SOSInitialized = true;

    for(auto& S : sourceProblem->specialOrderedSets)
    {
        std::vector<int> variableIndexes;
        std::vector<double> variableWeights;

        for(auto& V : S->variables)
            variableIndexes.push_back(V->index);

        if(S->weights.size() > 0)
        {
            for(auto& W : S->weights)
                variableWeights.push_back(W);

            SOSInitialized
                = SOSInitialized && destination->addSpecialOrderedSet(S->type, variableIndexes, variableWeights);
        }
        else
        {
            SOSInitialized = SOSInitialized && destination->addSpecialOrderedSet(S->type, variableIndexes);
        }
    }

    if(!SOSInitialized)
        return false;

    bool problemFinalized = destination->finalizeProblem();

    return (problemFinalized);
}

std::string TaskCreateDualProblem::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT