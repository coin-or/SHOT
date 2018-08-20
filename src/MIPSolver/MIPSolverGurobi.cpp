/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolverGurobi.h"

MIPSolverGurobi::MIPSolverGurobi()
{
    //Should not be called
}

MIPSolverGurobi::MIPSolverGurobi(EnvironmentPtr envPtr)
{
    env = envPtr;

    discreteVariablesActivated = true;

    try
    {
        gurobiEnv = new GRBEnv();
        gurobiModel = new GRBModel(*gurobiEnv);
    }
    catch (GRBException &e)
    {
        {
            env->output->outputError("Error when initializing Gurobi:", e.getMessage());
        }

        return;
    }

    cachedSolutionHasChanged = true;
    isVariablesFixed = false;

    checkParameters();
}

MIPSolverGurobi::~MIPSolverGurobi()
{
    delete gurobiModel;
    delete gurobiEnv;
}

bool MIPSolverGurobi::createLinearProblem(OptProblem *origProblem)
{
    originalProblem = origProblem;

    try
    {
        auto numVar = origProblem->getNumberOfVariables();
        auto tmpLBs = origProblem->getVariableLowerBounds();
        auto tmpUBs = origProblem->getVariableUpperBounds();
        auto tmpNames = origProblem->getVariableNames();
        auto tmpTypes = origProblem->getVariableTypes();

        int numCon = origProblem->getNumberOfConstraints();
        if (origProblem->isObjectiveFunctionNonlinear())
            numCon--;

        for (int i = 0; i < numVar; i++)
        {
            if (tmpTypes.at(i) == 'C')
            {
                auto tmpLB = tmpLBs.at(i);
                auto tmpUB = tmpUBs.at(i);

                GRBVar tmpVar = gurobiModel->addVar(tmpLB, tmpUB, 0.0, GRB_CONTINUOUS, tmpNames.at(i));
            }
            else if (tmpTypes.at(i) == 'I')
            {
                auto tmpLB = tmpLBs.at(i);
                auto tmpUB = tmpUBs.at(i);

                GRBVar tmpVar = gurobiModel->addVar(tmpLB, tmpUB, 0.0, GRB_INTEGER, tmpNames.at(i));
            }
            else if (tmpTypes.at(i) == 'B')
            {
                GRBVar tmpVar = gurobiModel->addVar(tmpLBs.at(i), tmpUBs.at(i), 0.0, GRB_BINARY, tmpNames.at(i));
            }
            else if (tmpTypes.at(i) == 'D')
            {
                GRBVar tmpVar = gurobiModel->addVar(tmpLBs.at(i), tmpUBs.at(i), 0.0, GRB_SEMICONT, tmpNames.at(i));
            }
            else
            {
                env->output->outputWarning(
                    "Error variable type " + std::to_string(tmpTypes.at(i)) + " for " + tmpNames.at(i));
            }
        }

        gurobiModel->update();

        auto tmpObjPairs = origProblem->getObjectiveFunctionVarCoeffPairs();

        gurobiModel->set(GRB_DoubleAttr_ObjCon, origProblem->getObjectiveConstant());

        if (origProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic)
        {
            GRBLinExpr *expr = new GRBLinExpr(0);

            for (int i = 0; i < tmpObjPairs.size(); i++)
            {
                *expr += +(tmpObjPairs.at(i).second) * (gurobiModel->getVar(tmpObjPairs.at(i).first));
            }

            double objConstant = origProblem->getObjectiveConstant();
            if (objConstant != 0.0)
                *expr += objConstant;

            gurobiModel->setObjective(*expr);

            delete expr;
        }
        else
        {
            GRBQuadExpr *expr = new GRBQuadExpr(0);

            for (int i = 0; i < tmpObjPairs.size(); i++)
            {
                *expr += (tmpObjPairs.at(i).second) * (gurobiModel->getVar(tmpObjPairs.at(i).first));
            }

            auto quadTerms = origProblem->getQuadraticTermsInConstraint(-1);

            for (auto T : quadTerms)
            {
                *expr += (T->coef * gurobiModel->getVar(T->idxOne) * gurobiModel->getVar(T->idxTwo));
            }

            double objConstant = origProblem->getObjectiveConstant();
            if (objConstant != 0.0)
                *expr += objConstant;

            gurobiModel->setObjective(*expr);

            delete expr;
        }

        gurobiModel->update();

        if (origProblem->isTypeOfObjectiveMinimize())
        {
            gurobiModel->set(GRB_IntAttr_ModelSense, 1);
        }
        else
        {
            gurobiModel->set(GRB_IntAttr_ModelSense, -1);
        }

        gurobiModel->update();

        // Now creating the constraints

        int row_nonz = 0;
        int obj_nonz = 0;
        int varIdx = 0;

        SparseMatrix *m_linearConstraintCoefficientsInRowMajor =
            origProblem->getProblemInstance()->getLinearConstraintCoefficientsInRowMajor();

        auto constrTypes = origProblem->getProblemInstance()->getConstraintTypes();
        auto constrNames = origProblem->getProblemInstance()->getConstraintNames();
        auto constrLBs = origProblem->getProblemInstance()->getConstraintLowerBounds();
        auto constrUBs = origProblem->getProblemInstance()->getConstraintUpperBounds();

        for (int rowIdx = 0; rowIdx < numCon; rowIdx++)
        {
            // Only use constraints that don't contain a nonlinear part (may include a quadratic part)
            if (!origProblem->isConstraintNonlinear(rowIdx))
            {
                auto quadTerms = origProblem->getQuadraticTermsInConstraint(rowIdx);

                if (quadTerms.size() == 0)
                {
                    GRBLinExpr *expr = new GRBLinExpr(0);
                    *expr += origProblem->getProblemInstance()->instanceData->constraints->con[rowIdx]->constant;

                    if (origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients != NULL && origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients->numberOfValues > 0)
                    {
                        row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1] - m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];

                        for (int j = 0; j < row_nonz; j++)
                        {
                            double val =
                                m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
                            varIdx =
                                m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
                            auto variable = gurobiModel->getVar(varIdx);

                            *expr += val * variable;
                        }
                    }
                    if (constrTypes[rowIdx] == 'L')
                    {
                        gurobiModel->addConstr(*expr <= constrUBs[rowIdx], constrNames[rowIdx]);
                    }
                    else if (constrTypes[rowIdx] == 'G')
                    {
                        gurobiModel->addConstr(*expr >= constrLBs[rowIdx], constrNames[rowIdx]);
                    }
                    else if (constrTypes[rowIdx] == 'E')
                    {
                        gurobiModel->addConstr(*expr == constrUBs[rowIdx], constrNames[rowIdx]);
                    }
                    else
                    {
                    }

                    delete expr;
                }
                else
                {
                    GRBQuadExpr *expr = new GRBQuadExpr(0);
                    *expr += origProblem->getProblemInstance()->instanceData->constraints->con[rowIdx]->constant;

                    if (origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients != NULL && origProblem->getProblemInstance()->instanceData->linearConstraintCoefficients->numberOfValues > 0)
                    {
                        row_nonz = m_linearConstraintCoefficientsInRowMajor->starts[rowIdx + 1] - m_linearConstraintCoefficientsInRowMajor->starts[rowIdx];

                        for (int j = 0; j < row_nonz; j++)
                        {
                            double val =
                                m_linearConstraintCoefficientsInRowMajor->values[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
                            varIdx =
                                m_linearConstraintCoefficientsInRowMajor->indexes[m_linearConstraintCoefficientsInRowMajor->starts[rowIdx] + j];
                            auto variable = gurobiModel->getVar(varIdx);

                            *expr += val * variable;
                        }
                    }

                    for (auto T : quadTerms)
                    {
                        *expr += (T->coef * gurobiModel->getVar(T->idxOne) * gurobiModel->getVar(T->idxTwo));
                    }

                    if (constrTypes[rowIdx] == 'L')
                    {
                        gurobiModel->addQConstr(*expr <= constrUBs[rowIdx], constrNames[rowIdx]);
                    }
                    else if (constrTypes[rowIdx] == 'G')
                    {
                        gurobiModel->addQConstr(*expr >= constrLBs[rowIdx], constrNames[rowIdx]);
                    }
                    else if (constrTypes[rowIdx] == 'E')
                    {
                        gurobiModel->addQConstr(*expr == constrUBs[rowIdx], constrNames[rowIdx]);
                    }
                    else
                    {
                    }

                    delete expr;
                }
            }
        }

        gurobiModel->update();
    }
    catch (GRBException &e)
    {
        {
            env->output->outputError("Error when creating linear problem:", e.getMessage());
        }

        return (false);
    }

    return (true);
}

void MIPSolverGurobi::initializeSolverSettings()
{
    try
    {
        if (!env->settings->getBoolSetting("Console.DualSolver.Show", "Output"))
        {
            gurobiModel->getEnv().set(GRB_IntParam_OutputFlag, 0);
        }

        gurobiModel->getEnv().set(GRB_DoubleParam_MIPGap, env->settings->getDoubleSetting("ObjectiveGap.Relative", "Termination") / 1.0);
        gurobiModel->getEnv().set(GRB_DoubleParam_MIPGapAbs, env->settings->getDoubleSetting("ObjectiveGap.Absolute", "Termination") / 1.0);

        // Default 0 to fix som problems with some problems
        gurobiModel->getEnv().set(GRB_IntParam_ScaleFlag, env->settings->getIntSetting("Gurobi.ScaleFlag", "Subsolver"));
        gurobiModel->getEnv().set(GRB_IntParam_NumericFocus, env->settings->getIntSetting("Gurobi.NumericFocus", "Subsolver"));
        gurobiModel->getEnv().set(GRB_IntParam_MIPFocus, env->settings->getIntSetting("Gurobi.MIPFocus", "Subsolver"));
        //gurobiModel->getEnv().set(GRB_DoubleParam_FeasibilityTol, 1e-6);
        //gurobiModel->getEnv().set(GRB_DoubleParam_IntFeasTol, 1e-6);
        //gurobiModel->getEnv().set(GRB_DoubleParam_OptimalityTol, 1e-6);
        //gurobiModel->getEnv().set(GRB_DoubleParam_MarkowitzTol, 1e-4);
        //gurobiModel->getEnv().set(GRB_DoubleParam_NodeLimit, 1e15);
        gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
        gurobiModel->getEnv().set(GRB_IntParam_SolutionNumber, env->settings->getIntSetting("MIP.SolutionPool.Capacity", "Dual") + 1);
    }
    catch (GRBException &e)
    {
        {
            env->output->outputError("Error when initializing parameters for linear solver",
                                     e.getMessage());
        }
    }
}

int MIPSolverGurobi::addLinearConstraint(std::vector<PairIndexValue> elements, double constant, bool isGreaterThan)
{
    try
    {
        GRBLinExpr *expr = new GRBLinExpr(0.0);

        for (int i = 0; i < elements.size(); i++)
        {
            auto variable = gurobiModel->getVar(elements.at(i).index);
            *expr = *expr + elements.at(i).value * variable;
        }

        if (isGreaterThan)
        {
            gurobiModel->addConstr(-constant >= *expr, "");
        }
        else
        {
            gurobiModel->addConstr(*expr <= -constant, "");
        }

        delete expr;

        gurobiModel->update();
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when adding linear constraint", e.getMessage());

        return (-1);
    }

    return (gurobiModel->get(GRB_IntAttr_NumConstrs) - 1);
}

VectorDouble MIPSolverGurobi::getVariableSolution(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();

    int numVar = gurobiModel->get(GRB_IntAttr_NumVars);
    VectorDouble solution(numVar);

    try
    {
        if (isMIP && solIdx > 0)
        {
            gurobiModel->getEnv().set(GRB_IntParam_SolutionNumber, solIdx);

            for (int i = 0; i < numVar; i++)
            {
                GRBVar tmpVar = gurobiModel->getVar(i);
                solution.at(i) = tmpVar.get(GRB_DoubleAttr_Xn);
            }
        }
        else
        {
            for (int i = 0; i < numVar; i++)
            {
                GRBVar tmpVar = gurobiModel->getVar(i);
                solution.at(i) = tmpVar.get(GRB_DoubleAttr_X);
            }
        }
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when reading solution with index " + std::to_string(solIdx),
                                 e.getMessage());
    }

    return (solution);
}

int MIPSolverGurobi::getNumberOfSolutions()
{
    int numSols = 0;

    numSols = gurobiModel->get(GRB_IntAttr_SolCount);

    return (numSols);
}

void MIPSolverGurobi::activateDiscreteVariables(bool activate)
{
    auto variableTypes = env->model->originalProblem->getVariableTypes();
    int numVar = env->model->originalProblem->getNumberOfVariables();

    if (activate)
    {
        env->output->outputInfo("Activating MIP strategy.");

        for (int i = 0; i < numVar; i++)
        {
            if (variableTypes.at(i) == 'I')
            {
                GRBVar tmpVar = gurobiModel->getVar(i);

                tmpVar.set(GRB_CharAttr_VType, 'I');
            }
            else if (variableTypes.at(i) == 'B')
            {
                GRBVar tmpVar = gurobiModel->getVar(i);

                tmpVar.set(GRB_CharAttr_VType, 'B');
            }
        }

        discreteVariablesActivated = true;
    }
    else
    {
        env->output->outputInfo("Activating LP strategy.");
        for (int i = 0; i < numVar; i++)
        {
            if (variableTypes.at(i) == 'I' || variableTypes.at(i) == 'B')
            {
                GRBVar tmpVar = gurobiModel->getVar(i);

                tmpVar.set(GRB_CharAttr_VType, 'C');
            }
        }

        discreteVariablesActivated = false;
    }

    gurobiModel->update();
}

E_ProblemSolutionStatus MIPSolverGurobi::getSolutionStatus()
{
    E_ProblemSolutionStatus MIPSolutionStatus;

    int status = gurobiModel->get(GRB_IntAttr_Status);

    if (status == GRB_OPTIMAL)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Optimal;
    }
    else if (status == GRB_INFEASIBLE)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
    }
    else if (status == GRB_INF_OR_UNBD)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Infeasible;
    }
    else if (status == GRB_UNBOUNDED)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if (status == GRB_ITERATION_LIMIT)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if (status == GRB_NODE_LIMIT)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Unbounded;
    }
    else if (status == GRB_TIME_LIMIT)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::TimeLimit;
    }
    else if (status == GRB_SOLUTION_LIMIT)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::SolutionLimit;
    }
    else if (status == GRB_INTERRUPTED)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Abort;
    }
    else if (status == GRB_NUMERIC)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Numeric;
    }
    else if (status == GRB_CUTOFF)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::CutOff;
    }
    else if (status == GRB_SUBOPTIMAL)
    {
        MIPSolutionStatus = E_ProblemSolutionStatus::Feasible;
    }
    else
    {
        env->output->outputError("MIP solver return status " + std::to_string(status));
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

E_ProblemSolutionStatus MIPSolverGurobi::solveProblem()
{
    E_ProblemSolutionStatus MIPSolutionStatus;
    cachedSolutionHasChanged = true;

    try
    {
        GurobiInfoCallback gurobiCallback = GurobiInfoCallback(env);
        gurobiModel->setCallback(&gurobiCallback);
        gurobiModel->optimize();

        MIPSolutionStatus = getSolutionStatus();
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when solving MIP/LP problem", e.getMessage());
        MIPSolutionStatus = E_ProblemSolutionStatus::Error;
    }

    return (MIPSolutionStatus);
}

int MIPSolverGurobi::increaseSolutionLimit(int increment)
{
    gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit,
                              gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit) + increment);

    return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobi::setSolutionLimit(long limit)
{
    if (limit > GRB_MAXINT)
        gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, GRB_MAXINT);
    else
        gurobiModel->getEnv().set(GRB_IntParam_SolutionLimit, limit);
}

int MIPSolverGurobi::getSolutionLimit()
{
    return (gurobiModel->getEnv().get(GRB_IntParam_SolutionLimit));
}

void MIPSolverGurobi::setTimeLimit(double seconds)
{
    try
    {
        if (seconds > 0)
        {
            gurobiModel->getEnv().set(GRB_DoubleParam_TimeLimit, seconds);
        }
        else
        {
            gurobiModel->getEnv().set(GRB_DoubleParam_TimeLimit, 0.00001);
        }
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when setting time limit", e.getMessage());
    }
}

void MIPSolverGurobi::setCutOff(double cutOff)
{
    try
    {
        // Gurobi has problems if not an epsilon value is added to the cutoff...

        double cutOffTol = env->settings->getDoubleSetting("MIP.CutOffTolerance", "Dual");

        if (originalProblem->isTypeOfObjectiveMinimize())
        {
            gurobiModel->getEnv().set(GRB_DoubleParam_Cutoff, cutOff + cutOffTol);

            env->output->outputInfo(
                "     Setting cutoff value to " + std::to_string(cutOff) + " for minimization.");
        }
        else
        {
            gurobiModel->getEnv().set(GRB_DoubleParam_Cutoff, cutOff - cutOffTol);
            env->output->outputInfo(
                "     Setting cutoff value to " + std::to_string(cutOff) + " for maximization.");
        }
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when setting cut off value", e.getMessage());
    }
}

void MIPSolverGurobi::addMIPStart(VectorDouble point)
{
    int numVar = gurobiModel->get(GRB_IntAttr_NumVars);
    VectorDouble solution(numVar);

    try
    {
        for (int i = 0; i < numVar; i++)
        {
            GRBVar tmpVar = gurobiModel->getVar(i);
            tmpVar.set(GRB_DoubleAttr_Start, point.at(i));
        }
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when adding MIP starting point", e.getMessage());
    }

    env->output->outputInfo("      Added MIP starting point.");
}

void MIPSolverGurobi::writeProblemToFile(std::string filename)
{
    try
    {
        gurobiModel->write(filename);
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when saving model to file", e.getMessage());
    }
}

double MIPSolverGurobi::getObjectiveValue(int solIdx)
{
    bool isMIP = getDiscreteVariableStatus();

    double objVal = NAN;

    if (!isMIP && solIdx > 0) // LP problems only have one solution!
    {
        env->output->outputError(
            "Cannot obtain solution with index " + std::to_string(solIdx) + " since the problem is LP/QP!");

        return (objVal);
    }

    try
    {
        if ((isMIP && solIdx == 0) || !isMIP)
        {
            objVal = gurobiModel->get(GRB_DoubleAttr_ObjVal);
        }
        else // Gurobi has no functionality to access the objective value of a specific solution
        {
            gurobiModel->getEnv().set(GRB_IntParam_SolutionNumber, solIdx);

            int numvars = gurobiModel->get(GRB_IntAttr_NumVars);

            auto objective = gurobiModel->getObjective();
            objVal = objective.getLinExpr().getConstant();

            for (int i = 0; i < objective.size(); i++)
            {
                objVal += objective.getCoeff(i) * objective.getVar1(i).get(GRB_DoubleAttr_Xn) * objective.getVar2(i).get(GRB_DoubleAttr_Xn);
            }

            auto linexpr = objective.getLinExpr();

            for (int i = 0; i < linexpr.size(); i++)
            {
                objVal += linexpr.getCoeff(i) * linexpr.getVar(i).get(GRB_DoubleAttr_Xn);
            }
        }
    }
    catch (GRBException &e)
    {
        env->output->outputError(
            "Error when obtaining objective value for solution index " + std::to_string(solIdx), e.getMessage());
    }

    return (objVal);
}

void MIPSolverGurobi::deleteMIPStarts()
{
    int numVar = gurobiModel->get(GRB_IntAttr_NumVars);
    VectorDouble solution(numVar);

    try
    {
        for (int i = 0; i < numVar; i++)
        {
            GRBVar tmpVar = gurobiModel->getVar(i);
            tmpVar.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
        }
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when deleting MIP starting points", e.getMessage());
    }

    env->output->outputDebug("    Deleted MIP starting points.");
}

void MIPSolverGurobi::fixVariable(int varIndex, double value)
{
    updateVariableBound(varIndex, value, value);
}

void MIPSolverGurobi::updateVariableBound(int varIndex, double lowerBound, double upperBound)
{
    try
    {
        GRBVar tmpVar = gurobiModel->getVar(varIndex);

        tmpVar.set(GRB_DoubleAttr_LB, lowerBound);
        tmpVar.set(GRB_DoubleAttr_UB, upperBound);

        gurobiModel->update();
    }
    catch (GRBException &e)
    {
        env->output->outputError(
            "Error when updating variable bounds for variable index" + std::to_string(varIndex), e.getMessage());
    }
}

PairDouble MIPSolverGurobi::getCurrentVariableBounds(int varIndex)
{
    PairDouble tmpBounds;

    try
    {
        GRBVar tmpVar = gurobiModel->getVar(varIndex);

        tmpBounds.first = tmpVar.get(GRB_DoubleAttr_LB);
        tmpBounds.second = tmpVar.get(GRB_DoubleAttr_UB);

        gurobiModel->update();
    }
    catch (GRBException &e)
    {
        env->output->outputError(
            "Error when obtaining variable bounds for variable index" + std::to_string(varIndex), e.getMessage());
    }

    return (tmpBounds);
}

bool MIPSolverGurobi::supportsQuadraticObjective()
{
    return (true);
}
bool MIPSolverGurobi::supportsQuadraticConstraints()
{
    return (true);
}

double MIPSolverGurobi::getDualObjectiveValue()
{

    bool isMIP = getDiscreteVariableStatus();
    double objVal = NAN;

    try
    {
        if (isMIP)
        {
            objVal = gurobiModel->get(GRB_DoubleAttr_ObjBound);
        }
        else
        {
            objVal = gurobiModel->get(GRB_DoubleAttr_ObjVal);
        }
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when obtaining dual objective value", e.getMessage());
    }

    return (objVal);
}

void MIPSolverGurobi::writePresolvedToFile(std::string filename)
{
}

void MIPSolverGurobi::checkParameters()
{
}

std::pair<VectorDouble, VectorDouble> MIPSolverGurobi::presolveAndGetNewBounds()
{
    return (std::make_pair(originalProblem->getVariableLowerBounds(), env->model->originalProblem->getVariableLowerBounds()));
}

int MIPSolverGurobi::getNumberOfExploredNodes()
{
    try
    {
        return ((int)gurobiModel->get(GRB_DoubleAttr_NodeCount));
    }
    catch (GRBException &e)
    {
        env->output->outputError("Error when getting number of nodes", e.getMessage());
        return 0;
    }
}

GurobiInfoCallback::GurobiInfoCallback(EnvironmentPtr envPtr) : env(envPtr)
{
}

// Used to get the number of open nodes
void GurobiInfoCallback::callback()
{
    try
    {
        if (where == GRB_CB_MIP)
        {
            auto currIter = env->process->getCurrentIteration();
            currIter->numberOfExploredNodes = (int)getDoubleInfo(GRB_CB_MIP_NODCNT);
            currIter->numberOfOpenNodes = (int)getDoubleInfo(GRB_CB_MIP_NODLFT);
        }
    }
    catch (GRBException &e)
    {
        env->output->outputError("Gurobi error when running main callback method", e.getMessage());
    }
}
