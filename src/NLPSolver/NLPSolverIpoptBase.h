/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "NLPSolverBase.h"
#include "../SharedOS.h"

#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"

namespace SHOT
{

class NLPSolverIpoptBase;

// The following class is adapted from COIN-OR Optimization Services Ipopt interface
class IpoptProblem : public Ipopt::TNLP
{
public:
    bool hasSolution = false;
    VectorDouble variableSolution;
    double objectiveValue;

    E_NLPSolutionStatus solutionStatus;
    std::string solutionDescription;

    /** the IpoptProblemclass constructor */
    IpoptProblem(EnvironmentPtr envPtr, NLPSolverIpoptBase* ipoptSolver, ProblemPtr problem);
    virtual ~IpoptProblem() = default;

    /** IPOpt specific methods for defining the nlp problem */
    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag,
        IndexStyleEnum& index_style);

    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(

        Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(Ipopt::Index n, bool init_sx, Ipopt::Number* x, bool init_z, Ipopt::Number* z_L,
        Ipopt::Number* z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda);

    /** Method to return the objective value */
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value);

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f);

    /** Method to return the constraint residuals */
    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g);

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac,
        Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values);

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m,
        const Ipopt::Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol,
        Ipopt::Number* values);

    virtual bool get_scaling_parameters(Ipopt::Number& obj_scaling, bool& use_x_scaling, Ipopt::Index n,
        Ipopt::Number* x_scaling, bool& use_g_scaling, Ipopt::Index m, Ipopt::Number* g_scaling);

    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x,
        const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
        const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data,
        Ipopt::IpoptCalculatedQuantities* ip_cq);

private:
    ProblemPtr sourceProblem;

    EnvironmentPtr env;

    NLPSolverIpoptBase* ipoptSolver;

    std::map<std::pair<int, int>, int> lagrangianHessianCounterPlacement;
    std::map<std::pair<int, int>, int> jacobianCounterPlacement;
};

class NLPSolverIpoptBase : virtual public INLPSolver
{
    friend IpoptProblem;

private:
protected:
    std::shared_ptr<IpoptProblem> ipoptProblem;
    ProblemPtr sourceProblem;

    std::unique_ptr<Ipopt::IpoptApplication> ipoptApplication;

    VectorInteger fixedVariableIndexes;
    VectorDouble fixedVariableValues;

    VectorInteger startingPointVariableIndexes;
    VectorDouble startingPointVariableValues;

    VectorDouble lowerBounds;
    VectorDouble upperBounds;

    virtual E_NLPSolutionStatus solveProblemInstance();

    void fixVariables(VectorInteger variableIndexes, VectorDouble variableValues);
    void unfixVariables();

    virtual void setInitialSettings();
    virtual void setSolverSpecificInitialSettings() = 0;
    virtual void updateSettings();

    virtual VectorDouble getVariableLowerBounds();
    virtual VectorDouble getVariableUpperBounds();

    virtual void updateVariableLowerBound(int variableIndex, double bound);
    virtual void updateVariableUpperBound(int variableIndex, double bound);

    VectorDouble lowerBoundsBeforeFix;
    VectorDouble upperBoundsBeforeFix;

    std::vector<E_VariableType> originalVariableType;

public:
    virtual ~NLPSolverIpoptBase(){};

    virtual void setStartingPoint(VectorInteger variableIndexes, VectorDouble variableValues);
    virtual void clearStartingPoint();

    virtual VectorDouble getSolution();
    virtual double getSolution(int i);
    virtual double getObjectiveValue();

    virtual void saveOptionsToFile(std::string fileName);
    virtual void saveProblemToFile(std::string fileName);
};
} // namespace SHOT