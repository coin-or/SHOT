/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Solver.h"

#include "Output.h"
#include "Settings.h"

#ifdef HAS_GAMS
#include "ModelingSystem/ModelingSystemGAMS.h"
#endif
#ifdef HAS_AMPL
#include "ModelingSystem/ModelingSystemAMPL.h"
#endif
#include "ModelingSystem/ModelingSystemOSiL.h"

namespace SHOT
{

void Solver::initializeSettings()
{
    if(env->settings->settingsInitialized)
    {
        env->output->outputWarning(" Warning! Settings have already been initialized. Ignoring new settings.");
        return;
    }

    std::string empty; // Used to create empty string options

    env->output->outputDebug(" Starting initialization of settings:");

    env->settings->createSettingGroup("Dual", "", "Dual strategy",
        "These settings control the various functionality of the dual strategy in SHOT, i.e., the polyhedral outer "
        "approximation utilizing the ESH or ECP algorithms.");

    // Dual strategy settings: ECP and ESH

    VectorString enumHyperplanePointStrategy;
    enumHyperplanePointStrategy.push_back("ESH");
    enumHyperplanePointStrategy.push_back("ECP");
    enumHyperplanePointStrategy.push_back("Only external (through callback)");
    env->settings->createSetting("Dual.CutStrategy", static_cast<int>(ES_HyperplaneCutStrategy::ESH),
        "Dual cut strategy", enumHyperplanePointStrategy, 0);
    enumHyperplanePointStrategy.clear();

    env->settings->createSettingGroup("Dual", "ESH", "Extended supporting hyperplane method",
        "These settings control various aspects of the ESH implementation, including the strategy to obtain the "
        "interior point.");

    // Dual strategy settings: Interior point search strategy

    VectorString enumInteriorPointStrategy;
    enumInteriorPointStrategy.push_back("Use internal strategy");
    enumInteriorPointStrategy.push_back("Only external (through callback)");
    env->settings->createSetting("Dual.ESH.InteriorPoint.Strategy",
        static_cast<int>(ES_ESHInteriorPointStrategy::UseInternalStrategy), "Strategy for finding ESH interior points",
        enumInteriorPointStrategy, 0);
    enumInteriorPointStrategy.clear();

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.BitPrecision", 8,
        "Required termination bit precision for minimization subsolver (at most half the digits of a double)", 1, 26,
        true);

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.ConstraintSelectionFactor", 0.25,
        "Max number of violated nonlinear constraints, the most violated first, to generate cutting planes "
        "for in the interior point search, as a fraction of the number of nonlinear constraints",
        0.0, 1.0);

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.IterationLimit", 100,
        "Iteration limit for minimax cutting plane solver", 1, SHOT_INT_MAX);

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.IterationLimitSubsolver", 100,
        "Iteration limit for minimization subsolver", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "Dual.ESH.InteriorPoint.CuttingPlane.TimeLimit", 10.0, "Time limit for minimax solver", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "Dual.ESH.InteriorPoint.CuttingPlane.Reuse", false, "Reuse valid cutting planes in main dual model");

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.TerminationToleranceAbs", 1.0,
        "Absolute termination tolerance between LP and linesearch objective", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.ESH.InteriorPoint.CuttingPlane.TerminationToleranceRel", 1.0,
        "Relative termination tolerance between LP and linesearch objective", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.ESH.InteriorPoint.MinimaxObjectiveLowerBound", -1e12,
        "Lower bound for minimax objective variable", SHOT_DBL_MIN, 0);

    env->settings->createSetting("Dual.ESH.InteriorPoint.MinimaxObjectiveUpperBound", 0.1,
        "Upper bound for minimax objective variable", SHOT_DBL_MIN, SHOT_DBL_MAX);

    VectorString enumAddPrimalPointAsInteriorPoint;
    enumAddPrimalPointAsInteriorPoint.push_back("Only if there is no interior point");
    enumAddPrimalPointAsInteriorPoint.push_back("Add as new");
    enumAddPrimalPointAsInteriorPoint.push_back("Replace old");
    enumAddPrimalPointAsInteriorPoint.push_back("Use average");
    env->settings->createSetting("Dual.ESH.InteriorPoint.UsePrimalSolution",
        static_cast<int>(ES_AddPrimalPointAsInteriorPoint::KeepBoth), "Utilize primal solution as interior point",
        enumAddPrimalPointAsInteriorPoint, 0);
    enumAddPrimalPointAsInteriorPoint.clear();

    env->settings->createSetting("Dual.ESH.Rootsearch.ConstraintTolerance", 1e-8,
        "Constraint tolerance for when not to add individual hyperplanes", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.ESH.Rootsearch.UseMaxFunction", false,
        "Perform rootsearch on max function, otherwise on individual constraints");

    // Dual strategy settings: Hyperplane generation

    env->settings->createSettingGroup("Dual", "HyperplaneCuts", "Generated hyperplane cuts",
        "These settings control how the cutting planes or supporting hyperplanes are generated.");

    env->settings->createSetting("Dual.HyperplaneCuts.ConstraintSelectionFactor", 0.5,
        "Max number of violated nonlinear constraints, the most violated first, to generate hyperplanes for "
        "in each solution point, as a fraction of the number of nonlinear constraints",
        0.0, 1.0);

    env->settings->createSetting(
        "Dual.HyperplaneCuts.Delay", true, "Add hyperplane cuts to model only after optimal MIP solution");

    env->settings->createSetting("Dual.HyperplaneCuts.MaxConstraintFactor", 0.1,
        "Rootsearch performed on constraints with values larger than this factor times the maximum value", 1e-6, 1.0);

    env->settings->createSetting("Dual.HyperplaneCuts.MaxPerIteration", 200,
        "Maximal number of hyperplanes to add per iteration", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.HyperplaneCuts.NonfinitePointRetreatFactor", 1e-3,
        "The smallest fraction of the distance to a point where the constraint is finite that the point a "
        "hyperplane is generated in is moved, when the constraint is not finite in it",
        1e-12, 0.5);

    env->settings->createSetting(
        "Dual.HyperplaneCuts.UniqueConstraints", false, "Allow only one hyperplane per constraint per iteration");

    env->settings->createSetting("Dual.HyperplaneCuts.UseIntegerCuts", false,
        "Add integer cuts for infeasible integer-combinations for binary problems");

    VectorString enumObjectiveRootsearch;
    enumObjectiveRootsearch.push_back("Always");
    enumObjectiveRootsearch.push_back("IfConvex");
    enumObjectiveRootsearch.push_back("Never");
    env->settings->createSetting("Dual.HyperplaneCuts.ObjectiveRootSearch",
        static_cast<int>(ES_ObjectiveRootsearch::IfConvex), "When to use the objective root search",
        enumObjectiveRootsearch, 0);
    enumObjectiveRootsearch.clear();

    // Dual strategy settings: MIP solver

    env->settings->createSettingGroup("Dual", "MIP", "MIP solver",
        "These settings control the general functionality of the MIP solver in the dual strategy. Note that "
        "solver-specific settings for Cplex, Gurobi and Cbc are available under the \"Subsolver\" category.");

    env->settings->createSetting(
        "Dual.MIP.CutOff.InitialValue", SHOT_DBL_MAX, "Initial cutoff value to use", SHOT_DBL_MIN, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.MIP.CutOff.UseInitialValue", false, "Use the initial cutoff value");

    env->settings->createSetting("Dual.MIP.CutOff.Tolerance", 0.00001,
        "An extra tolerance for the objective cutoff value (to prevent infeasible subproblems)", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "Dual.MIP.InfeasibilityRepair.IntegerCuts", false, "Allow feasibility repair of integer cuts");

    env->settings->createSetting("Dual.MIP.InfeasibilityRepair.CutResetLimit", 3,
        "Max number of times the cuts of nonconvex constraints are removed without primal objective value improvement "
        "when the infeasibility repair loops",
        0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.MIP.InfeasibilityRepair.IterationLimit", 100,
        "Max number of infeasible problems repaired without primal objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.MIP.InfeasibilityRepair.RelaxCutsViolatedByPrimal", true,
        "Relax the cuts of nonconvex constraints that cut off a known primal solution");

    env->settings->createSetting("Dual.MIP.InfeasibilityRepair.TimeLimit", 10.0,
        "Time limit when repairing infeasible problem", 0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "Dual.MIP.InfeasibilityRepair.Use", true, "Enable the infeasibility repair strategy for nonconvex problems");

    env->settings->createSetting("Dual.MIP.OptimalityTolerance", 1e-6,
        "The reduced-cost tolerance for optimality in the MIP solver", 1e-9, 1e-2);

    env->settings->createSetting(
        "Dual.MIP.NodeLimit", 0, "Node limit for each solve of the MIP solver: 0: No limit", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "Dual.MIP.NumberOfThreads", 0, "Number of threads to use in MIP solver: 0: Automatic", 0, 999);

    env->settings->createSetting("Dual.MIP.RandomSeed", 0,
        "Random seed for the pseudorandom choices in the MIP solver: 0: The default of the solver used", 0,
        SHOT_INT_MAX);

    env->settings->createSetting("Dual.MIP.SolutionLimit.ForceOptimal.Iteration", 10000,
        "Iterations without dual bound updates for forcing optimal MIP solution (multi-tree)", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.MIP.SolutionLimit.ForceOptimal.Time", 1000.0,
        "Time (s) without dual bound updates for forcing optimal MIP solution (multi-tree)", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.MIP.SolutionLimit.IncreaseFactor", 2.0,
        "Factor the MIP solution limit is multiplied by when increased: 1: Increase by one", 1.0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.MIP.SolutionLimit.IncreaseIterations", 50,
        "Max number of iterations between MIP solution limit increases", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.MIP.SolutionLimit.Initial", 1, "Initial MIP solution limit", 1, SHOT_INT_MAX);

    env->settings->createSetting("Dual.MIP.SolutionLimit.UpdateTolerance", 0.001,
        "The constraint tolerance for when to update MIP solution limit", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Dual.MIP.SolutionPool.Capacity", 100,
        "The maximum number of solutions in the solution pool of the MIP solver", 1, SHOT_INT_MAX);

    VectorString enumMIPSolver;
    enumMIPSolver.push_back("Cplex");
    enumMIPSolver.push_back("Gurobi");
    enumMIPSolver.push_back("Cbc");
    enumMIPSolver.push_back("Highs");

    ES_MIPSolver usedMIPSolver;

#ifdef HAS_GUROBI
    usedMIPSolver = ES_MIPSolver::Gurobi;
#elif HAS_CPLEX
    usedMIPSolver = ES_MIPSolver::Cplex;
#elif HAS_HIGHS
    // HiGHS before Cbc, as when the MIP solver chosen is not available
    usedMIPSolver = ES_MIPSolver::Highs;
#elif HAS_CBC
    usedMIPSolver = ES_MIPSolver::Cbc;
#else
    env->output->outputCritical(" SHOT has not been compiled with support for any MIP solver.");
#endif

    env->settings->createSetting(
        "Dual.MIP.Solver", static_cast<int>(usedMIPSolver), "Which MIP solver to use", enumMIPSolver, 0);
    enumMIPSolver.clear();

    env->settings->createSetting(
        "Dual.MIP.UpdateObjectiveBounds", false, "Update nonlinear objective variable bounds to primal/dual bounds");

    // Convex bounding

    env->settings->createSettingGroup("Dual", "ConvexBounding", "Convex Bounding",
        "These settings control the convex bounding strategy that solves a MIP problem with all hyperplane cuts "
        "generated for convex constraints so far and ignoring those generated for nonconvex constraints. This will "
        "give a dual bound for the nonconvex problem.");

    env->settings->createSetting(
        "Dual.ConvexBounding.Use", true, "Enable the convex bounding strategy for nonconvex problems");

    env->settings->createSetting("Dual.ConvexBounding.IdleIterations", 10,
        "How many iterations with new hyperplanes the convex bounding strategy is skipped before it is performed "
        "again: 0 = Every iteration with new hyperplanes, 1 = Every second such iteration, etc.",
        0, SHOT_INT_MAX);

    // Dual strategy settings: reduction cuts for nonconvex problems

    env->settings->createSettingGroup("Dual", "ReductionCut", "Dual reduction cut",
        "These settings control the added dual reduction cuts from the primal solution that will try to force a better "
        "primal solution. This functionality is only used if SHOT cannot deduce that the problem is convex.");

    env->settings->createSetting(
        "Dual.ReductionCut.Use", true, "Enable the dual reduction cut strategy for nonconvex problems");

    VectorString enumReductionCutStrategy;
    enumReductionCutStrategy.push_back("Fraction");
    enumReductionCutStrategy.push_back("GoldenRatio");

    ES_ReductionCutStrategy reductionCutStrategy;

    reductionCutStrategy = ES_ReductionCutStrategy::Fraction;

    env->settings->createSetting("Dual.ReductionCut.Strategy", static_cast<int>(reductionCutStrategy),
        "The reduction cut strategy to use", enumReductionCutStrategy,
        static_cast<int>(ES_ReductionCutStrategy::Fraction));
    enumReductionCutStrategy.clear();

    env->settings->createSetting("Dual.ReductionCut.MaxIterations", 20,
        "Maximum number of reduction cuts without primal improvement before terminating", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.ReductionCut.ReductionFactor", 0.001,
        "Reduction of the cutoff per reduction cut, relative to max(1, |cutoff|), in the Fraction strategy when the "
        "relative gap is at most 0.1 or above 1",
        0, 1.0);

    // Dual strategy settings: Relaxation strategies

    env->settings->createSettingGroup("Dual", "Relaxation", "Relaxation strategies",
        "These settings control various aspects regarding integer-relaxation of the dual problem.");

    env->settings->createSetting("Dual.Relaxation.Use", true, "Initially solve continuous dual relaxations");

    env->settings->createSetting("Dual.Relaxation.Frequency", 0,
        "The frequency to solve an LP problem, if Dual.Relaxation.Use is true (by default not for nonconvex problems): "
        "0: Disable",
        0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.Relaxation.IterationLimit", 200,
        "The max number of relaxed LP problems to solve initially", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.Relaxation.MaxLazyConstraints", 0,
        "Max number of lazy constraints to add in relaxed solutions in single-tree strategy", 0, SHOT_INT_MAX);

    env->settings->createSetting("Dual.Relaxation.TerminationTolerance", 0.5,
        "Stop solving relaxed problems when the maximum constraint deviation is at most this value", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "Dual.Relaxation.TimeLimit", 30.0, "Time limit (s) when solving LP problems initially", 0, SHOT_DBL_MAX);

    // Dual strategy settings: Main tree strategy

    env->settings->createSettingGroup("Dual", "TreeStrategy", "Tree strategy",
        "The single-tree strategy is normally more efficient than the multi-tree one. However, not all MIP solvers "
        "support the required lazy constraint callbacks. These settings selects this strategy and controls its "
        "behaviour.");

    VectorString enumSolutionStrategy;
    enumSolutionStrategy.push_back("Multi-tree");
    enumSolutionStrategy.push_back("Single-tree");
    env->settings->createSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree),
        "The main strategy to use", enumSolutionStrategy, 0);
    enumSolutionStrategy.clear();

    env->settings->createSetting("Dual.TreeStrategy.Multi.Reinitialize", false,
        "Reinitialize the dual model in the subsolver each iteration", true);

    // Optimization model settings

    env->settings->createSettingGroup("Model", "", "Optimization model",
        "These settings control various aspects of SHOT's representation for and handling of the provided "
        "optimization model.");

    // Bound tightening

    env->settings->createSettingGroup("Model", "BoundTightening", "Bound tightening",
        "SHOT performs bound tightening to strengthen the internal representation of the problem. These settings "
        "control how and when bound tightening is performed.");

    // Bound tightening: feasibility based

    env->settings->createSetting(
        "Model.BoundTightening.FeasibilityBased.MaxIterations", 5, "Maximal number of bound tightening iterations");

    env->settings->createSetting(
        "Model.BoundTightening.FeasibilityBased.TimeLimit", 2.0, "Time limit for bound tightening", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "Model.BoundTightening.FeasibilityBased.Use", true, "Perform feasibility-based bound tightening");

    env->settings->createSetting("Model.BoundTightening.FeasibilityBased.UseNonlinear", true,
        "Perform feasibility-based bound tightening on nonlinear expressions");

    // Bound tightening: initial POA

    // The bounds are those of the Termination settings of the outer approximation solver the values are copied into
    env->settings->createSetting("Model.BoundTightening.InitialPOA.ConstraintTolerance", 1e-1,
        "Constraint termination tolerance", 0, SHOT_DBL_MAX);

    VectorString enumCutStrategy;
    enumCutStrategy.push_back("ESH");
    enumCutStrategy.push_back("ECP");
    env->settings->createSetting("Model.BoundTightening.InitialPOA.CutStrategy",
        static_cast<int>(ES_HyperplaneCutStrategy::ECP), "Dual cut strategy", enumCutStrategy, 0);
    enumCutStrategy.clear();

    env->settings->createSetting("Model.BoundTightening.InitialPOA.DirectionalSolves", 10,
        "Maximum number of additional solves minimizing or maximizing a variable in a convex constraint without "
        "cuts",
        0, SHOT_INT_MAX);

    env->settings->createSetting(
        "Model.BoundTightening.InitialPOA.IterationLimit", 50, "Iteration limit for POA", 1, SHOT_INT_MAX);

    env->settings->createSetting("Model.BoundTightening.InitialPOA.ObjectiveConstraintTolerance", 1e-3,
        "Objective constraint termination tolerance", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Model.BoundTightening.InitialPOA.ObjectiveGapAbsolute", 1e-1,
        "Absolute objective gap termination level", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Model.BoundTightening.InitialPOA.ObjectiveGapRelative", 1e-1,
        "Relative objective gap termination level", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Model.BoundTightening.InitialPOA.StagnationConstraintTolerance", 1e-2,
        "Tolerance factor for when no progress is made", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Model.BoundTightening.InitialPOA.StagnationIterationLimit", 5,
        "Limit for iterations without significant progress", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "Model.BoundTightening.InitialPOA.Use", false, "Create an initial polyhedral outer approximation");

    env->settings->createSetting("Model.BoundTightening.InitialPOA.TimeLimit", 5.0,
        "Time limit for all the problems solved for the initial POA", 0, SHOT_DBL_MAX);

    // Convexity settings

    env->settings->createSettingGroup(
        "Model", "Convexity", "Convexity", "These settings control the convexity detection functionality");

    env->settings->createSetting("Model.Convexity.AssumeConvex", false, "Assume that the problem is convex");

    env->settings->createSetting("Model.Convexity.Quadratics.EigenValueTolerance", 1e-5,
        "Convexity tolerance for the eigenvalues of the Hessian matrix for quadratic terms", 0.0, SHOT_DBL_MAX);

    // Variable settings

    env->settings->createSettingGroup("Model", "Variables", "Variables",
        "These settings control the maximum variable bounds allowed in SHOT. Projection will be performed onto these "
        "intervals. Note that the MIP solvers may have stricter requirements, in which case those may be used.");

    // The variable bounds are projected onto these limits, so a positive minimum lower bound or a negative maximum
    // upper bound would cut off feasible points
    env->settings->createSetting("Model.Variables.Continuous.MinimumLowerBound", -1e50,
        "Minimum lower bound for continuous variables", SHOT_DBL_MIN, 0.0);

    env->settings->createSetting("Model.Variables.Continuous.MaximumUpperBound", 1e50,
        "Maximum upper bound for continuous variables", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Model.Variables.Integer.MinimumLowerBound", -2.0e9,
        "Minimum lower bound for integer variables", SHOT_DBL_MIN, 0.0);

    env->settings->createSetting("Model.Variables.Integer.MaximumUpperBound", 2.0e9,
        "Maximum upper bound for integer variables", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Model.Variables.NonlinearObjectiveVariable.Bound", 1e12,
        "Absolute bound for the auxiliary nonlinear objective variable, used when the bounds of the objective function "
        "cannot be calculated with interval arithmetic",
        0.0, SHOT_DBL_MAX);

    // Reformulation settings

    env->settings->createSettingGroup("Model", "Reformulation", "Automatic reformulations",
        "These settings control the automatic reformulations performed in SHOT.");

    // Reformulations for bilinears
    env->settings->createSetting("Model.Reformulation.Bilinear.AddConvexEnvelope", false,
        "Add convex envelopes (subject to original bounds) to bilinear terms");

    // Reformulations for integer bilinears
    VectorString enumBilinearIntegerReformulation;
    enumBilinearIntegerReformulation.push_back("No");
    enumBilinearIntegerReformulation.push_back("No if nonconvex quadratic terms allowed by MIP solver");
    enumBilinearIntegerReformulation.push_back("Yes");
    env->settings->createSetting("Model.Reformulation.Bilinear.IntegerFormulation",
        static_cast<int>(ES_ReformulateBilinearInteger::NoIfQuadraticSupport), "Reformulate integer bilinear terms",
        enumBilinearIntegerReformulation, 0);
    enumBilinearIntegerReformulation.clear();

    env->settings->createSetting("Model.Reformulation.Bilinear.IntegerFormulation.MaxDomain", 100,
        "Do not reformulate integer variables in bilinear terms which can assume more than this number of discrete "
        "values",
        2, SHOT_INT_MAX);

    // Reformulation of constraints c - a*x1*x2 <= 0 with positive variables to the convex c/(a*x1) - x2 <= 0
    VectorString enumBilinearInverseReformulation;
    enumBilinearInverseReformulation.push_back("None");
    enumBilinearInverseReformulation.push_back("One constraint");
    enumBilinearInverseReformulation.push_back("Both constraints");
    env->settings->createSetting("Model.Reformulation.Bilinear.InverseFormulation",
        static_cast<int>(ES_ReformulateBilinearInverse::BothConstraints),
        "Reformulate constraints a*x1*x2 >= c with positive variables as c/(a*x1) - x2 <= 0 (and c/(a*x2) - x1 <= 0)",
        enumBilinearInverseReformulation, 0);
    enumBilinearInverseReformulation.clear();

    // Reformulations for constraints
    VectorString enumNonlinearTermPartitioning;
    enumNonlinearTermPartitioning.push_back("Always");
    enumNonlinearTermPartitioning.push_back("If result is convex");
    enumNonlinearTermPartitioning.push_back("Never");
    env->settings->createSetting("Model.Reformulation.Constraint.PartitionNonlinearTerms",
        static_cast<int>(ES_PartitionNonlinearSums::IfConvex), "When to partition nonlinear sums in constraints",
        enumNonlinearTermPartitioning, 0);

    env->settings->createSetting("Model.Reformulation.Constraint.PartitionQuadraticTerms",
        static_cast<int>(ES_PartitionNonlinearSums::IfConvex),
        "When to partition quadratic sums in constraints, also in nonlinear constraints", enumNonlinearTermPartitioning,
        0);

    // Reformulations for monomials

    env->settings->createSetting("Model.Reformulation.AbsoluteValue.MaximumBigM", 1e8,
        "The largest big-M used to bound an absolute value from above with a binary variable; above it, the absolute "
        "value is only bounded from below",
        0.0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "Model.Reformulation.Monomials.Extract", true, "Extract monomial terms from nonlinear expressions");

    VectorString enumBinaryMonomialReformulation;
    enumBinaryMonomialReformulation.push_back("None");
    enumBinaryMonomialReformulation.push_back("Simple");
    enumBinaryMonomialReformulation.push_back("Costa and Liberti");
    env->settings->createSetting("Model.Reformulation.Monomials.Formulation",
        static_cast<int>(ES_ReformulationBinaryMonomials::Simple), "How to reformulate binary monomials",
        enumBinaryMonomialReformulation, 0);
    enumBinaryMonomialReformulation.clear();

    // Reformulations for objective functions
    VectorString enumObjectiveEpigraphStrategy;
    enumObjectiveEpigraphStrategy.push_back("Unchanged");
    enumObjectiveEpigraphStrategy.push_back("Objective function");
    enumObjectiveEpigraphStrategy.push_back("Epigraph constraint");
    env->settings->createSetting("Model.Reformulation.ObjectiveFunction.EpigraphStrategy",
        static_cast<int>(ES_ObjectiveEpigraphStrategy::Unchanged),
        "How to choose between objective-function and epigraph-constraint form for the objective. 'Unchanged' "
        "performs no epigraph or anti-epigraph reformulation, keeping the objective function exactly as given; "
        "'Objective function' always keeps (or folds back to) a direct objective function; 'Epigraph constraint' "
        "always introduces an epigraph auxiliary-variable constraint for nonlinear or quadratic objectives",
        enumObjectiveEpigraphStrategy, 0);
    enumObjectiveEpigraphStrategy.clear();

    env->settings->createSetting("Model.Reformulation.ObjectiveFunction.PartitionNonlinearTerms",
        static_cast<int>(ES_PartitionNonlinearSums::IfConvex), "When to partition nonlinear sums in objective function",
        enumNonlinearTermPartitioning, 0);

    env->settings->createSetting("Model.Reformulation.ObjectiveFunction.PartitionQuadraticTerms",
        static_cast<int>(ES_PartitionNonlinearSums::IfConvex),
        "When to partition quadratic sums in the objective function, also in a nonlinear objective function",
        enumNonlinearTermPartitioning, 0);

    enumNonlinearTermPartitioning.clear();

    // Reformulations for signomials

    env->settings->createSetting(
        "Model.Reformulation.Signomials.Extract", true, "Extract signomial terms from nonlinear expressions");

    // Reformulations for quadratic objective and constraints

    VectorString enumQuadExtractStrategy;
    enumQuadExtractStrategy.push_back("Do not extract");
    enumQuadExtractStrategy.push_back("Extract to same objective or constraint");
    enumQuadExtractStrategy.push_back("Extract to quadratic equality constraint if nonconvex");
    enumQuadExtractStrategy.push_back("Extract to quadratic equality constraint even if convex");

    env->settings->createSetting("Model.Reformulation.Quadratics.ExtractStrategy",
        static_cast<int>(ES_QuadraticTermsExtractStrategy::ExtractTermsToSame),
        "How to extract quadratic terms from nonlinear expressions", enumQuadExtractStrategy, 0);
    enumQuadExtractStrategy.clear();

    VectorString enumQPStrategy;
    enumQPStrategy.push_back("All nonlinear");
    enumQPStrategy.push_back("Use quadratic objective");
    enumQPStrategy.push_back("Use convex quadratic objective and constraints");
    enumQPStrategy.push_back("Use nonconvex quadratic objective and constraints");

    env->settings->createSetting("Model.Reformulation.Quadratics.Strategy",
        static_cast<int>(ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained),
        "How to treat quadratic functions", enumQPStrategy, 0);
    enumQPStrategy.clear();

    VectorString enumQPDecomposition;
    enumQPDecomposition.push_back("No decomposition");
    enumQPDecomposition.push_back("Eigenvalue decomposition");
    enumQPDecomposition.push_back("LDL decomposition");

    env->settings->createSetting("Model.Reformulation.Quadratics.Decomposition.Method",
        static_cast<int>(ES_QuadraticDecomposition::LDLDecomposition), "How to decompose convex quadratic functions",
        enumQPDecomposition, 0);
    enumQPDecomposition.clear();

    VectorString enumQuadraticDecompCoeffStrategy;
    enumQuadraticDecompCoeffStrategy.push_back("Coefficient is included in reformulation");
    enumQuadraticDecompCoeffStrategy.push_back("Coefficient remains");

    env->settings->createSetting("Model.Reformulation.Quadratics.Decomposition.Formulation",
        static_cast<int>(ES_QuadraticDecompositionFormulation::CoefficientReformulated),
        "Placement of the original term coefficient when decomposing a quadratic term",
        enumQuadraticDecompCoeffStrategy, 0);
    enumQuadraticDecompCoeffStrategy.clear();

    env->settings->createSetting("Model.Reformulation.Quadratics.Decomposition.Tolerance", 1e-7,
        "Terms with corresponding eigenvalues or diagonal elements in D-matrix smaller than this value will be ignored",
        0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Model.Reformulation.Quadratics.Decomposition.LDLMaximumFill", 100.0,
        "Skip the LDL decomposition if L would have more than this times the nonzeros of the quadratic matrix, and "
        "over "
        "a million (0 = no limit)",
        0.0, SHOT_DBL_MAX);

    // Modeling system settings

    env->settings->createSettingGroup("ModelingSystem", "", "Modeling system",
        "These settings control functionality used in the interfaces to different modeling environments.");

    // Logging and output settings

    env->settings->createSettingGroup("Output", "", "Solver output",
        "These settings control how much and what output is shown to the user from the solver.");

    env->settings->createSetting("Output.Console.DualSolver.Show", false, "Show output from dual solver on console");

    VectorString enumIterationDetail;
    enumIterationDetail.push_back("Full");
    enumIterationDetail.push_back("On objective gap update");
    enumIterationDetail.push_back("On objective gap update and all primal NLP calls");
    env->settings->createSetting("Output.Console.Iteration.Detail",
        static_cast<int>(ES_IterationOutputDetail::ObjectiveGapUpdates),
        "Which iterations are shown on the console: all, or only those that update the objective gap (and those with "
        "a primal NLP call). Some iterations are shown regardless after 100 hidden ones or 2 s",
        enumIterationDetail, 0);
    enumIterationDetail.clear();

    VectorString enumLogLevel;
    enumLogLevel.push_back("Trace");
    enumLogLevel.push_back("Debug");
    enumLogLevel.push_back("Info");
    enumLogLevel.push_back("Warning");
    enumLogLevel.push_back("Error");
    enumLogLevel.push_back("Critical");
    enumLogLevel.push_back("Off");
    env->settings->createSetting(
        "Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info), "Log level for console output", enumLogLevel, 0);

    env->settings->createSetting(
        "Output.Console.PrimalSolver.Show", false, "Show output from primal solver on console");

    env->settings->createSetting("Output.Debug.Enable", false, "Use debug functionality");

    env->settings->createSetting("Output.Debug.Path", empty, "The folder where to save the debug information", false);

    env->settings->createSetting(
        "Output.File.LogLevel", static_cast<int>(E_LogLevel::Info), "Log level for file output", enumLogLevel, 0);
    enumLogLevel.clear();

    env->settings->createSetting("Output.GAMS.AlternateSolutionsFile", std::string(),
        "Name of GAMS GDX file to write alternative solutions to", false);

    VectorString enumOutputDirectory;
    enumOutputDirectory.push_back("Problem directory");
    enumOutputDirectory.push_back("Program directory");
    env->settings->createSetting("Output.OutputDirectory", static_cast<int>(ES_OutputDirectory::Program),
        "Where to save the output files", enumOutputDirectory, 0);
    enumOutputDirectory.clear();

    env->settings->createSetting("Output.SaveNumberOfSolutions", 1,
        "Max number of primal solutions to keep and to save to OSrL or GDX files", 1, SHOT_INT_MAX);

    env->settings->createSettingGroup(
        "Primal", "", "Primal heuristics", "These settings control the primal heuristics used in SHOT.");

    env->settings->createSettingGroup("Primal", "FixedInteger", "Fixed-integer (NLP) strategy",
        "The main primal strategy in SHOT is to solve integer-fixed NLP problems. These settings control, e.g., how "
        "often NLP problems are solved.");

    // Primal settings: Fixed integer strategy
    VectorString enumPrimalNLPStrategy;
    enumPrimalNLPStrategy.push_back("Use each iteration");
    enumPrimalNLPStrategy.push_back("Based on iteration or time");
    enumPrimalNLPStrategy.push_back(
        "Based on iteration or time, and whenever a MIP solution fulfils the nonlinear constraints");

    env->settings->createSetting("Primal.FixedInteger.CallStrategy",
        static_cast<int>(ES_PrimalNLPStrategy::IterationOrTimeAndAllFeasibleSolutions),
        "When should the fixed strategy be used", enumPrimalNLPStrategy, 0);
    enumPrimalNLPStrategy.clear();

    env->settings->createSetting("Primal.FixedInteger.CreateInfeasibilityCut", false,
        "Create cutting planes in the solution point of the fixed-integer NLP problem for the nonlinear "
        "constraints it violates");

    env->settings->createSetting("Primal.FixedInteger.CreateSupportingHyperplanes", true,
        "Create supporting hyperplanes in a feasible solution of the fixed-integer NLP problem for the convex "
        "nonlinear "
        "objective function and the nonlinear constraints active in it.");

    env->settings->createSetting("Primal.FixedInteger.SupportingHyperplanes.ActiveTolerance", 1e-5,
        "A constraint with a value within this tolerance of its bound is active in the fixed-integer NLP solution", 0.0,
        SHOT_DBL_MAX);

    env->settings->createSetting("Primal.FixedInteger.SupportingHyperplanes.MaxPerSolution", 50,
        "Max number of supporting hyperplanes in each fixed-integer NLP solution", 0, SHOT_INT_MAX);

    env->settings->createSetting("Primal.FixedInteger.SupportingHyperplanes.Nonconvex", false,
        "Also create the supporting hyperplanes for nonconvex constraints and objective functions. As the hyperplanes "
        "of the dual strategy for these, they are not global and can be relaxed if they cut away a primal solution");

    env->settings->createSetting("Primal.FixedInteger.SupportingHyperplanes.OnlyViolated", false,
        "Only create the supporting hyperplanes that cut away the dual solution the integer assignment comes from");

    env->settings->createSetting("Primal.FixedInteger.Enumeration.MaxCombinations", 50,
        "Max number of combinations of the discrete variables for the exhaustive search to be performed", 1,
        SHOT_INT_MAX);

    env->settings->createSetting("Primal.FixedInteger.Enumeration.TimeLimit", 20.0,
        "Time limit (s) for all NLP problems in the exhaustive search", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Primal.FixedInteger.Enumeration.UseInitially", false,
        "Solve NLP problems for all combinations of the discrete variables before the dual strategy");

    env->settings->createSetting("Primal.FixedInteger.Enumeration.UseAsFallback", true,
        "Solve NLP problems for all combinations of the discrete variables if the objective gap could not be closed");

    env->settings->createSetting(
        "Primal.FixedInteger.Frequency.Dynamic", true, "Dynamically update the call frequency based on success");

    env->settings->createSetting(
        "Primal.FixedInteger.Frequency.Iteration", 10, "Max number of iterations between calls", 0, SHOT_INT_MAX);

    env->settings->createSetting(
        "Primal.FixedInteger.Frequency.Time", 5.0, "Max duration (s) between calls", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Primal.FixedInteger.DualPointGap.Relative", 0.001,
        "If the objective gap between the MIP point and dual solution is less than this the fixed strategy is "
        "activated",
        0, SHOT_DBL_MAX);

    env->settings->createSetting("Primal.FixedInteger.IterationLimit", 0,
        "Max number of iterations per call of the NLP solver: 0: The default of the NLP solver", 0, SHOT_INT_MAX);

    env->settings->createSetting("Primal.FixedInteger.OnlyUniqueIntegerCombinations", true,
        "Solve the NLP problem only once for each combination of the discrete variables. If false, it is "
        "solved again from different continuous values, e.g. for nonconvex problems");

    VectorString enumPrimalNLPSolver;
    enumPrimalNLPSolver.push_back("Ipopt");
    enumPrimalNLPSolver.push_back("GAMS");
    enumPrimalNLPSolver.push_back("SHOT");
    enumPrimalNLPSolver.push_back("Uno");
    enumPrimalNLPSolver.push_back("Conopt");

    env->settings->createSetting("Primal.FixedInteger.Solver", static_cast<int>(ES_PrimalNLPSolver::Ipopt),
        "NLP solver to use", enumPrimalNLPSolver, 0);
    enumPrimalNLPSolver.clear();

    VectorString enumPrimalBoundNLPStartingPoint;
    enumPrimalBoundNLPStartingPoint.push_back("All");
    enumPrimalBoundNLPStartingPoint.push_back("First");
    enumPrimalBoundNLPStartingPoint.push_back("All feasible");
    enumPrimalBoundNLPStartingPoint.push_back("First and the one with the smallest constraint deviation");
    enumPrimalBoundNLPStartingPoint.push_back("With smallest constraint deviation");
    env->settings->createSetting("Primal.FixedInteger.Source",
        static_cast<int>(ES_PrimalNLPFixedPoint::FirstAndFeasibleSolutions),
        "Which solutions of the dual problem the fixed-integer NLP problem is solved from. In single-tree, the "
        "callbacks use each new incumbent, and this applies to the solution pool after the search",
        enumPrimalBoundNLPStartingPoint, 0);
    enumPrimalBoundNLPStartingPoint.clear();

    VectorString enumPrimalBoundNLPProblemSource;
    enumPrimalBoundNLPProblemSource.push_back("Original problem");
    enumPrimalBoundNLPProblemSource.push_back("Reformulated problem");
    enumPrimalBoundNLPProblemSource.push_back("Both");
    // The reformulated problem is the default, since the NLP solver then gets a nonlinear equality constraint as two
    // inequality constraints. With the equality constraints of the original problem, Ipopt found clearly worse local
    // solutions for, e.g., pooling problems. NLP solvers in GAMS always use the original problem.
    env->settings->createSetting("Primal.FixedInteger.SourceProblem",
        static_cast<int>(ES_PrimalNLPProblemSource::ReformulatedProblem),
        "Which problem formulation to use for NLP problem", enumPrimalBoundNLPProblemSource, 0);
    enumPrimalBoundNLPProblemSource.clear();

    env->settings->createSetting(
        "Primal.FixedInteger.TimeLimit", 10.0, "Time limit (s) per NLP problem", 0, SHOT_DBL_MAX);

    // Independent of Dual.MIP.NumberOfThreads, so that NLP problems can be solved at the same time as the MIP problem.
    // The BLAS library, e.g., OpenBLAS used by MUMPS in Ipopt, otherwise uses all cores.
    env->settings->createSetting("Primal.FixedInteger.NumberOfThreads", 1,
        "Number of threads for each NLP problem, in the NLP solver and its BLAS and OpenMP libraries: 0: Automatic", 0,
        SHOT_INT_MAX);

    env->settings->createSetting("Primal.FixedInteger.UseStartingPoint", false,
        "Solve an NLP problem from a given starting point, e.g. the levels of the variables in GAMS or the initial "
        "values in AMPL, before the first dual problem, and from points given by the PrimalCandidateSearch callback "
        "at the next NLP call. Off by default until benchmarked on all of MINLPLib");

    env->settings->createSetting("Primal.FixedInteger.Use", true, "Use the fixed integer primal strategy");

    env->settings->createSetting("Primal.FixedInteger.Warmstart", true, "Warm start the NLP solver");

    env->settings->createSetting("Primal.PolishSolution.NumberOfPoints", 1,
        "After the search, polish the solution with NLP problems started from the best primal solution and from the "
        "dual solutions of this many last iterations, not for LP, MILP or convex continuous problems: 0: Off",
        0, SHOT_INT_MAX);

    // Primal settings: rootsearch

    env->settings->createSettingGroup("Primal", "Rootsearch", "Primal root search",
        "SHOT can utilize root searches between the dual solution point and an integer-fixed interior point. This "
        "setting controls whether this strategy is used.");

    env->settings->createSetting("Primal.Rootsearch.Use", true, "Use a rootsearch to find primal solutions");

    // Primal settings: tolerances for accepting primal solutions

    env->settings->createSettingGroup("Primal", "Tolerance", "Primal solution tolerances",
        "These settings sets various tolerances for accepting primal solutions.");

    env->settings->createSetting("Primal.Tolerance.TrustLinearConstraintValues", true,
        "Trust that subsolvers (NLP, MIP) give primal solutions that respect linear constraints");

    // The integer and linear constraint tolerances are also used as MIP solver tolerances, and the MIP solver
    // interfaces move them into the range each solver accepts. With an integer tolerance of 0.5, any value is integral.
    env->settings->createSetting(
        "Primal.Tolerance.Integer", 1e-5, "Integer tolerance for accepting primal solutions", 0.0, 0.5);

    env->settings->createSetting("Primal.Tolerance.LinearConstraint", 1e-6,
        "Linear constraint tolerance for accepting primal solutions", 0.0, SHOT_DBL_MAX);

    env->settings->createSetting("Primal.Tolerance.NonlinearConstraint", 1e-5,
        "Nonlinear constraint tolerance for accepting primal solutions", 0.0, SHOT_DBL_MAX);

    // Strategy settings

    env->settings->createSettingGroup("Strategy", "", "Strategy", "Overall strategy parameters used in SHOT.");

    env->settings->createSetting("Strategy.UseRecommendedSettings", true,
        "Modifies some settings to their recommended values based on the strategy");

    // Subsolver settings: Cplex

    env->settings->createSettingGroup("Subsolver", "", "Subsolver functionality",
        "These settings allow for more direct control of the  different subsolvers utilized in SHOT.");

#ifdef HAS_CPLEX

    env->settings->createSettingGroup("Subsolver", "Cplex", "Cplex", "");

    env->settings->createSetting("Subsolver.Cplex.AddRelaxedLazyConstraintsAsLocal", false,
        "Whether to add lazy constraints generated in relaxed points as local or global");

    VectorString enumCplexOptimalityTarget;
    enumCplexOptimalityTarget.push_back("Automatic");
    enumCplexOptimalityTarget.push_back("Searches for a globally optimal solution to a convex model");
    enumCplexOptimalityTarget.push_back("Searches for a solution that satisfies first-order optimality conditions, but "
                                        "is not necessarily globally optimal");
    enumCplexOptimalityTarget.push_back("Searches for a globally "
                                        "optimal solution to a nonconvex model");
    env->settings->createSetting("Subsolver.Cplex.OptimalityTarget", 0,
        "Specifies how CPLEX treats nonconvex quadratics", enumCplexOptimalityTarget, 0);
    enumCplexOptimalityTarget.clear();

    VectorString enumCplexFeasOptMode;
    enumCplexFeasOptMode.push_back("Minimize the sum of all required relaxations in first phase only");
    enumCplexFeasOptMode.push_back("Minimize the sum of all required relaxations in first phase and execute second "
                                   "phase to find optimum among minimal relaxations");
    enumCplexFeasOptMode.push_back("Minimize the number of constraints and bounds requiring relaxation in first "
                                   "phase only");
    enumCplexFeasOptMode.push_back("Minimize the number of constraints and bounds requiring relaxation in first "
                                   "phase and execute second phase to find optimum among minimal relaxations");
    enumCplexFeasOptMode.push_back("Minimize the sum of squares of required "
                                   "relaxations in first phase only");
    enumCplexFeasOptMode.push_back("Minimize the sum of squares of required relaxations in first phase and execute "
                                   "second phase to find optimum among minimal relaxations");
    env->settings->createSetting(
        "Subsolver.Cplex.FeasOptMode", 0, "Strategy to use for the feasibility repair", enumCplexFeasOptMode, 0);
    enumCplexFeasOptMode.clear();

    VectorString enumCplexMIPEmphasis;
    enumCplexMIPEmphasis.push_back("Balanced");
    enumCplexMIPEmphasis.push_back("Feasibility");
    enumCplexMIPEmphasis.push_back("Optimality");
    enumCplexMIPEmphasis.push_back("Best bound");
    enumCplexMIPEmphasis.push_back("Hidden feasible");
    env->settings->createSetting("Subsolver.Cplex.MIPEmphasis", 0, "Sets the MIP emphasis", enumCplexMIPEmphasis, 0);
    enumCplexMIPEmphasis.clear();

    env->settings->createSetting("Subsolver.Cplex.MemoryEmphasis", 0, "Try to conserve memory when possible", 0, 1);

    VectorString enumCplexNodeFile;
    enumCplexNodeFile.push_back("No file");
    enumCplexNodeFile.push_back("Compressed in memory");
    enumCplexNodeFile.push_back("On disk");
    enumCplexNodeFile.push_back("Compressed on disk");
    env->settings->createSetting("Subsolver.Cplex.NodeFile", 1, "Where to store the node file", enumCplexNodeFile, 0);
    enumCplexNodeFile.clear();

    env->settings->createSetting("Subsolver.Cplex.NumericalEmphasis", 0, "Emphasis on numerical stability", 0, 1);

    VectorString enumCplexParallelMode;
    enumCplexParallelMode.push_back("Opportunistic");
    enumCplexParallelMode.push_back("Automatic");
    enumCplexParallelMode.push_back("Deterministic");
    env->settings->createSetting("Subsolver.Cplex.ParallelMode", 0,
        "Parallel mode in CPLEX, i.e., whether it runs deterministically when using several threads",
        enumCplexParallelMode, -1);
    enumCplexParallelMode.clear();

    VectorString enumCplexProbe;
    enumCplexProbe.push_back("No probing");
    enumCplexProbe.push_back("Automatic");
    enumCplexProbe.push_back("Moderate");
    enumCplexProbe.push_back("Aggressive");
    enumCplexProbe.push_back("Very aggressive");
    env->settings->createSetting("Subsolver.Cplex.Probe", 0, "Sets the MIP probing level", enumCplexProbe, -1);
    enumCplexProbe.clear();

    env->settings->createSetting("Subsolver.Cplex.SolutionPoolGap", 1.0e+75,
        "Sets the relative gap filter on objective values in the solution pool", 0, 1.0e+75);

    VectorString enumCplexSolPoolIntensity;
    enumCplexSolPoolIntensity.push_back("Automatic");
    enumCplexSolPoolIntensity.push_back("Mild");
    enumCplexSolPoolIntensity.push_back("Moderate");
    enumCplexSolPoolIntensity.push_back("Aggressive");
    enumCplexSolPoolIntensity.push_back("Very aggressive");
    env->settings->createSetting("Subsolver.Cplex.SolutionPoolIntensity", 0,
        "Controls how much time and memory should be used when filling the solution pool", enumCplexSolPoolIntensity,
        0);
    enumCplexSolPoolIntensity.clear();

    VectorString enumCplexSolPoolReplace;
    enumCplexSolPoolReplace.push_back("Replace oldest");
    enumCplexSolPoolReplace.push_back("Replace worst");
    enumCplexSolPoolReplace.push_back("Find diverse");
    env->settings->createSetting("Subsolver.Cplex.SolutionPoolReplace", 0,
        "How to replace solutions in the solution pool when full", enumCplexSolPoolReplace, 0);
    enumCplexSolPoolReplace.clear();

    std::string workdir = "";
    env->settings->createSetting("Subsolver.Cplex.WorkDirectory", workdir, "Directory for swap file");

    env->settings->createSetting(
        "Subsolver.Cplex.WorkMemory", 0.0, "Memory limit for when to start swapping to disk", 0.0, 1.0e+75);

#endif

    // Subsolver settings: Gurobi

#ifdef HAS_GUROBI

    env->settings->createSettingGroup("Subsolver", "Gurobi", "Gurobi", "");

    env->settings->createSetting(
        "Subsolver.Gurobi.Heuristics", 0.05, "The relative amount of time spent in MIP heuristics.", 0.0, 1.0);

    VectorString enumGurobiMIPFocus;
    enumGurobiMIPFocus.push_back("Automatic");
    enumGurobiMIPFocus.push_back("Feasibility");
    enumGurobiMIPFocus.push_back("Optimality");
    enumGurobiMIPFocus.push_back("Best bound");
    env->settings->createSetting("Subsolver.Gurobi.MIPFocus", 0, "MIP focus", enumGurobiMIPFocus, 0);
    enumGurobiMIPFocus.clear();

    VectorString enumGurobiNumericFocus;
    enumGurobiNumericFocus.push_back("Automatic");
    enumGurobiNumericFocus.push_back("Mild");
    enumGurobiNumericFocus.push_back("Moderate");
    enumGurobiNumericFocus.push_back("Aggressive");
    env->settings->createSetting("Subsolver.Gurobi.NumericFocus", 0,
        "How much effort Gurobi spends on detecting and managing numerical issues", enumGurobiNumericFocus, 0);
    enumGurobiNumericFocus.clear();

    VectorString enumGurobiPoolSearchMode;
    enumGurobiPoolSearchMode.push_back("No extra effort");
    enumGurobiPoolSearchMode.push_back("Try to find solutions");
    enumGurobiPoolSearchMode.push_back("Find n best solutions");
    env->settings->createSetting(
        "Subsolver.Gurobi.PoolSearchMode", 0, "Finds extra solutions", enumGurobiPoolSearchMode, 0);
    enumGurobiPoolSearchMode.clear();

    VectorString enumGurobiScaleFlag;
    enumGurobiScaleFlag.push_back("Automatic");
    enumGurobiScaleFlag.push_back("Off");
    enumGurobiScaleFlag.push_back("Mild");
    enumGurobiScaleFlag.push_back("Moderate");
    enumGurobiScaleFlag.push_back("Aggressive");
    env->settings->createSetting("Subsolver.Gurobi.ScaleFlag", -1, "Controls model scaling", enumGurobiScaleFlag, -1);
    enumGurobiScaleFlag.clear();

#endif

    // Subsolver settings: Cbc

#ifdef HAS_CBC

    env->settings->createSettingGroup("Subsolver", "Cbc", "Cbc", "");

    env->settings->createSetting("Subsolver.Cbc.AutoScale", false,
        "Whether to scale objective, rhs and bounds of problem if they look odd (experimental)");

    env->settings->createSetting("Subsolver.Cbc.Cuts", false,
        "Use the cut generators of Cbc, which can give a wrong optimal value for a MILP problem");

    VectorString enumCbcNodeStrategy;
    enumCbcNodeStrategy.push_back("depth");
    enumCbcNodeStrategy.push_back("downdepth");
    enumCbcNodeStrategy.push_back("downfewest");
    enumCbcNodeStrategy.push_back("fewest");
    enumCbcNodeStrategy.push_back("hybrid");
    enumCbcNodeStrategy.push_back("updepth");
    enumCbcNodeStrategy.push_back("upfewest");
    env->settings->createSetting("Subsolver.Cbc.NodeStrategy", 4, "Node strategy", enumCbcNodeStrategy, 0);
    enumCbcNodeStrategy.clear();

    env->settings->createSetting(
        "Subsolver.Cbc.DeterministicParallelMode", false, "Run Cbc with multiple threads in deterministic mode");

    VectorString enumCbcScaling;
    enumCbcScaling.push_back("automatic");
    enumCbcScaling.push_back("dynamic");
    enumCbcScaling.push_back("equilibrium");
    enumCbcScaling.push_back("geometric");
    enumCbcScaling.push_back("off");
    enumCbcScaling.push_back("rowsonly");
    env->settings->createSetting("Subsolver.Cbc.Scaling", 0, "Whether to scale problem", enumCbcScaling, 0);
    enumCbcScaling.clear();

    VectorString enumStrategy;
    enumStrategy.push_back("easy problems");
    enumStrategy.push_back("default");
    enumStrategy.push_back("aggressive");
    env->settings->createSetting("Subsolver.Cbc.Strategy", 1, "This turns on newer features", enumStrategy, 0);
    enumStrategy.clear();

#endif

#ifdef HAS_HIGHS

    env->settings->createSettingGroup("Subsolver", "Highs", "Highs", "");

    env->settings->createSetting("Subsolver.Highs.MIPAllowRestart", true, "Whether MIP restart is permitted");

    env->settings->createSetting("Subsolver.Highs.MIPDetectSymmetry", true, "Whether MIP symmetry should be detected");

    env->settings->createSetting("Subsolver.Highs.MIPHeuristicEffort", 0.05, "Effort spent for MIP heuristics", 0, 1);

    env->settings->createSetting("Subsolver.Highs.MIPHeuristicRunZiRound", false, "Run Zi Round heuristic");

    env->settings->createSetting("Subsolver.Highs.MIPHeuristicRunShifting", false, "Run Shifting heuristic");

    VectorString enumHighsRunCrossover;
    enumHighsRunCrossover.push_back("off");
    enumHighsRunCrossover.push_back("choose");
    enumHighsRunCrossover.push_back("on");
    env->settings->createSetting("Subsolver.Highs.RunCrossover", 0, "Run crossover", enumHighsRunCrossover, 0);
    enumHighsRunCrossover.clear();

    VectorString enumHighsDebugLevel;
    enumHighsDebugLevel.push_back("off");
    enumHighsDebugLevel.push_back("low");
    enumHighsDebugLevel.push_back("medium");
    enumHighsDebugLevel.push_back("high");
    env->settings->createSetting(
        "Subsolver.Highs.DebugLevel", 0, "Debug level for HiGHS internal assertions", enumHighsDebugLevel, 0);
    enumHighsDebugLevel.clear();

    VectorString enumHighsMIPIPMSolver;
    enumHighsMIPIPMSolver.push_back("choose");
    enumHighsMIPIPMSolver.push_back("ipx");
    enumHighsMIPIPMSolver.push_back("hipo (only if HiGHS has been built with HiPO)");
    env->settings->createSetting("Subsolver.Highs.MIPIPMSolver", 0,
        "Interior point solver that HiGHS uses in the MIP solver, e.g., for the analytic center", enumHighsMIPIPMSolver,
        0);
    enumHighsMIPIPMSolver.clear();

    VectorString enumHighsParallel;
    enumHighsParallel.push_back("off");
    enumHighsParallel.push_back("choose");
    enumHighsParallel.push_back("on");
    env->settings->createSetting("Subsolver.Highs.Parallel", 1, "Use parallelization", enumHighsParallel, 0);
    enumHighsParallel.clear();

    VectorString enumHighsPresolve;
    enumHighsPresolve.push_back("off");
    enumHighsPresolve.push_back("choose");
    enumHighsPresolve.push_back("on");
    env->settings->createSetting("Subsolver.Highs.Presolve", 1, "Use presolve", enumHighsPresolve, 0);
    enumHighsPresolve.clear();

#endif

    // Subsolver settings: CONOPT

#ifdef HAS_CONOPT
    env->settings->createSettingGroup("Subsolver", "Conopt", "CONOPT settings", "Options for the CONOPT NLP solver");
    env->settings->createSetting(
        "Subsolver.Conopt.FeasibilityTolerance", 1e-9, "CONOPT feasibility tolerance (Rtnwma)", 1e-12, 1e-2);
    env->settings->createSetting(
        "Subsolver.Conopt.OptimalityTolerance", 1e-9, "CONOPT reduced-gradient tolerance (Rtredg)", 1e-12, 1e-2);
    env->settings->createSetting("Subsolver.Conopt.DerivativeCheck", 0,
        "CONOPT compares the first and second derivatives with numerical ones (Lkdebg, Lkdeb2): 0 = never, -1 = at "
        "the initial point, n = every n-th iteration",
        -1, SHOT_INT_MAX);
#endif

    // Subsolver settings: Ipopt
#ifdef HAS_IPOPT

    env->settings->createSettingGroup("Subsolver", "Ipopt", "Ipopt", "");

    env->settings->createSetting("Subsolver.Ipopt.ConstraintViolationTolerance", 1E-8,
        "Constraint violation tolerance in Ipopt", 0.0, SHOT_DBL_MAX);

    VectorString enumIPOptSolver;
    enumIPOptSolver.push_back("Default");
    enumIPOptSolver.push_back("MA27");
    enumIPOptSolver.push_back("MA57");
    enumIPOptSolver.push_back("MA86");
    enumIPOptSolver.push_back("MA97");
    enumIPOptSolver.push_back("MUMPS");
    env->settings->createSetting("Subsolver.Ipopt.LinearSolver", static_cast<int>(ES_IpoptSolver::IpoptDefault),
        "Ipopt linear subsolver", enumIPOptSolver, 0);
    enumIPOptSolver.clear();

    VectorString enumIpoptHessianApproximation;
    enumIpoptHessianApproximation.push_back("Exact");
    enumIpoptHessianApproximation.push_back("Limited memory");
    enumIpoptHessianApproximation.push_back("Automatic");
    env->settings->createSetting("Subsolver.Ipopt.HessianApproximation",
        static_cast<int>(ES_IpoptHessianApproximation::Automatic),
        "The Hessian of the Lagrangian used by Ipopt. Automatic uses a limited-memory approximation if the exact "
        "Hessian of a nonlinear expression with at least 64 variables is nearly dense, since it is then expensive",
        enumIpoptHessianApproximation, 0);
    enumIpoptHessianApproximation.clear();

    env->settings->createSetting("Subsolver.Ipopt.OptionsFile", std::string(),
        "An Ipopt options file with any Ipopt options, read instead of ipopt.opt in the working directory. Its options "
        "override those set by SHOT, e.g. the linear solver, except the time limit (max_wall_time) of each solve",
        false);

    env->settings->createSetting("Subsolver.Ipopt.RelativeConvergenceTolerance", 1E-8,
        "Relative convergence tolerance in Ipopt (tol)", 0.0, SHOT_DBL_MAX);

#endif

    // Subsolver settings: Uno

#ifdef HAS_UNO

    env->settings->createSettingGroup("Subsolver", "Uno", "Uno", "");

    /* These default to Uno's own tolerances rather than to the tighter ones used for Ipopt. Uno reports an
       algorithmic error, rather than the point it reached, when it cannot certify stationarity to the requested
       tolerance, and the accuracy of SHOT's function evaluations does not always allow 1e-8 to be reached. */
    env->settings->createSetting(
        "Subsolver.Uno.ConstraintViolationTolerance", 1E-6, "Constraint violation tolerance in Uno", 0.0, SHOT_DBL_MAX);

    VectorString enumUnoHessianModel;
    enumUnoHessianModel.push_back("Exact");
    enumUnoHessianModel.push_back("L-BFGS");
    enumUnoHessianModel.push_back("L-SR1");
    env->settings->createSetting("Subsolver.Uno.HessianModel", static_cast<int>(ES_UnoHessianModel::exact),
        "How Uno models the Hessian", enumUnoHessianModel, 0);
    enumUnoHessianModel.clear();

    env->settings->createSetting(
        "Subsolver.Uno.OptionsFile", std::string(), "Uno option file to read additional options from");

    VectorString enumUnoPreset;
    enumUnoPreset.push_back("Automatic");
    enumUnoPreset.push_back("Ipopt");
    enumUnoPreset.push_back("filterSQP");
    env->settings->createSetting(
        "Subsolver.Uno.Preset", static_cast<int>(ES_UnoPreset::UnoAuto), "The Uno algorithm to use", enumUnoPreset, 0);
    enumUnoPreset.clear();

    env->settings->createSetting("Subsolver.Uno.RelativeConvergenceTolerance", 1E-6,
        "Dual (stationarity) tolerance in Uno (dual_tolerance)", 0.0, SHOT_DBL_MAX);

#endif

    env->settings->createSettingGroup("Subsolver", "SHOT", "SHOT primal NLP solver", "");

    env->settings->createSetting(
        "Subsolver.SHOT.ReuseHyperplanes.Use", true, "Reuse valid generated hyperplanes in main dual model.");
    env->settings->createSetting(
        "Subsolver.SHOT.ReuseHyperplanes.Fraction", 0.1, "The fraction of generated hyperplanes to reuse.", 0.0, 1.0);

    env->settings->createSetting("Subsolver.SHOT.UseFBBT", true, "Do FBBT on NLP problem.");

    // Subsolver settings: root searches

    env->settings->createSettingGroup(
        "Subsolver", "Rootsearch", "Root search solver", "Settings for the Boost rootsearch functionality.");

    env->settings->createSetting(
        "Subsolver.Rootsearch.MaxIterations", 100, "Maximal root search iterations", 0, SHOT_INT_MAX);

    VectorString enumRootsearchMethod;
    enumRootsearchMethod.push_back("TOMS748");
    enumRootsearchMethod.push_back("Bisection");
    env->settings->createSetting("Subsolver.Rootsearch.Method", static_cast<int>(ES_RootsearchMethod::BoostTOMS748),
        "Root search method to use", enumRootsearchMethod, 0);
    enumRootsearchMethod.clear();

    env->settings->createSetting("Subsolver.Rootsearch.TerminationTolerance", 1e-16,
        "Termination tolerance for the length of the root search interval for lambda in [0, 1], which also stops when "
        "the interval cannot be divided further",
        0.0, SHOT_DBL_MAX);

    // Termination settings

    env->settings->createSettingGroup(
        "Termination", "", "Termination", "These settings control when SHOT will terminate the solution process.");

    env->settings->createSetting("Termination.ConstraintTolerance", 1e-8,
        "Termination tolerance for nonlinear constraints (not used for termination in the multi-tree strategy)", 0,
        SHOT_DBL_MAX);

    env->settings->createSetting("Termination.ObjectiveConstraintTolerance", 1e-8,
        "Termination tolerance for the nonlinear objective constraint", 0, SHOT_DBL_MAX);

    env->settings->createSetting(
        "Termination.IterationLimit", 200000, "Iteration limit for main strategy", 1, SHOT_INT_MAX);

    env->settings->createSetting("Termination.ObjectiveGap.Absolute", 0.001,
        "Absolute gap termination tolerance for objective function", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Termination.ObjectiveGap.Relative", 0.001,
        "Relative gap termination tolerance for objective function", 0, SHOT_DBL_MAX);

    env->settings->createSetting("Termination.DualStagnation.ConstraintTolerance", 1e-6,
        "Min absolute difference between max nonlinear constraint errors in subsequent iterations for termination", 0,
        SHOT_DBL_MAX);

    env->settings->createSetting("Termination.DualStagnation.IterationLimit", 1000,
        "Max number of iterations without significant dual objective value improvement", 0, SHOT_INT_MAX);

    env->settings->createSetting("Termination.PrimalStagnation.IterationLimit", 50,
        "Number of iterations without significant primal improvement after which an objective cut is added "
        "(nonconvex problems with reduction cuts only)",
        0, SHOT_INT_MAX);

    env->settings->createSetting("Termination.TimeLimit", SHOT_DBL_MAX, "Time limit (s) for solver", 0.0, SHOT_DBL_MAX);

    // Hidden settings for problem information

    VectorString enumFileFormat;
    enumFileFormat.push_back("OSiL");
    enumFileFormat.push_back("GAMS");
    enumFileFormat.push_back("NL");
    enumFileFormat.push_back("API");
    env->settings->createSetting("Input.ModelingSystem", static_cast<int>(ES_ModelingSystem::API),
        "The format of the problem file", enumFileFormat, 0, true);
    enumFileFormat.clear();

    env->settings->createSetting("Input.ProblemFile", empty, "The filename of the problem", true);

    env->settings->createSetting("Input.ProblemName", empty, "The name of the problem instance", true);

    env->settings->createSetting("Input.OptionsFile", empty, "The name of the options file used", true);

    env->settings->createSetting("Output.ResultPath", empty, "The path where to save the result information", true);

    ModelingSystemOSiL::augmentSettings(env->settings);

#ifdef HAS_AMPL
    ModelingSystemAMPL::augmentSettings(env->settings);
#endif

#ifdef HAS_GAMS
    ModelingSystemGAMS::augmentSettings(env->settings);
#endif

    env->settings->settingsInitialized = true;

    env->output->outputDebug(" Initialization of settings complete.");
}

} // namespace SHOT
