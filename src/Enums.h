#pragma once

enum class E_DualSolutionSource
{
	LPSolution, 
	MIPSolutionOptimal, 
	MIPSolutionFeasible, 
	ObjectiveConstraint
};

enum class E_HyperplaneSource
{
	MIPOptimalLinesearch,
	MIPSolutionPoolLinesearch,
	LPRelaxedLinesearch,
	MIPOptimalSolutionPoint,
	MIPSolutionPoolSolutionPoint,
	LPRelaxedSolutionPoint,
	LPFixedIntegers,
	PrimalSolutionSearch,
	PrimalSolutionSearchInteriorObjective,
	InteriorPointSearch
};

enum class E_IterationProblemType
{
	MIP, 
	Relaxed
};

enum class E_PrimalNLPSource
{
	FirstSolution, 
	FeasibleSolution, 
	InfeasibleSolution, 
	SmallestDeviationSolution, 
	FirstSolutionNewDualBound
};

enum class E_PrimalSolutionSource
{
	Linesearch,
	LinesearchFixedIntegers,
	NLPFixedIntegers,
	NLPRelaxed,
	MIPSolutionPool,
	ObjectiveConstraint,
	LPFixedIntegers,
	LazyConstraintCallback,
	HeuristicCallback,
	IncumbentCallback
};

enum class E_NLPSolutionStatus
{
	Feasible, 
	Optimal, 
	Infeasible, 
	Unbounded, 
	IterationLimit,
	TimeLimit, 
	Error
};

enum class E_ObjectiveFunctionType
{
	Linear, 
	Quadratic, 
	Nonlinear
};

enum class E_ProblemSolutionStatus
{
	Feasible, 
	Optimal, 
	Infeasible, 
	Unbounded, 
	IterationLimit, 
	TimeLimit, 
	SolutionLimit, 
	Error, 
	CutOff
};

enum class E_TerminationReason
{
	ConstraintTolerance,
	ObjectiveStagnation,
	IterationLimit,
	TimeLimit,
	InfeasibleProblem,
	Error,
	AbsoluteGap,
	RelativeGap,
	InteriorPointError
};

enum class ES_AddPrimalPointAsInteriorPoint
{
	KeepOriginal, 
	KeepBoth, 
	KeepNew, 
	OnlyAverage
};

enum class ES_HyperplaneCutStrategy
{
	ESH, 
	ECP
};

enum class ES_IpoptSolver
{
	ma27, 
	ma57, 
	ma86, 
	ma97, 
	mumps
};

enum class ES_RootsearchConstraintStrategy
{
	AllAsMaxFunct, 
	IndividualConstraints
};

enum class ES_RootsearchMethod
{
	BoostTOMS748, 
	BoostBisection, 
	Bisection
};

enum class ES_MIPSolver
{
	Cplex, 
	Gurobi, 
	Cbc
};

enum class ES_NLPSolver
{
	CuttingPlaneMiniMax, 
	IpoptMinimax, 
	IpoptRelaxed, 
	IpoptMinimaxAndRelaxed
};

enum class ES_PrimalNLPFixedPoint
{
	AllSolutions, 
	FirstSolution, 
	AllFeasibleSolutions, 
	FirstAndFeasibleSolutions, 
	SmallestDeviationSolution
};

enum class ES_PrimalNLPSolver
{
	CuttingPlane, 
	Ipopt, 
	GAMS
};

enum class ES_MIPPresolveStrategy
{
	Never, 
	Once, 
	EveryIteration
};

enum class ES_OutputDirectory
{
	Problem, 
	Program
};

enum class ES_PrimalNLPStrategy
{
	AlwaysUse, 
	IterationOrTime, 
	IterationOrTimeAndAllFeasibleSolutions
};

enum class ES_QuadraticProblemStrategy
{
	Nonlinear, 
	QuadraticObjective, 
	QuadraticallyConstrained
};

enum class ES_SolutionStrategy
{
	MultiTree, 
	SingleTree
};