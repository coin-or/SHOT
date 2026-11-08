# MIP solver capabilities

Each MIP backend defines one `inline static constexpr MIPSolverCapabilities
capabilities` record in its class header. For example:

```cpp
bool supportsSOS1 = SHOT::MIPSolverGurobi::capabilities.SOS1;
bool supportsNonconvexObjective = SHOT::MIPSolverCplex::capabilities.nonconvexQuadraticObjective;
```

Code that selects a backend by enum can include
`MIPSolver/MIPSolverCapabilities.h` without vendor SDK headers:

```cpp
auto capabilities = SHOT::getMIPSolverCapabilities(SHOT::ES_MIPSolver::Gurobi);

if(capabilities && capabilities->convexQuadraticConstraints)
{
    // The compiled SHOT adapter can retain convex quadratic constraints.
}
```

The lookup returns no value for an unavailable backend, including `NotUsed`.
It constructs no solver and needs no solver license. A capability describes
support through SHOT's adapter; it does not enable a feature, override a user's
reformulation settings, or establish that a license is available.

`capabilities.unboundedVariableBound` holds SHOT's finite default for missing
variable bounds, available through the same lookup before a solver instance
exists. Every backend assigns it explicitly: Cbc uses `1e50`; CPLEX, Gurobi,
and HiGHS use `1e20`. The existing instance getter
`getUnboundedVariableBoundValue()` reuses this field for Cbc, CPLEX, and Gurobi.
HiGHS continues to return its native infinity value from the instance getter,
which is separate from SHOT's finite default.

## Python API

SHOTpy exposes the same fields as read-only properties of
`MIPSolverCapabilities`:

```python
import SHOTpy

capabilities = SHOTpy.getMIPSolverCapabilities(SHOTpy.MIPSolver.Gurobi)

if capabilities is not None:
    print(capabilities.SOS1)
    print(capabilities.nonconvexQuadraticConstraints)
    print(capabilities.semicontinuous == SHOTpy.SemiVariableSupport.SignedInterval)
```

The function returns `None` for an unavailable backend or `MIPSolver.NotUsed`.
It constructs no solver and needs no solver license. Assigning or deleting a
property raises `AttributeError`; the records are obtained from the lookup and
cannot be constructed directly in Python. Semi-variable support is exposed as
`SemiVariableSupport.None_`, `NonnegativeInterval`, or `SignedInterval`;
`None_` corresponds to C++'s `E_SemiVariableSupport::None`.

## Quadratic objectives and constraints

There are separate fields for convex/nonconvex quadratic objectives and
constraints. Convex objective support means convex minimization or concave
maximization. Strategy selection consumes the existing model classifications
and checks the complete reformulated model, including combinations of features.

Supported quadratic parts are retained by default also when the same objective
or constraint contains higher-degree terms. For example, Gurobi can keep
nonconvex binary quadratic terms while cubic and quartic binary products are
linearized. Explicit `Model.Reformulation.ObjectiveFunction.PartitionQuadraticTerms`
and `Model.Reformulation.Constraint.PartitionQuadraticTerms` settings still
control partitioning when these mixed functions are reformulated.
An explicit request to reformulate integer bilinear terms also takes precedence
over default native quadratic retention.

CPLEX supports nonconvex quadratic objectives globally with linear constraints,
and convex quadratic constraints with compatible objectives. The combination
of a nonconvex objective and quadratic constraints is not generally supported.
Its `nonconvexObjectiveExcludesQuadraticConstraints` restriction records this
relationship; independent feature flags cannot express it. SHOT retains the
exact objective and handles the unsupported constraints through nonlinear
strategies. Existing binary-product reformulations remain applicable.

Nearly-convex constraints use a separate numerical acceptance check in
reformulation. Gurobi's `PSDTol` is configured using SHOT's eigenvalue tolerance;
there is no additional nearly-convex capability flag. Gurobi 10.0 or newer is
required, so older nonconvex-support version gates are unnecessary.

## Discrete features

`SOS1`, `SOS2`, and `singleTree` are independent flags. Single-tree support
describes the SHOT callback integration; SHOT still uses multi-tree
decomposition for nonconvex models. An SOS-only model with continuous columns
is discrete and must not be sent to the continuous NLP strategy.

`semicontinuous` and `semiinteger` use `E_SemiVariableSupport`:

| Value | Supported nonzero branch, in addition to zero |
| --- | --- |
| `None` | No support for the variable type. |
| `NonnegativeInterval` | `[semiBound, upperBound]` with nonnegative `semiBound`. |
| `SignedInterval` | Also `[lowerBound, semiBound]` with negative `semiBound`. |

Semi-integer values must also be integers. Cbc and Gurobi support signed
intervals through their current adapters. CPLEX and HiGHS currently support
nonnegative intervals. CPLEX's
[`IloSemiContVar`](https://www.ibm.com/docs/en/icos/22.1.2?topic=classes-ilosemicontvar)
requires a nonnegative active lower bound; the previous negative-branch mapping did not produce a supported
model. HiGHS's adapter also lacks the required negative-branch mapping.

Full-model initialization validates these features before preprocessing and
again before constructing the reformulated MIP model. Unsupported features
produce a diagnostic naming the backend and the set index or variable.
Construction failures stop initialization; a successfully built model is
finalized once. Callers constructing a separate MIP instance pass that
backend's capability record to `TaskCreateMIPProblem` explicitly.

Availability and feature compatibility are resolved before reading a model.
Full settings verification, including numerical infinity defaults, retains its
later position. Moving those defaults before parsing changes how missing bounds
propagate and can create badly scaled finite bounds in an otherwise valid QCP.

Capability to model a semi-variable does not imply capability to relax it.
Existing relaxation limitations in the adapters are separate. Auxiliary model
builders such as the minimax interior-point solver intentionally omit some
discrete restrictions; the full-model validator must not reject a continuous
relaxation just because its source model contains an SOS set.

## Adding a feature

Add a field only when a caller needs it. Assign every field explicitly in each
backend header, including default values, in declaration order. Unsupported
features are false.
Use the enum-based lookup in common code rather than adding backend-name
matrices. Keep construction, vendor parameters, numerical acceptance rules,
and solver-specific recommendations in their appropriate backend/policy code.

`MIPCapabilities` tests cover lookup, validation, SOS-only routing,
semi-variable domains, construction failures, finalization, and unavailable
backend fallback. `StrategySelection` freezes expected routes independently of
the capability definitions. Set `SHOT_STRATEGY_SNAPSHOT=1` when running those
tests to print routing and reformulated-model statistics for comparisons.

The strategy matrix also covers explicit quadratic modes, partitioning choices,
and forced integer-bilinear reformulation across all four backends. Mixed
quadratic/cubic/quartic models are checked at every binary assignment, with
direct-solve objective values and bounds compared against exhaustive enumeration.
