C++ API
*******

These declarations are extracted from the selected public-facing headers.
Some declarations still need explanatory API comments; see the guides for usage.

Solver
------

.. doxygenclass:: SHOT::Solver
   :members:
   :undoc-members:

Settings
--------

.. doxygenclass:: SHOT::Settings
   :members:
   :undoc-members:

Results
-------

.. doxygenclass:: SHOT::Results
   :members:
   :undoc-members:

Callbacks
---------

.. doxygenfile:: Callback.h

.. _cpp-mip-solver-capabilities:

MIP solver capabilities
-----------------------

The lookup returns the capabilities of a compiled MIP backend without
constructing or licensing a solver. See :doc:`../MIPSolverCapabilities` for
field meanings and :doc:`../concepts` for usage examples.

.. doxygenstruct:: SHOT::MIPSolverCapabilities
   :members:
   :undoc-members:

.. doxygenenum:: SHOT::E_SemiVariableSupport

.. doxygenfunction:: SHOT::getMIPSolverCapabilities
