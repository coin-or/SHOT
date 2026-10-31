# SHOT documentation

SHOT is an open-source solver for mixed-integer nonlinear programming (MINLP).
These guides explain how to build a model, configure the solver, and understand
its results. {{ version_notice }}

Choose an interface to get started, or use the reference pages when you need a
specific setting or API detail.

<div class="docs-start-grid">
<a class="docs-start-card" href="getting-started.html">
<span class="label shot-eyebrow">Start here</span>
<strong>Your first model</strong>
<span>Install SHOTpy and solve a small nonlinear optimization problem.</span>
</a>
<a class="docs-start-card" href="SHOTpy_Tutorial.html">
<span class="label shot-eyebrow">Python</span>
<strong>SHOTpy tutorial</strong>
<span>Work through modeling, solver settings, callbacks, and larger problems.</span>
</a>
<a class="docs-start-card" href="CommandLineUsage.html">
<span class="label shot-eyebrow">Command line</span>
<strong>Solve a problem file</strong>
<span>Choose input formats, pass options, and run the standalone solver.</span>
</a>
<a class="docs-start-card" href="cpp.html">
<span class="label shot-eyebrow">C++</span>
<strong>Integrate SHOT</strong>
<span>Embed the solver in your application and customize its behavior.</span>
</a>
<a class="docs-start-card" href="reference/settings.html">
<span class="label shot-eyebrow">Configuration</span>
<strong>Solver settings</strong>
<span>Search options, inspect defaults, and copy settings for your runs.</span>
</a>
<a class="docs-start-card" href="DebuggingSHOT.html">
<span class="label shot-eyebrow">Troubleshooting</span>
<strong>Diagnose solver behavior</strong>
<span>Read diagnostic output and investigate unexpected results.</span>
</a>
</div>

For the complete interfaces, see the [SHOTpy API](reference/python.rst) and
[C++ API](reference/cpp.rst). To build from source, follow the
[installation guide](CompilationInstructions.md).

```{toctree}
:hidden:
:maxdepth: 2
:caption: Get started

getting-started
Installation <CompilationInstructions>
Command line <CommandLineUsage>
```

```{toctree}
:hidden:
:maxdepth: 2
:caption: Guides

SHOTpy tutorial <SHOTpy_Tutorial>
C++ integration <cpp>
Callbacks
Solver concepts <concepts>
Debugging <DebuggingSHOT>
```

```{toctree}
:hidden:
:maxdepth: 1
:caption: Reference

reference/settings
reference/python
reference/cpp
```
