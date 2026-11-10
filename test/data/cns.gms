* A constrained nonlinear system (CNS): two nonlinear equations in two unknowns and no objective.
* Solutions: x*y = 1 and x^2 + y^2 = 4 at (1.9319, 0.5176) and (0.5176, 1.9319).

variables x, y;
equations circle, hyperbola;

circle.. sqr(x) + sqr(y) =e= 4;
hyperbola.. x * y =e= 1;

x.lo = 0; x.up = 3; x.l = 1.5;
y.lo = 0; y.up = 3; y.l = 1;

model m / all /;
solve m using cns;
