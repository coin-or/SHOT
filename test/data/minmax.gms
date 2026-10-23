* min and max with two and three arguments in convex and nonconvex positions; the optimum is 2.5 in (1.5, -1)
Variables x, y, obj;
x.lo = -2; x.up = 2; y.lo = -2; y.up = 2;
Equations e1, e2, eobj;
e1.. max(x, y, 1 - x) =l= 1.5;
e2.. min(x, y) =g= -1;
eobj.. obj =e= max(x, y) - min(x, y);
Model m /all/;
Solve m using dnlp maximizing obj;
