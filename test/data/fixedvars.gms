* Every variable in the nonlinear/quadratic term is fixed, so the term is a
* constant and the constraint holds nothing nonlinear. Optimal objective is 1.
variable obj, x, y, z;
equation c0, c1;

c0.. x*y + z =g= 7;
c1.. obj =e= z;

model m /all/;

x.fx = 2;
y.fx = 3;
z.lo = 0;
z.up = 10;

solve m min obj use minlp;
