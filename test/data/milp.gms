scalar rhs;
variable z;
integer variable y;
equation a,b;

a.. z =e= y;
b.. y =l= rhs;
model m /all/;

option intvarup = 0;

y.lo = -inf;
rhs = -41.1;
solve m max z use minlp;

