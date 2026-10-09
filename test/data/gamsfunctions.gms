* The GAMS functions that SHOT builds from other expressions: each constraint uses one of them

Variables x, y, z, obj;
x.lo = 0; x.up = 0.9; y.lo = -2; y.up = 3; z.lo = 0.1; z.up = 5;
Equations e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,eobj;
e1.. arcsin(x) =l= 10;
e2.. arccos(x) =l= 10;
e3.. arctan(y) =l= 10;
e4.. sinh(y) =l= 100;
e5.. cosh(y) =l= 100;
e6.. tanh(y) =l= 10;
e7.. arctan2(y, x) =l= 10;
e8.. entropy(z) =l= 10;
e9.. centropy(x, z) =l= 10;
e10.. centropy(x, z, 0.1) =l= 10;
e11.. sigmoid(y) =l= 10;
e12.. poly(y, 1, 2, 3, 4) =l= 1000;
e13.. arctan2(x, y) =l= 10;
e14.. edist(x, y, z) + y =l= 10;
eobj.. obj =e= x + y + z;
x.l = 0.5; y.l = 1; z.l = 1;
Model m /all/;
Solve m using nlp minimizing obj;
