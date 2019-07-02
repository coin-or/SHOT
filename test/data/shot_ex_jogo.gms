variables x1, objvar;
integer variables x2;
equations objeq, c1, c2, l;

x1.lo = 1; x1.up = 20;
x2.lo = 1; x2.up = 20;

l.. 2*x1 -3*x2 =L= 2;
c1.. 0.15*power(x1-8,2) +0.1*power(x2-6,2) + 0.025*exp(x1)/(power(x2,2)) =L= 5;
c2.. 1/x1 + 1/x2 - x1**0.5*x2**0.5 =L= -4;
objeq.. objvar =E= -x1 -x2;

model m /ALL/;

SOLVE m USING MINLP MINIMIZING objvar; 