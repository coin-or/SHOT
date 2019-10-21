equations objeq;
binary variables b1,b2,b3,b4;
variable objvar;

objeq.. - b1*b2*b3 + 0.1*b1*b2 -1*b1 -1*b2 -1*b3 -b4 -objvar =E= 0;

model m /ALL/;

SOLVE m USING MINLP MINIMIZING objvar; 
