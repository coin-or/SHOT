g3 1 1 0	# max(x0, x1, 1 - x0) <= 1.5, min(x0, x1) >= -1, maximize max(x0, x1) - min(x0, x1)
 2 2 1 0 0 	# vars, constraints, objectives, ranges, eqns
 2 1 0 0 0 0	# nonlinear constrs, objs; ccons: lin, nonlin, nd, nzlb
 0 0	# network constraints: nonlinear, linear
 2 2 2 	# nonlinear vars in constraints, objectives, both
 0 0 0 1	# linear network variables; functions; arith, flags
 0 0 0 0 0 	# discrete variables: binary, integer, nonlinear (b,c,o)
 4 2 	# nonzeros in Jacobian, obj. gradient
 0 0	# max name lengths: constraints, variables
 0 0 0 0 0	# common exprs: b,c,o,c1,o1
C0
o12
3
v0
v1
o1
n1
v0
C1
o11
2
v0
v1
O0 1
o1
o12
2
v0
v1
o11
2
v0
v1
r
1 1.5
2 -1
b
0 -2 2
0 -2 2
k1
2
J0 2
0 0
1 0
J1 2
0 0
1 0
G0 2
0 0
1 0
