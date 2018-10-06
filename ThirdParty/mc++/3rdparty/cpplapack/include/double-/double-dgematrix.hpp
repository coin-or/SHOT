//=============================================================================
/*! double*dgematrix operator */
inline _dgematrix operator*(const double& d, const dgematrix& mat)
{VERBOSE_REPORT;
  dgematrix newmat(mat.m, mat.n);
  for(long i=0; i<mat.m*mat.n; i++){ newmat.array[i] =d*mat.array[i]; }
  return _(newmat);
}
